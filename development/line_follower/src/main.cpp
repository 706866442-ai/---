#include <Arduino.h>
#include <Wire.h>
#include "RobotConfig.h"
#include "Parking.h"
#include "Route.h"
#include "WheelSpeedControl.h"

// C03C STM32F103C8T6 board and the previously assigned TB6612 wiring.
constexpr pin_size_t LED = PA4;          // Active-low user LED.
constexpr pin_size_t KEY = PA5;          // Assumed active-low user button.
constexpr pin_size_t PWMA = PA0;
constexpr pin_size_t PWMB = PA1;
constexpr pin_size_t AIN1 = PB6;
constexpr pin_size_t AIN2 = PB7;
constexpr pin_size_t BIN1 = PB8;
constexpr pin_size_t BIN2 = PB9;
constexpr pin_size_t STBY = PB12;

// Change these after watching the brief motor test with the wheels raised.
constexpr bool A_FORWARD_IN1_HIGH = cfg::A_FORWARD_IN1_HIGH;
constexpr bool B_FORWARD_IN1_HIGH = cfg::B_FORWARD_IN1_HIGH;
constexpr bool CHANNEL_1_IS_LEFT = cfg::CHANNEL_1_IS_LEFT;

constexpr uint8_t FIRST_GRAY_ADDRESS = 0x4C;
constexpr uint8_t LAST_GRAY_ADDRESS = 0x4F;
constexpr uint8_t GRAY_PING = 0xAA;
constexpr uint8_t GRAY_DIGITAL = 0xDD;
constexpr uint8_t TEST_PWM = 75;       // 0..255, deliberately modest.
constexpr uint8_t BASE_PWM = 75;
constexpr uint8_t MAX_PWM = 120;
constexpr int16_t MAX_CORRECTION = 45;
constexpr uint32_t CONTROL_PERIOD_MS = 40;
constexpr uint32_t MAX_RUN_MS = 10000; // One trial lasts at most 10 seconds.
constexpr uint32_t SPEED_PERIOD_MS = 100;

TwoWire grayBus(PB11, PB10); // SDA, SCL: STM32 I2C2.
Uart debugPort(PA3, PA2);   // Optional 3.3 V UART, TX on PA2.

enum class State : uint8_t {
    WaitForRelease,
    Ready,
    Pressing,
    TestA,
    TestGap,
    TestB,
    Follow,
    Route,
    Parking,
    Fault
};

enum class Mode : uint8_t { Follow, FollowAB, FollowLap, ReverseBay, ParallelBay };
Mode selectedMode = Mode::Follow;
parking::Controller parkingController;
route::Controller routeController;
drive::WheelSpeedControl speedController(
    {{cfg::SPEED_KP, cfg::SPEED_KI, cfg::SPEED_KD}, cfg::LEFT_MM_PER_COUNT},
    {{cfg::SPEED_KP, cfg::SPEED_KI, cfg::SPEED_KD}, cfg::RIGHT_MM_PER_COUNT});
uint32_t lastSpeedMs = 0, lastEncoderMotionMs = 0;
int32_t previousE1 = 0, previousE2 = 0;
uint32_t initialSpeedErrors = 0;
volatile uint32_t encoder1 = 0, encoder2 = 0;
volatile uint32_t invalid1 = 0, invalid2 = 0;
volatile uint8_t previous1 = 0, previous2 = 0;
constexpr int8_t QUAD[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
uint8_t phase(pin_size_t a, pin_size_t b) {
    return (digitalRead(a) == HIGH ? 2 : 0) | (digitalRead(b) == HIGH ? 1 : 0);
}
void encoder1Changed() {
    const uint8_t p = phase(PB13, PB14);
    if ((p ^ previous1) == 3) ++invalid1;
    encoder1 += QUAD[(previous1 << 2) | p]; previous1 = p;
}
void encoder2Changed() {
    const uint8_t p = phase(PB15, PA8);
    if ((p ^ previous2) == 3) ++invalid2;
    encoder2 += QUAD[(previous2 << 2) | p]; previous2 = p;
}
void encoderSnapshot(int32_t &e1, int32_t &e2, uint32_t &errors) {
    noInterrupts();
    e1 = static_cast<int32_t>(encoder1); e2 = static_cast<int32_t>(encoder2);
    errors = invalid1 + invalid2;
    interrupts();
}
void wheelDistances(float &left, float &right, uint32_t &errors) {
    int32_t e1, e2; encoderSnapshot(e1, e2, errors);
    const float first = static_cast<float>(e1) * cfg::E1_FORWARD_SIGN;
    const float second = static_cast<float>(e2) * cfg::E2_FORWARD_SIGN;
    left = (cfg::E1_IS_RIGHT ? second : first) * cfg::LEFT_MM_PER_COUNT;
    right = (cfg::E1_IS_RIGHT ? first : second) * cfg::RIGHT_MM_PER_COUNT;
}
uint32_t parkingInitialErrors = 0;

bool buttonPressed(uint32_t now) {
    static bool raw = false, stable = false;
    static uint32_t changedAt = 0;
    const bool sample = digitalRead(KEY) == LOW;
    if (sample != raw) { raw = sample; changedAt = now; }
    if (now - changedAt >= 30) stable = raw;
    return stable;
}

State state = State::WaitForRelease;
uint32_t stateSinceMs = 0;
uint32_t releasedSinceMs = 0;
uint32_t lastControlMs = 0;
uint8_t grayAddress = 0;
enum class FaultCode : uint8_t {
    SensorMissing = 1, ReadFailed = 2, AllWhite = 3,
    AllBlack = 4, Configuration = 5, Parking = 6
};
// Persistent RAM diagnostics, readable over SWD without a USB-UART adapter:
// version, fault, address, last digital byte, digital valid, I2C TX status, RX length.
volatile uint8_t grayDiagnostics[7] = {1, 0, 0, 0, 0, 0, 0};
volatile uint32_t grayRecoveryCount = 0;
volatile uint8_t grayRecoveryFailed = 0;

// The STM32duino core rounds setClock(80000) to 100 kHz. PCLK1 is 36 MHz here.
// Program the F1 CCR directly for an actual 80 kHz bus (ST ES096 section 2.8.5).
void setGrayClock80k() {
    grayBus.setClock(80000);
    CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
    I2C2->CCR = 225;
    I2C2->TRISE = 37;
    SET_BIT(I2C2->CR1, I2C_CR1_PE);
}

bool waitBusLevel(pin_size_t pin, uint32_t level) {
    const uint32_t started = micros();
    while (digitalRead(pin) != level) {
        if (micros() - started >= 1000) return false;
    }
    delayMicroseconds(5);
    return true;
}

// STM32F103 ES096 section 2.8.7: update both analog input filters before SWRST.
// Only called with motors disabled, never as an automatic retry while driving.
bool recoverGrayBus() {
    ++grayRecoveryCount;
    grayBus.end();
    __HAL_RCC_I2C2_CLK_ENABLE();
    CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
    digitalWrite(PB10, HIGH); digitalWrite(PB11, HIGH);
    pinMode(PB10, OUTPUT_OPEN_DRAIN); pinMode(PB11, OUTPUT_OPEN_DRAIN);
    bool ok = waitBusLevel(PB10, HIGH) && waitBusLevel(PB11, HIGH);
    if (ok) { digitalWrite(PB11, LOW); ok = waitBusLevel(PB11, LOW); }
    if (ok) { digitalWrite(PB10, LOW); ok = waitBusLevel(PB10, LOW); }
    if (ok) { digitalWrite(PB10, HIGH); ok = waitBusLevel(PB10, HIGH); }
    if (ok) { digitalWrite(PB11, HIGH); ok = waitBusLevel(PB11, HIGH); }
    // Always release the bus, even if an external device holds a line low.
    digitalWrite(PB10, HIGH); digitalWrite(PB11, HIGH);
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    SET_BIT(I2C2->CR1, I2C_CR1_SWRST);
    delayMicroseconds(5);
    CLEAR_BIT(I2C2->CR1, I2C_CR1_SWRST);
    grayBus.begin();
    setGrayClock80k();
    ok = ok && !(I2C2->SR2 & I2C_SR2_BUSY);
    grayRecoveryFailed = ok ? 0 : 1;
    return ok;
}

void stopMotors()
{
    // Disable bridge before touching direction or PWM pins.
    digitalWrite(STBY, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void setForward(uint8_t leftPwm, uint8_t rightPwm)
{
    digitalWrite(STBY, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, A_FORWARD_IN1_HIGH ? HIGH : LOW);
    digitalWrite(AIN2, A_FORWARD_IN1_HIGH ? LOW : HIGH);
    digitalWrite(BIN1, B_FORWARD_IN1_HIGH ? HIGH : LOW);
    digitalWrite(BIN2, B_FORWARD_IN1_HIGH ? LOW : HIGH);
    analogWrite(PWMA, leftPwm);
    analogWrite(PWMB, rightPwm);
    digitalWrite(STBY, HIGH);
}

void updateForwardPwm(uint8_t leftPwm, uint8_t rightPwm)
{
    analogWrite(PWMA, cfg::A_IS_RIGHT ? rightPwm : leftPwm);
    analogWrite(PWMB, cfg::A_IS_RIGHT ? leftPwm : rightPwm);
}

void setWheels(int left, int right) {
    const int a = constrain(cfg::A_IS_RIGHT ? right : left, -MAX_PWM, MAX_PWM);
    const int b = constrain(cfg::A_IS_RIGHT ? left : right, -MAX_PWM, MAX_PWM);
    if (a == 0 && b == 0) { stopMotors(); return; }
    digitalWrite(STBY, LOW);
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    const bool aHigh = (a >= 0) == cfg::A_FORWARD_IN1_HIGH;
    const bool bHigh = (b >= 0) == cfg::B_FORWARD_IN1_HIGH;
    digitalWrite(AIN1, aHigh ? HIGH : LOW); digitalWrite(AIN2, aHigh ? LOW : HIGH);
    digitalWrite(BIN1, bHigh ? HIGH : LOW); digitalWrite(BIN2, bHigh ? LOW : HIGH);
    analogWrite(PWMA, abs(a)); analogWrite(PWMB, abs(b));
    digitalWrite(STBY, HIGH);
}

bool readGrayCommand(uint8_t address, uint8_t command, uint8_t &value)
{
    grayDiagnostics[6] = 0;
    if (command == GRAY_DIGITAL) grayDiagnostics[4] = 0;
    grayBus.beginTransmission(address);
    const size_t written = grayBus.write(command);
    const uint8_t txStatus = grayBus.endTransmission(false);
    grayDiagnostics[5] = txStatus;
    if (written != 1 || txStatus != 0) {
        return false;
    }
    const size_t received = grayBus.requestFrom(address, static_cast<size_t>(1), true);
    grayDiagnostics[6] = static_cast<uint8_t>(received);
    if (received != 1 ||
        !grayBus.available()) {
        return false;
    }
    value = static_cast<uint8_t>(grayBus.read());
    if (command == GRAY_DIGITAL) {
        grayDiagnostics[3] = value;
        grayDiagnostics[4] = 1;
    }
    return true;
}

uint8_t findGrayBoard()
{
    uint8_t answer = 0;
    for (uint8_t address = FIRST_GRAY_ADDRESS;
         address <= LAST_GRAY_ADDRESS; ++address) {
        if ((I2C2->SR2 & I2C_SR2_BUSY) && !recoverGrayBus()) return 0;
        if (readGrayCommand(address, GRAY_PING, answer) && answer == 0x66) {
            return address;
        }
    }
    return 0;
}

bool lineErrorFromDigital(uint8_t digital, int16_t &error)
{
    // Sensor manual: 1=white, 0=black; bit 0 is channel 1.
    int32_t weighted = 0;
    uint8_t blackCount = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if ((digital & (1U << i)) == 0) {
            int16_t position = static_cast<int16_t>((2 * i - 7) * 500);
            if (!CHANNEL_1_IS_LEFT) {
                position = -position;
            }
            weighted += position;
            ++blackCount;
        }
    }
    // No line or an all-black field is ambiguous: stop instead of guessing.
    if (blackCount == 0 || blackCount == 8) {
        return false;
    }
    error = static_cast<int16_t>(weighted / blackCount); // -3500..3500.
    return true;
}

void enterState(State next, uint32_t now)
{
    stopMotors();
    parkingController.stop();
    routeController.stop();
    speedController.reset();
    state = next;
    stateSinceMs = now;
    if (next == State::WaitForRelease) releasedSinceMs = 0;
}

void fault(const char *message, uint32_t now, FaultCode code = FaultCode::Parking)
{
    enterState(State::Fault, now);
    grayDiagnostics[1] = static_cast<uint8_t>(code);
    debugPort.println(message);
}

bool readLineOrFault(int16_t &error, uint32_t now) {
    uint8_t digital = 0;
    if (!readGrayCommand(grayAddress, GRAY_DIGITAL, digital)) {
        fault("STOP 2: gray digital read failed", now, FaultCode::ReadFailed);
        return false;
    }
    if (!lineErrorFromDigital(digital, error)) {
        const bool white = digital == 0xFF;
        fault(white ? "STOP 3: all white" : "STOP 4: all black", now,
              white ? FaultCode::AllWhite : FaultCode::AllBlack);
        return false;
    }
    return true;
}

void startParking(uint32_t now) {
    if (!cfg::MOTOR_DIRECTIONS_VERIFIED || !cfg::ENCODER_INPUTS_ENABLED ||
        !cfg::ENCODERS_VERIFIED ||
        !cfg::PARKING_START_POSE_VERIFIED || !parking::positive(cfg::LEFT_MM_PER_COUNT) ||
        !parking::positive(cfg::RIGHT_MM_PER_COUNT) ||
        (cfg::E1_FORWARD_SIGN != 1 && cfg::E1_FORWARD_SIGN != -1) ||
        (cfg::E2_FORWARD_SIGN != 1 && cfg::E2_FORWARD_SIGN != -1)) {
        fault("STOP: parking calibration/pose not verified", now, FaultCode::Configuration); return;
    }
    parking::Plan plan;
    const bool valid = selectedMode == Mode::ReverseBay
        ? parking::reverseBay(plan, cfg::TRACK_MM, cfg::REVERSE_DEPTH_MM, cfg::BAY_SIDE)
        : parking::parallelBay(plan, cfg::TRACK_MM, cfg::PARALLEL_RADIUS_MM,
                               cfg::PARALLEL_ANGLE_DEG, cfg::PARALLEL_TRIM_MM, cfg::BAY_SIDE);
    float left, right; wheelDistances(left, right, parkingInitialErrors);
    if (!valid || !parkingController.start(plan, now, left, right)) {
        fault("STOP: invalid parking geometry", now); return;
    }
    state = State::Parking; stateSinceMs = now; lastControlMs = now;
    debugPort.println("PARKING: encoder motion from calibrated starting pose");
}

void startRoute(uint32_t now) {
    if (!cfg::MOTOR_DIRECTIONS_VERIFIED || !cfg::SENSOR_ORDER_VERIFIED ||
        !cfg::ENCODER_INPUTS_ENABLED || !cfg::ENCODERS_VERIFIED ||
        !cfg::ROUTE_START_POSE_VERIFIED ||
        !parking::positive(cfg::LEFT_MM_PER_COUNT) ||
        !parking::positive(cfg::RIGHT_MM_PER_COUNT) ||
        (selectedMode == Mode::FollowLap &&
         cfg::ROUTE_TURN_SIGN != 1 && cfg::ROUTE_TURN_SIGN != -1)) {
        fault("STOP: route calibration/pose not verified", now, FaultCode::Configuration);
        return;
    }
    grayAddress = findGrayBoard();
    grayDiagnostics[2] = grayAddress;
    if (!grayAddress) {
        fault("STOP: route gray board missing", now, FaultCode::SensorMissing);
        return;
    }
    float left, right; uint32_t errors;
    wheelDistances(left, right, errors);
    parkingInitialErrors = errors;
    route::Config config;
    config.sideMm = cfg::ROUTE_SIDE_MM;
    config.wheelTrackMm = cfg::TRACK_MM;
    config.turnSign = cfg::ROUTE_TURN_SIGN;
    config.maxTaskMs = selectedMode == Mode::FollowAB ? 10000 : 30000;
    if (!routeController.start(selectedMode == Mode::FollowAB
            ? route::Task::AB : route::Task::Lap, config, now, left, right)) {
        fault("STOP: invalid route geometry", now, FaultCode::Configuration);
        return;
    }
    state = State::Route;
    stateSinceMs = now;
    lastControlMs = now - CONTROL_PERIOD_MS;
    debugPort.println("ROUTE: AB or four legs, watch for corner marks");
}

void showStatus(uint32_t now)
{
    bool on = false;
    switch (state) {
        case State::Ready:
        case State::WaitForRelease:
            // One to five pulses per 2s identify the selected operating mode.
            on = (now % 2000UL) / 200UL <= static_cast<uint8_t>(selectedMode) &&
                 (now % 200UL) < 60UL;
            break;
        case State::Pressing:
            on = true;
            break;
        case State::TestA:
        case State::TestGap:
        case State::TestB:
        case State::Follow:
        case State::Route:
        case State::Parking:
            on = (now % 250UL) < 40UL;
            break;
        case State::Fault:
            // Every 4s: 800ms header, 400ms gap, then N short reason flashes.
            // Latch until button acknowledgement, so the reason is not lost.
            {
                const uint32_t phase = (now - stateSinceMs) % 4000UL;
                on = phase < 800UL ||
                     (phase >= 1200UL &&
                      (phase - 1200UL) / 300UL < grayDiagnostics[1] &&
                      (phase - 1200UL) % 300UL < 120UL);
            }
            break;
    }
    digitalWrite(LED, on ? LOW : HIGH);
}

void setup()
{
    // Establish the stop state before initializing communication.
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, LOW);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    stopMotors();

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    pinMode(KEY, INPUT_PULLUP);
    grayBus.begin();
    setGrayClock80k();
    debugPort.begin(115200);
    pinMode(PB13, INPUT); pinMode(PB14, INPUT);
    pinMode(PB15, INPUT); pinMode(PA8, INPUT);
    if (cfg::ENCODER_INPUTS_ENABLED) {
        previous1 = phase(PB13, PB14); previous2 = phase(PB15, PA8);
        attachInterrupt(digitalPinToInterrupt(PB13), encoder1Changed, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PB14), encoder1Changed, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PB15), encoder2Changed, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PA8), encoder2Changed, CHANGE);
    }
    debugPort.println("LINE FOLLOWER READY: release button first");
    // Read-only startup check; motor outputs remain stopped throughout.
    grayAddress = findGrayBoard();
    grayDiagnostics[2] = grayAddress;
    if (grayAddress != 0) {
        uint8_t initialDigital = 0;
        readGrayCommand(grayAddress, GRAY_DIGITAL, initialDigital);
    }
}

void loop()
{
    const uint32_t now = millis();
    const bool pressed = buttonPressed(now);
    static uint32_t lastDiagnosticMs = 0;
    if (cfg::ENCODER_INPUTS_ENABLED && now - lastDiagnosticMs >= 500) {
        lastDiagnosticMs = now;
        int32_t e1, e2; uint32_t errors; encoderSnapshot(e1, e2, errors);
        debugPort.print("E1="); debugPort.print(e1);
        debugPort.print(" E2="); debugPort.print(e2);
        debugPort.print(" invalid="); debugPort.println(errors);
    }

    switch (state) {
        case State::WaitForRelease:
            if (pressed) {
                releasedSinceMs = 0;
            } else if (releasedSinceMs == 0) {
                releasedSinceMs = now;
            } else if (now - releasedSinceMs >= 500) {
                state = State::Ready;
                debugPort.println("READY: tap=mode, 1s=wheel test, >=2s=selected mode");
            }
            break;

        case State::Ready:
            if (pressed) {
                state = State::Pressing;
                stateSinceMs = now;
            }
            break;

        case State::Pressing:
            if (!pressed) {
                const uint32_t heldMs = now - stateSinceMs;
                if (heldMs >= 2000) {
                    grayDiagnostics[1] = 0;
                    grayDiagnostics[2] = 0;
                    grayDiagnostics[4] = 0;
                    if (selectedMode == Mode::ReverseBay ||
                        selectedMode == Mode::ParallelBay) {
                        startParking(now); break;
                    }
                    if (selectedMode == Mode::FollowAB ||
                        selectedMode == Mode::FollowLap) {
                        startRoute(now); break;
                    }
                    if (!cfg::MOTOR_DIRECTIONS_VERIFIED || !cfg::SENSOR_ORDER_VERIFIED) {
                        fault("STOP: verify wheel forward direction and sensor order", now, FaultCode::Configuration); break;
                    }
                    if (cfg::SPEED_PID_ENABLED &&
                        (!cfg::ENCODER_INPUTS_ENABLED || !cfg::ENCODERS_VERIFIED ||
                         !parking::positive(cfg::LEFT_MM_PER_COUNT) ||
                         !parking::positive(cfg::RIGHT_MM_PER_COUNT) ||
                         !parking::positive(cfg::SPEED_BASE_MMPS) ||
                         !parking::positive(cfg::SPEED_FEEDFORWARD_PWM))) {
                        fault("STOP: speed PID requires measured encoder/calibration", now,
                              FaultCode::Configuration); break;
                    }
                    grayAddress = findGrayBoard();
                    grayDiagnostics[2] = grayAddress;
                    int16_t error = 0;
                    if (grayAddress == 0) {
                        fault("STOP 1: no gray ping response", now, FaultCode::SensorMissing);
                    } else if (readLineOrFault(error, now)) {
                        setForward(BASE_PWM, BASE_PWM);
                        state = State::Follow;
                        stateSinceMs = now;
                        lastControlMs = now - CONTROL_PERIOD_MS;
                        lastSpeedMs = lastEncoderMotionMs = now;
                        encoderSnapshot(previousE1, previousE2, initialSpeedErrors);
                        speedController.reset();
                        debugPort.println("FOLLOW: max 10 s, press button to stop");
                    }
                } else if (heldMs >= 500) {
                    setForward(TEST_PWM, 0);
                    state = State::TestA;
                    stateSinceMs = now;
                    debugPort.println("TEST: channel A forward");
                } else {
                    if (heldMs >= 60) {
                        selectedMode = static_cast<Mode>((static_cast<uint8_t>(selectedMode) + 1) % 5);
                        debugPort.print("MODE (0=test 1=AB 2=lap 3=reverse 4=parallel): ");
                        debugPort.println(static_cast<uint8_t>(selectedMode));
                    }
                    state = State::Ready;
                }
            }
            break;

        case State::TestA:
            if (pressed) {
                enterState(State::WaitForRelease, now);
            } else if (now - stateSinceMs >= 400) {
                enterState(State::TestGap, now);
            }
            break;

        case State::TestGap:
            if (pressed) {
                enterState(State::WaitForRelease, now);
            } else if (now - stateSinceMs >= 700) {
                setForward(0, TEST_PWM);
                state = State::TestB;
                stateSinceMs = now;
                debugPort.println("TEST: channel B forward");
            }
            break;

        case State::TestB:
            if (pressed || now - stateSinceMs >= 400) {
                enterState(State::WaitForRelease, now);
                debugPort.println("TEST: finished, motors stopped");
            }
            break;

        case State::Follow:
            if (pressed || now - stateSinceMs >= MAX_RUN_MS) {
                enterState(State::WaitForRelease, now);
                debugPort.println("STOP: button or 10 s limit");
            } else if (now - lastControlMs >= CONTROL_PERIOD_MS) {
                lastControlMs = now;
                int16_t error = 0;
                if (readLineOrFault(error, now)) {
                    if (cfg::SPEED_PID_ENABLED) {
                        if (now - lastSpeedMs >= SPEED_PERIOD_MS) {
                            int32_t e1, e2; uint32_t errors;
                            encoderSnapshot(e1, e2, errors);
                            const int32_t first = e1 - previousE1;
                            const int32_t second = e2 - previousE2;
                            previousE1 = e1; previousE2 = e2;
                            const int32_t leftCounts = (cfg::E1_IS_RIGHT ? second : first) *
                                                       (cfg::E1_IS_RIGHT ? cfg::E2_FORWARD_SIGN : cfg::E1_FORWARD_SIGN);
                            const int32_t rightCounts = (cfg::E1_IS_RIGHT ? first : second) *
                                                        (cfg::E1_IS_RIGHT ? cfg::E1_FORWARD_SIGN : cfg::E2_FORWARD_SIGN);
                            if (leftCounts || rightCounts) lastEncoderMotionMs = now;
                            if (now - lastEncoderMotionMs > 1000 ||
                                errors - initialSpeedErrors > 8) {
                                fault("STOP: speed PID encoder stalled/invalid", now);
                                break;
                            }
                            const float correction = cfg::SPEED_LINE_CORRECTION_MMPS *
                                                     error / 3500.0f;
                            const float leftTarget = constrain(cfg::SPEED_BASE_MMPS + correction,
                                                               0.0f, 2 * cfg::SPEED_BASE_MMPS);
                            const float rightTarget = constrain(cfg::SPEED_BASE_MMPS - correction,
                                                                0.0f, 2 * cfg::SPEED_BASE_MMPS);
                            const drive::DriveSample measured = speedController.update(
                                leftTarget, rightTarget, leftCounts, rightCounts,
                                (now - lastSpeedMs) * 0.001f, true);
                            lastSpeedMs = now;
                            if (!measured.valid) {
                                fault("STOP: invalid speed PID sample", now);
                                break;
                            }
                            const uint8_t leftPwm = static_cast<uint8_t>(constrain(
                                cfg::SPEED_FEEDFORWARD_PWM + measured.leftPwm, 0.0f,
                                static_cast<float>(MAX_PWM)));
                            const uint8_t rightPwm = static_cast<uint8_t>(constrain(
                                cfg::SPEED_FEEDFORWARD_PWM + measured.rightPwm, 0.0f,
                                static_cast<float>(MAX_PWM)));
                            updateForwardPwm(leftPwm, rightPwm);
                        }
                    } else {
                        const int16_t correction = static_cast<int32_t>(error) *
                                                   MAX_CORRECTION / 3500;
                        const int16_t left = constrain(BASE_PWM + correction, 0, MAX_PWM);
                        const int16_t right = constrain(BASE_PWM - correction, 0, MAX_PWM);
                        updateForwardPwm(static_cast<uint8_t>(left),
                                         static_cast<uint8_t>(right));
                    }
                }
            }
            break;

        case State::Route:
            // The raw button is an immediate stop; slower debouncing is used
            // only for selecting/starting a mode while stationary.
            if (digitalRead(KEY) == LOW) {
                enterState(State::WaitForRelease, now);
                debugPort.println("STOP: route cancelled");
            } else if (now - lastControlMs >= CONTROL_PERIOD_MS) {
                lastControlMs = now;
                uint8_t digital = 0;
                if (!readGrayCommand(grayAddress, GRAY_DIGITAL, digital)) {
                    fault("STOP: route gray read failed", now, FaultCode::ReadFailed);
                    break;
                }
                float left, right; uint32_t errors;
                wheelDistances(left, right, errors);
                if (errors - parkingInitialErrors > 8) {
                    fault("STOP: route encoder transitions invalid", now);
                    break;
                }
                const route::Result output = routeController.update(digital, now, left, right);
                if (output.status == route::Status::Complete) {
                    const route::StopPoint point = output.stopPoint;
                    enterState(State::WaitForRelease, now);
                    if (point == route::StopPoint::B) {
                        debugPort.println("ROUTE: point B reached, motors stopped");
                    } else if (point == route::StopPoint::A) {
                        debugPort.println("ROUTE: point A reached, motors stopped");
                    }
                } else if (output.status == route::Status::Fault) {
                    fault("STOP: route lost marker/line or timed out", now);
                } else if (output.status == route::Status::Turn) {
                    setWheels(output.leftPwm, output.rightPwm);
                } else if (digital == 0x00) {
                    // Only permitted for a short start mark by Route::update.
                    setWheels(45, 45);
                } else {
                    int16_t error = 0;
                    if (!lineErrorFromDigital(digital, error)) {
                        fault("STOP: route line position invalid", now);
                        break;
                    }
                    const int basePwm = output.approachingPoint ? 45 : BASE_PWM;
                    const int16_t correction = static_cast<int32_t>(error) *
                                               MAX_CORRECTION / 3500;
                    setWheels(constrain(basePwm + correction, 0, MAX_PWM),
                              constrain(basePwm - correction, 0, MAX_PWM));
                }
            }
            break;

        case State::Parking:
            // React to raw press immediately for stop; start uses debounced input.
            if (digitalRead(KEY) == LOW) {
                enterState(State::WaitForRelease, now);
                debugPort.println("STOP: parking cancelled");
            } else if (now - lastControlMs >= 20) {
                lastControlMs = now;
                float left, right; uint32_t errors; wheelDistances(left, right, errors);
                if (errors - parkingInitialErrors > 8) {
                    fault("STOP: invalid encoder transitions", now); break;
                }
                const parking::Output output = parkingController.update(now, left, right);
                const parking::Status status = parkingController.status;
                if (status == parking::Status::Complete) {
                    enterState(State::WaitForRelease, now);
                    debugPort.println("PARKING: distance sequence finished; inspect actual pose");
                } else if (status != parking::Status::Running) {
                    debugPort.print("PARKING fault code: "); debugPort.println(static_cast<int>(status));
                    fault("STOP: parking feedback/timeout fault", now);
                } else setWheels(output.left, output.right);
            }
            break;

        case State::Fault:
            if (pressed) {
                enterState(State::WaitForRelease, now);
            }
            break;
    }

    showStatus(now);
}
