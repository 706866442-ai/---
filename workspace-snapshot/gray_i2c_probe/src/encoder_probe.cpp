#include <Arduino.h>
#include <cstdio>
#include "EncoderTransition.h"

// Confirmed by user, 2026-10-13. E1/E2 are not yet assigned left/right.
constexpr pin_size_t E1_A = PB13;
constexpr pin_size_t E1_B = PB14;
constexpr pin_size_t E2_A = PB15;
constexpr pin_size_t E2_B = PA8;
constexpr pin_size_t USER_LED = PA4;
Uart debugPort(PA3, PA2); // Optional 3.3 V USB-TTL monitor: TX on PA2.

volatile uint32_t counts[2] = {0, 0};
volatile uint32_t invalidTransitions[2] = {0, 0};
volatile uint8_t previousStates[2] = {0, 0};

// ST-LINK can read this structure while the CPU runs. Sequence must be equal
// and even before/after reading to obtain one coherent sample.
struct EncoderProbe {
    uint32_t signature;       // 0x454E4331 (ENC1), versioned layout marker
    uint32_t sequence;
    uint32_t uptimeMs;
    uint32_t e1Count;         // unsigned storage; interpret signed modulo delta
    uint32_t e2Count;
    int32_t e1Delta;
    int32_t e2Delta;
    uint32_t sampleUs;
    uint32_t e1Invalid;
    uint32_t e2Invalid;
    uint32_t e1Ab;            // A bit 1, B bit 0
    uint32_t e2Ab;
    uint32_t motorEnabled;    // always zero in this firmware
};
extern "C" {
volatile EncoderProbe g_encoderProbe = {0x454E4331, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}

uint8_t readState(unsigned int channel) {
    const pin_size_t a = channel == 0 ? E1_A : E2_A;
    const pin_size_t b = channel == 0 ? E1_B : E2_B;
    return (digitalRead(a) == HIGH ? 2u : 0u)
         | (digitalRead(b) == HIGH ? 1u : 0u);
}

void onEncoder(unsigned int channel) {
    // Either phase may trigger this callback. Prevent interleaved updates
    // when the two phase IRQs have different interrupt priorities.
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    const uint8_t current = readState(channel);
    const uint8_t previous = previousStates[channel];
    counts[channel] += static_cast<uint32_t>(encoderStep(previous, current));
    if (encoderInvalidTransition(previous, current)) ++invalidTransitions[channel];
    previousStates[channel] = current;
    __set_PRIMASK(saved);
}
void onE1() { onEncoder(0); }
void onE2() { onEncoder(1); }

void outputLow(pin_size_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

void setup() {
    // No motor-start code in this diagnostic. Disable the planned TB6612
    // STBY first, then PWM and direction lines. Do not enable driver power
    // before confirming the physical wiring and logic supply.
    outputLow(PB12);
    for (pin_size_t pin : {PA0, PA1, PB6, PB7, PB8, PB9}) outputLow(pin);
    debugPort.begin(115200);
    debugPort.println("E,ms,e1_raw,e2_raw,e1_delta,e2_delta,e1_invalid,e2_invalid,e1_ab,e2_ab");
    pinMode(USER_LED, OUTPUT);
    digitalWrite(USER_LED, HIGH);
    // No internal pull-ups: encoder output circuitry/power is not yet known.
    // Unpowered/floating inputs can generate spurious counts; they are not
    // proof of a working encoder. Validate by powered, hand-rotated wheels.
    for (pin_size_t pin : {E1_A, E1_B, E2_A, E2_B}) pinMode(pin, INPUT);
    previousStates[0] = readState(0);
    previousStates[1] = readState(1);
    attachInterrupt(digitalPinToInterrupt(E1_A), onE1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(E1_B), onE1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(E2_A), onE2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(E2_B), onE2, CHANGE);
}

void loop() {
    static uint32_t lastUs = micros();
    static uint32_t previousCounts[2] = {0, 0};
    static uint32_t lastPrintedMs = 0;
    const uint32_t nowUs = micros();
    const uint32_t elapsed = nowUs - lastUs;
    if (elapsed >= 20000u) {
        uint32_t sampleCounts[2], sampleInvalid[2];
        uint8_t sampleStates[2];
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        for (unsigned int i = 0; i < 2; ++i) {
            sampleCounts[i] = counts[i];
            sampleInvalid[i] = invalidTransitions[i];
            sampleStates[i] = readState(i);
        }
        __set_PRIMASK(saved);
        ++g_encoderProbe.sequence;
        __DMB();
        g_encoderProbe.uptimeMs = millis();
        g_encoderProbe.e1Count = sampleCounts[0];
        g_encoderProbe.e2Count = sampleCounts[1];
        g_encoderProbe.e1Delta = encoderDelta(sampleCounts[0], previousCounts[0]);
        g_encoderProbe.e2Delta = encoderDelta(sampleCounts[1], previousCounts[1]);
        g_encoderProbe.sampleUs = elapsed;
        g_encoderProbe.e1Invalid = sampleInvalid[0];
        g_encoderProbe.e2Invalid = sampleInvalid[1];
        g_encoderProbe.e1Ab = sampleStates[0];
        g_encoderProbe.e2Ab = sampleStates[1];
        __DMB();
        ++g_encoderProbe.sequence;
        previousCounts[0] = sampleCounts[0];
        previousCounts[1] = sampleCounts[1];
        lastUs = nowUs;
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintedMs >= 100u) {
            char line[112];
            const int n = snprintf(line, sizeof(line), "E,%lu,%ld,%ld,%ld,%ld,%lu,%lu,%u,%u\r\n",
                static_cast<unsigned long>(nowMs),
                static_cast<long>(static_cast<int32_t>(sampleCounts[0])),
                static_cast<long>(static_cast<int32_t>(sampleCounts[1])),
                static_cast<long>(g_encoderProbe.e1Delta),
                static_cast<long>(g_encoderProbe.e2Delta),
                static_cast<unsigned long>(sampleInvalid[0]),
                static_cast<unsigned long>(sampleInvalid[1]),
                sampleStates[0], sampleStates[1]);
            if (n > 0 && static_cast<size_t>(n) < sizeof(line)) {
                debugPort.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n));
                lastPrintedMs = nowMs;
            }
        }
    }
    // 100 ms on, 900 ms off: program heartbeat only, not encoder validation.
    digitalWrite(USER_LED, millis() % 1000u < 100u ? LOW : HIGH);
}
