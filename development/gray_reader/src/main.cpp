#include <Arduino.h>
#include <Wire.h>

// Ganv AFA07-S16FCC-00 Rev3.6: I2C2 SDA=PB11, SCL=PB10.
TwoWire grayBus(PB11, PB10);
// Optional 3.3 V USB-UART monitor: STM32 PA2 (TX) -> adapter RX.
Uart debugPort(PA3, PA2);

constexpr pin_size_t USER_LED = PA4; // C03C board: active low.
constexpr uint8_t FIRST_ADDRESS = 0x4C;
constexpr uint8_t LAST_ADDRESS = 0x4F;
constexpr uint8_t COMMAND_PING = 0xAA;
constexpr uint8_t COMMAND_DIGITAL = 0xDD;
constexpr uint8_t COMMAND_EIGHT_ANALOG = 0xB0;
constexpr uint32_t SAMPLE_PERIOD_MS = 250;

struct GrayFrame {
    uint8_t digital;
    uint8_t analog[8];
};

uint8_t grayAddress = 0;
uint32_t lastSampleMs = 0;
uint32_t lastMessageMs = 0;

void ledOn(bool on)
{
    digitalWrite(USER_LED, on ? LOW : HIGH);
}

void flashLed(uint16_t onMs, uint16_t offMs)
{
    ledOn(true);
    delay(onMs);
    ledOn(false);
    delay(offMs);
}

// A command write followed by a repeated-start read, as in manual sections
// 7.7 and 7.9. Check both stages before exposing data to the application.
bool readCommand(uint8_t address, uint8_t command, uint8_t *data, size_t length)
{
    grayBus.beginTransmission(address);
    if (grayBus.write(command) != 1 || grayBus.endTransmission(false) != 0) {
        return false;
    }

    const size_t received = grayBus.requestFrom(address, length, true);
    if (received != length) {
        while (grayBus.available()) {
            grayBus.read();
        }
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        if (!grayBus.available()) {
            return false;
        }
        data[i] = static_cast<uint8_t>(grayBus.read());
    }
    return true;
}

uint8_t findGrayBoard()
{
    uint8_t reply = 0;
    for (uint8_t address = FIRST_ADDRESS; address <= LAST_ADDRESS; ++address) {
        if (readCommand(address, COMMAND_PING, &reply, 1) && reply == 0x66) {
            return address;
        }
    }
    return 0;
}

bool readFrame(uint8_t address, GrayFrame &frame)
{
    return readCommand(address, COMMAND_DIGITAL, &frame.digital, 1) &&
           readCommand(address, COMMAND_EIGHT_ANALOG, frame.analog, 8);
}

void printFrame(uint8_t address, const GrayFrame &frame)
{
    debugPort.print("ADDR=0x");
    debugPort.print(address, HEX);
    debugPort.print(" DIGITAL[1..8]=");
    for (uint8_t i = 0; i < 8; ++i) {
        // Section 7.7: bit 0 is channel 1; high=white, low=black.
        debugPort.print((frame.digital & (1U << i)) ? 'W' : 'B');
    }
    debugPort.print(" ANALOG[1..8]=");
    for (uint8_t i = 0; i < 8; ++i) {
        if (i) {
            debugPort.print(',');
        }
        debugPort.print(frame.analog[i]);
    }
    debugPort.println();
}

void setup()
{
    pinMode(USER_LED, OUTPUT);
    ledOn(false);
    debugPort.begin(115200);
    grayBus.begin();
    grayBus.setClock(100000);

    for (uint8_t i = 0; i < 3; ++i) {
        flashLed(100, 100);
    }
    debugPort.println("GRAY READER: waiting for 0x4C..0x4F");
}

void loop()
{
    const uint32_t now = millis();
    if (now - lastSampleMs < SAMPLE_PERIOD_MS) {
        return;
    }
    lastSampleMs = now;

    if (grayAddress == 0) {
        grayAddress = findGrayBoard();
        if (grayAddress == 0) {
            // Slow 3 s on / 3 s off: no valid ping response.
            ledOn((now % 6000UL) < 3000UL);
            if (now - lastMessageMs >= 1000UL) {
                debugPort.println("WAIT: no gray board ping response");
                lastMessageMs = now;
            }
            return;
        }
        debugPort.print("CONNECTED: 0x");
        debugPort.println(grayAddress, HEX);
    }

    GrayFrame frame{};
    if (!readFrame(grayAddress, frame)) {
        debugPort.println("READ ERROR: will rescan gray board");
        grayAddress = 0;
        ledOn(false);
        return;
    }

    printFrame(grayAddress, frame);
    // A short heartbeat every 250 ms means both 8-channel reads succeeded.
    flashLed(35, 0);
}
