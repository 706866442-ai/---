#include <Arduino.h>
#include <Wire.h>

// STM32F103 I2C2: SDA = PB11, SCL = PB10.
TwoWire sensorBus(PB11, PB10);

constexpr pin_size_t LED_PIN = PA4;

void blink(unsigned int onMs, unsigned int offMs)
{
    // This board's PA4 user LED is active-low.
    digitalWrite(LED_PIN, LOW);
    delay(onMs);
    digitalWrite(LED_PIN, HIGH);
    delay(offMs);
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // I2C uses open-drain signaling. Pull-ups belong on the sensor's 5 V bus
    // when only the 5 V-tolerant PB10/PB11 pins are connected directly.
    sensorBus.begin();
    sensorBus.setClock(100000);

    // An unmistakable boot marker, distinct from the old 0.5 s blink sketch.
    for (uint8_t n = 0; n < 3; ++n) {
        blink(100, 100);
    }
    delay(1000);
}

void loop()
{
    bool found = false;
    // Ganv's default address is 0x4C; AD0/AD1 jumpers allow 0x4D..0x4F.
    for (uint8_t address = 0x4C; address <= 0x4F; ++address) {
        sensorBus.beginTransmission(address);
        if (sensorBus.endTransmission() == 0) {
            found = true;
            break;
        }
    }

    if (found) {
        // Three short flashes followed by a long pause: I2C answered.
        for (uint8_t n = 0; n < 3; ++n) {
            blink(100, 150);
        }
        delay(2500);
    } else {
        // Three-second ON/OFF cycle: no sensor answered.
        blink(3000, 3000);
    }
}
