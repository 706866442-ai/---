#pragma once
#include <cstdint>
#include <cstddef>
#include <deque>
#include <string>
#include <sstream>
#include <algorithm>
#include <cstdlib>
using pin_size_t = int;
enum { PA0=0, PA1, PA2, PA3, PA4, PA5, PA8=8, PB6=22, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15 };
enum { LOW=0, HIGH=1, INPUT=0, INPUT_PULLUP=2, OUTPUT=3, OUTPUT_OPEN_DRAIN=4, CHANGE=5 };
static uint32_t mockMs=0;
static int pinLevels[64], pwmLevels[64];
inline uint32_t millis() { return mockMs; }
inline uint32_t micros() { return mockMs * 1000; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(int, int) {}
inline int digitalRead(int p) { return pinLevels[p]; }
inline void digitalWrite(int p, int v) { pinLevels[p]=v; }
inline void analogWrite(int p, int v) { pwmLevels[p]=v; }
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
#define constrain(v,lo,hi) ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
struct MockI2C { uint32_t CR1=0, SR2=0; };
static MockI2C mockI2C;
#define I2C2 (&mockI2C)
#define I2C_CR1_PE 1
#define I2C_CR1_SWRST 2
#define I2C_SR2_BUSY 4
#define CLEAR_BIT(reg,bit) ((reg)&=~(bit))
#define SET_BIT(reg,bit) ((reg)|=(bit))
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)
enum { GPIO_PIN_10=1, GPIO_PIN_11=2, GPIO_MODE_AF_OD=3, GPIO_NOPULL=0, GPIO_SPEED_FREQ_HIGH=1 };
struct GPIO_InitTypeDef { int Pin=0,Mode=0,Pull=0,Speed=0; };
#define GPIOB nullptr
inline void HAL_GPIO_Init(void*,GPIO_InitTypeDef*) {}
class Uart {
public:
    Uart(int,int) {}
    void begin(int) {}
    int available() const { return static_cast<int>(rx.size()); }
    int read() { if(rx.empty()) return -1; int b=rx.front();rx.pop_front();return b; }
    int availableForWrite() const { return room; }
    size_t write(const uint8_t *p,size_t n) {
        assertWriteBudget = std::max(assertWriteBudget, n);
        const size_t sent=std::min(n,static_cast<size_t>(room));
        tx.append(reinterpret_cast<const char*>(p),sent); return sent;
    }
    template<class T> void print(T value) { std::ostringstream s;s<<value;tx+=s.str(); }
    template<class T> void println(T value) { print(value);tx+='\n'; }
    std::deque<uint8_t> rx; std::string tx;
    int room=64; size_t assertWriteBudget=0;
};
