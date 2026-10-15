#pragma once
#include "Arduino.h"
static uint8_t mockGray=0xe7;
static bool mockWireFailure=false;
static unsigned mockGrayReads=0;
class TwoWire {
public:
    TwoWire(int,int) {}
    void begin() {} void end() {} void setClock(int) {}
    void beginTransmission(uint8_t) {}
    size_t write(uint8_t command) { command_=command;return 1; }
    uint8_t endTransmission(bool) { return mockWireFailure?4:0; }
    size_t requestFrom(uint8_t,size_t,bool) { if(command_==0xdd)++mockGrayReads;return mockWireFailure?0:1; }
    int available() { return !mockWireFailure; }
    int read() { return command_==0xaa?0x66:mockGray; }
private: uint8_t command_=0;
};
