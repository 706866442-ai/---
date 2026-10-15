#include "EncoderTransition.h"
#include <cassert>
#include <iostream>

int main() {
    uint32_t total = 0;
    uint8_t previous = 0;
    for (uint8_t state : {1, 3, 2, 0}) {
        assert(!encoderInvalidTransition(previous, state));
        total += static_cast<uint32_t>(encoderStep(previous, state));
        previous = state;
    }
    assert(total == 4);
    for (uint8_t state : {2, 3, 1, 0}) {
        total += static_cast<uint32_t>(encoderStep(previous, state));
        previous = state;
    }
    assert(total == 0);
    for (uint8_t state = 0; state < 4; ++state) {
        assert(encoderStep(state, state) == 0);
        assert(!encoderInvalidTransition(state, state));
        assert(encoderStep(state, state ^ 3) == 0);
        assert(encoderInvalidTransition(state, state ^ 3));
    }
    assert(encoderStep(0, 1) + encoderStep(1, 0) == 0); // edge bounce cancels
    assert(encoderDelta(0, 0xffffffffu) == 1);
    assert(encoderDelta(0xffffffffu, 0) == -1);
    assert(encoderDelta(100, 104) == -4);
    assert(encoderDelta(0x80000000u, 0) == (-2147483647 - 1));
    std::cout << "PASS: x4 direction, reversals, invalid edges, bounce, counter wrap\n";
}
