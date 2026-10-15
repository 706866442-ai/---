#pragma once
#include <stdint.h>

// State = (A << 1) | B. x4 decoding: count every valid A/B edge.
// 00 -> 01 -> 11 -> 10 -> 00 is positive by software convention;
// the physical vehicle-forward direction must be established on the bench.
inline int8_t encoderStep(uint8_t previous, uint8_t current) {
    static const int8_t steps[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    return steps[((previous & 3u) << 2) | (current & 3u)];
}

inline bool encoderInvalidTransition(uint8_t previous, uint8_t current) {
    return ((previous ^ current) & 3u) == 3u;
}

// Convert modulo-2^32 counter changes without signed overflow.
inline int32_t encoderDelta(uint32_t current, uint32_t previous) {
    const uint32_t delta = current - previous;
    if (delta <= 0x7fffffffu) return static_cast<int32_t>(delta);
    return -static_cast<int32_t>(0xffffffffu - delta) - 1;
}
