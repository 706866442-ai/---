#pragma once
#include <cstdint>
#include <cstddef>
namespace telemetry {
struct Sample {
    uint32_t ms = 0, errors = 0, grayAgeMs = 0;
    int32_t e1 = 0, e2 = 0;
    int16_t lineError = 0, leftTarget = 0, rightTarget = 0;
    int16_t leftSpeed = 0, rightSpeed = 0, leftPwm = 0, rightPwm = 0;
    uint8_t state = 0, mode = 0, phase = 0, gray = 0, grayValid = 0, fault = 0, speedValid = 0;
};
template <size_t N> class Queue {
    static_assert(N > 0, "nonempty queue required");
public:
    bool push(const Sample &s) {
        if (size_ == N) { ++dropped; return false; }
        data_[(head_ + size_) % N] = s; ++size_; return true;
    }
    bool pop(Sample &s) {
        if (!size_) return false;
        s = data_[head_]; head_ = (head_ + 1) % N; --size_; return true;
    }
    size_t size() const { return size_; }
    uint32_t dropped = 0;
private:
    Sample data_[N];
    size_t head_ = 0, size_ = 0;
};
} // namespace telemetry
