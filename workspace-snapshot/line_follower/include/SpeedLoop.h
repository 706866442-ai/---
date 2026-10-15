#pragma once
#include "WheelSpeedControl.h"
#include <cmath>
#include <cstdint>

namespace drive {
// Forward-only adapter shared by Follow and Route straight segments.
// Call tick independently of gray sampling; counts are signed-forward modulo 2^32.
class SpeedLoop {
public:
    static constexpr uint32_t periodMs = 20;
    static constexpr uint32_t maxPeriodMs = 150;
    static constexpr uint32_t stallMs = 1000;
    SpeedLoop(WheelSpeedControl &controller, float feedforward, float limit)
        : controller_(controller), feedforward_(feedforward), limit_(limit) {}

    bool valid() const {
        return controller_.valid() && std::isfinite(feedforward_) &&
            feedforward_ >= 0 && std::isfinite(limit_) && limit_ > 0 &&
            limit_ <= 255 && feedforward_ <= limit_;
    }
    void stop() {
        active_ = false;
        leftTarget_ = rightTarget_ = leftPwm_ = rightPwm_ = leftSpeed_ = rightSpeed_ = 0;
        controller_.reset();
    }
    void start(uint32_t now, uint32_t left, uint32_t right, uint32_t errors) {
        stop(); active_ = true;
        lastMs_ = leftMotionMs_ = rightMotionMs_ = now;
        previousLeft_ = left; previousRight_ = right; initialErrors_ = errors;
    }
    bool targets(float left, float right, uint32_t now) {
        if (!active_ || !std::isfinite(left) || !std::isfinite(right) || left < 0 || right < 0)
            return fail();
        if (left == 0 || leftTarget_ == 0) leftMotionMs_ = now;
        if (right == 0 || rightTarget_ == 0) rightMotionMs_ = now;
        leftTarget_ = left; rightTarget_ = right;
        // Cut output immediately, even between speed samples.
        if (left == 0) { leftPwm_ = 0; controller_.resetLeft(); }
        if (right == 0) { rightPwm_ = 0; controller_.resetRight(); }
        return true;
    }
    bool tick(uint32_t now, uint32_t left, uint32_t right, uint32_t errors) {
        if (!active_) return true;
        const uint32_t elapsed = now - lastMs_;
        if (!valid() || elapsed > maxPeriodMs || errors - initialErrors_ > 8) return fail();
        if (elapsed < periodMs) return true;
        // Convert modulo differences without signed overflow or implementation-defined casts.
        const int64_t dl = delta(left, previousLeft_), dr = delta(right, previousRight_);
        if (dl > INT32_MAX || dl < INT32_MIN || dr > INT32_MAX || dr < INT32_MIN) return fail();
        if (leftTarget_ == 0 || dl > 0) leftMotionMs_ = now;
        if (rightTarget_ == 0 || dr > 0) rightMotionMs_ = now;
        if ((leftTarget_ > 0 && now - leftMotionMs_ >= stallMs) ||
            (rightTarget_ > 0 && now - rightMotionMs_ >= stallMs)) return fail();
        const auto sample = controller_.update(leftTarget_, rightTarget_,
            static_cast<int>(dl), static_cast<int>(dr), elapsed * 0.001f, true);
        if (!sample.valid) return fail();
        leftSpeed_ = sample.leftSpeed; rightSpeed_ = sample.rightSpeed;
        leftPwm_ = pwm(leftTarget_, sample.leftPwm);
        rightPwm_ = pwm(rightTarget_, sample.rightPwm);
        lastMs_ = now; previousLeft_ = left; previousRight_ = right;
        return true;
    }
    bool due(uint32_t now) const { return active_ && now - lastMs_ >= periodMs; }
    bool active() const { return active_; }
    float leftTarget() const { return leftTarget_; }
    float rightTarget() const { return rightTarget_; }
    float leftSpeed() const { return leftSpeed_; }
    float rightSpeed() const { return rightSpeed_; }
    float leftPwm() const { return leftPwm_; }
    float rightPwm() const { return rightPwm_; }
private:
    static int64_t delta(uint32_t current, uint32_t previous) {
        const uint32_t d = current - previous;
        return d <= INT32_MAX ? static_cast<int64_t>(d) : static_cast<int64_t>(d) - 0x100000000LL;
    }
    float pwm(float target, float feedback) const {
        if (target == 0) return 0;
        const float output = feedforward_ + feedback;
        return output < 0 ? 0 : (output > limit_ ? limit_ : output);
    }
    bool fail() { stop(); return false; }
    WheelSpeedControl &controller_;
    float feedforward_, limit_;
    bool active_ = false;
    float leftTarget_ = 0, rightTarget_ = 0, leftPwm_ = 0, rightPwm_ = 0;
    float leftSpeed_ = 0, rightSpeed_ = 0;
    uint32_t lastMs_ = 0, leftMotionMs_ = 0, rightMotionMs_ = 0;
    uint32_t previousLeft_ = 0, previousRight_ = 0, initialErrors_ = 0;
};
} // namespace drive
