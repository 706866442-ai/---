#include "WheelSpeedControl.h"
#include <cmath>

namespace drive {
namespace {
float clamp(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
}
bool finite(float value) { return std::isfinite(value); }
bool validPeriod(float seconds) {
    // This car samples wheel speed every 100 ms. Reject stalls rather than integrating
    // several seconds of error after a blocking I2C call or LED delay.
    return finite(seconds) && seconds >= 0.001f && seconds <= 0.15f;
}
} // namespace

SpeedPid::SpeedPid(PidGains gains, float pwmLimit, float integralLimit,
                   float derivativeFilterSeconds)
    : gains_(gains), pwmLimit_(pwmLimit), integralLimit_(integralLimit),
      derivativeFilterSeconds_(derivativeFilterSeconds) {}

bool SpeedPid::valid() const {
    return finite(gains_.kp) && finite(gains_.ki) && finite(gains_.kd)
        && gains_.kp >= 0 && gains_.ki >= 0 && gains_.kd >= 0
        && (gains_.kp > 0 || gains_.ki > 0)
        && finite(pwmLimit_) && pwmLimit_ > 0 && pwmLimit_ <= 255
        && finite(integralLimit_) && integralLimit_ >= 0
        && integralLimit_ <= pwmLimit_
        && finite(derivativeFilterSeconds_) && derivativeFilterSeconds_ >= 0;
}

void SpeedPid::reset() {
    integral_ = 0;
    previousMeasured_ = 0;
    derivative_ = 0;
    hasPrevious_ = false;
}

float SpeedPid::update(float target, float measured, float dtSeconds) {
    if (!valid() || !validPeriod(dtSeconds) || !finite(target)
        || !finite(measured) || target == 0.0f) {
        reset();
        return 0;
    }
    const float error = target - measured;
    const float rawDerivative = hasPrevious_
        ? -(measured - previousMeasured_) / dtSeconds : 0.0f;
    const float alpha = dtSeconds / (derivativeFilterSeconds_ + dtSeconds);
    derivative_ += alpha * (rawDerivative - derivative_);
    previousMeasured_ = measured;
    hasPrevious_ = true;

    // Derivative on measurement avoids a derivative kick on target changes.
    const float pd = gains_.kp * error + gains_.kd * derivative_;
    const float candidateI = clamp(integral_ + gains_.ki * error * dtSeconds,
                                   integralLimit_);
    const float candidateOutput = pd + candidateI;
    // Freeze integration when it would drive the saturated output further
    // into saturation. Permit integration that helps it recover.
    if ((candidateOutput <= pwmLimit_ && candidateOutput >= -pwmLimit_)
        || (candidateOutput > pwmLimit_ && error < 0)
        || (candidateOutput < -pwmLimit_ && error > 0)) {
        integral_ = candidateI;
    }
    const float output = pd + integral_;
    if (!finite(error) || !finite(derivative_) || !finite(output)) {
        reset();
        return 0;
    }
    return clamp(output, pwmLimit_);
}

WheelSpeedControl::WheelSpeedControl(WheelConfig left, WheelConfig right,
                                   float pwmLimit, float integralLimit)
    : leftConfig_(left), rightConfig_(right),
      left_(left.gains, pwmLimit, integralLimit),
      right_(right.gains, pwmLimit, integralLimit) {}

void WheelSpeedControl::reset() { left_.reset(); right_.reset(); }

DriveSample WheelSpeedControl::update(float leftTarget, float rightTarget,
                                     int leftCountDelta, int rightCountDelta,
                                     float dtSeconds, bool enabled) {
    DriveSample result;
    if (!enabled || !validPeriod(dtSeconds) || !left_.valid() || !right_.valid()
        || !finite(leftTarget) || !finite(rightTarget)
        || !finite(leftConfig_.mmPerCount) || leftConfig_.mmPerCount <= 0
        || !finite(rightConfig_.mmPerCount) || rightConfig_.mmPerCount <= 0) {
        reset();
        return result;
    }
    result.leftSpeed = leftCountDelta * leftConfig_.mmPerCount / dtSeconds;
    result.rightSpeed = rightCountDelta * rightConfig_.mmPerCount / dtSeconds;
    if (!finite(result.leftSpeed) || !finite(result.rightSpeed)) {
        reset();
        return DriveSample{};
    }
    result.leftPwm = left_.update(leftTarget, result.leftSpeed, dtSeconds);
    result.rightPwm = right_.update(rightTarget, result.rightSpeed, dtSeconds);
    result.valid = true;
    return result;
}

DriveSample WheelSpeedControl::straight(float targetSpeed, int leftCountDelta,
                                       int rightCountDelta, float dtSeconds,
                                       bool enabled) {
    return update(targetSpeed, targetSpeed, leftCountDelta, rightCountDelta,
                  dtSeconds, enabled);
}
} // namespace drive
