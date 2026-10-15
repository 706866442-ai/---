#pragma once

namespace drive {

struct PidGains {
    float kp;
    float ki;
    float kd;
};

// Input: mm/s; output: signed PWM command in [-pwmLimit, pwmLimit].
class SpeedPid {
public:
    SpeedPid(PidGains gains, float pwmLimit, float integralLimit,
             float derivativeFilterSeconds = 0.04f);
    float update(float target, float measured, float dtSeconds);
    void reset();
    bool valid() const;

private:
    PidGains gains_;
    float pwmLimit_;
    float integralLimit_;
    float derivativeFilterSeconds_;
    float integral_ = 0.0f;
    float previousMeasured_ = 0.0f;
    float derivative_ = 0.0f;
    bool hasPrevious_ = false;
};

struct WheelConfig {
    PidGains gains;
    // Wheel circumference / decoded counts per WHEEL revolution.
    // Include gear ratio and the actual x1/x2/x4 decoding factor.
    float mmPerCount;
};

struct DriveSample {
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    float leftPwm = 0.0f;
    float rightPwm = 0.0f;
    bool valid = false;
};

// Hardware-independent controller. The caller supplies signed count increments
// from the SAME sampling interval. Positive counts must mean vehicle forward.
// This does not detect encoder disconnection: add a feedback timeout/stall
// interlock in the hardware adapter before enabling motor power.
class WheelSpeedControl {
public:
    WheelSpeedControl(WheelConfig left, WheelConfig right,
                      float pwmLimit = 80.0f, float integralLimit = 40.0f);
    DriveSample update(float leftTarget, float rightTarget,
                       int leftCountDelta, int rightCountDelta,
                       float dtSeconds, bool enabled);
    DriveSample straight(float targetSpeed, int leftCountDelta,
                         int rightCountDelta, float dtSeconds, bool enabled);
    void reset();
    bool valid() const;

private:
    WheelConfig leftConfig_;
    WheelConfig rightConfig_;
    SpeedPid left_;
    SpeedPid right_;
};

} // namespace drive
