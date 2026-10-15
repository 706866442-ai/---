#pragma once
#include <cmath>
#include <cstdint>

// Gray line-following steering loop. Input is the signed line-position error
// produced by lineErrorFromDigital() (approximately -3500..3500, 0 = centred,
// sign follows the CHANNEL_1_IS_LEFT convention). Output is a signed PWM
// correction added to one wheel and subtracted from the other.
//
// This replaces the original fixed proportional term (error * K / 3500) with a
// full PID whose gains live in RobotConfig. The derivative is on error (whose
// setpoint is always zero, so error derivative == measured derivative) and is
// low-pass filtered to limit noise from the 40 ms sensor sample rate. The
// integral is bounded and frozen while saturated. It stays disabled by default
// (LINE_PID_ENABLED=false), so the original proportional behaviour is unchanged
// until the sensor order and gains are verified on the real track.
namespace line {

struct PidGains {
    float kp; // steering PWM per unit line error
    float ki; // steering PWM per unit error-second
    float kd; // steering PWM per unit error/second
};

class Controller {
public:
    Controller(PidGains gains, float outputLimit = 45.0f,
               float integralLimit = 30.0f,
               float derivativeFilterSeconds = 0.04f)
        : gains_(gains), outputLimit_(outputLimit), integralLimit_(integralLimit),
          derivativeFilterSeconds_(derivativeFilterSeconds) {}

    void reset() {
        integral_ = 0.0f;
        previousError_ = 0.0f;
        derivative_ = 0.0f;
        hasPrevious_ = false;
    }

    bool valid() const {
        return std::isfinite(gains_.kp) && std::isfinite(gains_.ki) &&
               std::isfinite(gains_.kd) && gains_.kp >= 0 && gains_.ki >= 0 &&
               gains_.kd >= 0 && (gains_.kp > 0 || gains_.ki > 0) &&
               std::isfinite(outputLimit_) && outputLimit_ > 0 &&
               outputLimit_ <= 255 &&
               std::isfinite(integralLimit_) && integralLimit_ >= 0 &&
               integralLimit_ <= outputLimit_ &&
               std::isfinite(derivativeFilterSeconds_) &&
               derivativeFilterSeconds_ >= 0;
    }

    // dtSeconds should be the actual control period (about 40 ms). A stall or
    // blocking I2C read that stretches the period is rejected rather than
    // integrated, mirroring WheelSpeedControl::validPeriod.
    float update(int16_t error, float dtSeconds) {
        if (!valid() || !std::isfinite(dtSeconds) || dtSeconds < 0.001f ||
            dtSeconds > 0.15f) {
            reset();
            return 0.0f;
        }
        const float e = static_cast<float>(error);
        const float rawDerivative =
            hasPrevious_ ? (e - previousError_) / dtSeconds : 0.0f;
        const float alpha = dtSeconds / (derivativeFilterSeconds_ + dtSeconds);
        derivative_ += alpha * (rawDerivative - derivative_);
        previousError_ = e;
        hasPrevious_ = true;

        const float pd = gains_.kp * e + gains_.kd * derivative_;
        const float candidateI =
            clamp(integral_ + gains_.ki * e * dtSeconds, integralLimit_);
        const float candidateOutput = pd + candidateI;
        // Freeze integration when it would push a saturated output further
        // into saturation; allow integration that recovers toward the centre.
        if ((candidateOutput <= outputLimit_ && candidateOutput >= -outputLimit_)
            || (candidateOutput > outputLimit_ && e < 0)
            || (candidateOutput < -outputLimit_ && e > 0)) {
            integral_ = candidateI;
        }
        const float output = pd + integral_;
        if (!std::isfinite(output)) {
            reset();
            return 0.0f;
        }
        return clamp(output, outputLimit_);
    }

private:
    static float clamp(float value, float limit) {
        return value > limit ? limit : (value < -limit ? -limit : value);
    }
    PidGains gains_;
    float outputLimit_;
    float integralLimit_;
    float derivativeFilterSeconds_;
    float integral_ = 0.0f;
    float previousError_ = 0.0f;
    float derivative_ = 0.0f;
    bool hasPrevious_ = false;
};

} // namespace line
