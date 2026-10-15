#include "WheelSpeedControl.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace drive;
static bool near(float a, float b, float tolerance = 0.001f) {
    return std::fabs(a - b) < tolerance;
}

int main() {
    SpeedPid p({1, 0, 0}, 80, 40);
    assert(near(p.update(50, 30, 0.02f), 20)); // Slow -> more drive.
    assert(near(p.update(50, 60, 0.02f), -10)); // Fast -> braking command.
    assert(near(p.update(-50, -30, 0.02f), -20));
    assert(near(p.update(5000, 0, 0.02f), 80));

    SpeedPid pi({1, 2, 0}, 80, 40);
    for (int i = 0; i < 1000; ++i) assert(near(pi.update(1000, 0, .02f), 80));
    assert(near(pi.update(20, 20, .02f), 0)); // No saturated integral left.
    for (int i = 0; i < 100; ++i) pi.update(20, 19, .02f);
    assert(pi.update(20, 20, .02f) > 0); // Integral corrects steady error.
    assert(near(pi.update(0, 20, .02f), 0));
    assert(near(pi.update(20, 20, .02f), 0)); // Stop resets history.
    for (float dt : {0.0f, -1.0f, 3.0f}) assert(near(pi.update(20, 0, dt), 0));
    assert(near(pi.update(std::numeric_limits<float>::quiet_NaN(), 0, .02f), 0));

    SpeedPid pd({1, 0, 1}, 80, 40);
    pd.update(10, 5, .02f);
    assert(near(pd.update(20, 5, .02f), 15)); // No setpoint derivative kick.

    WheelSpeedControl wheels({{1, 0, 0}, .5f}, {{1, 0, 0}, .25f});
    auto s = wheels.straight(60, 2, 4, .02f, true);
    assert(s.valid && near(s.leftSpeed, 50) && near(s.rightSpeed, 50));
    assert(near(s.leftPwm, 10) && near(s.rightPwm, 10));
    s = wheels.straight(60, 1, 4, .02f, true);
    assert(s.leftPwm > s.rightPwm); // Independently correct slow left wheel.
    s = wheels.straight(60, 1, 4, .02f, false);
    assert(!s.valid && s.leftPwm == 0 && s.rightPwm == 0);
    s = wheels.straight(60, 1, 4, 3.0f, true);
    assert(!s.valid && s.leftPwm == 0 && s.rightPwm == 0);
    WheelSpeedControl unknown({{1, 0, 0}, 0}, {{1, 0, 0}, .25f});
    assert(!unknown.straight(60, 0, 0, .02f, true).valid);

    // Simulated motors with different gain and time constant. These numbers
    // exercise convergence; they are NOT gains calibrated for the real car.
    SpeedPid left({.6f, 1.2f, 0}, 80, 60);
    SpeedPid right({.6f, 1.2f, 0}, 80, 60);
    float lv = 0, rv = 0;
    for (int i = 0; i < 1500; ++i) {
        const float lu = left.update(100, lv, .02f);
        const float ru = right.update(100, rv, .02f);
        lv += .02f / .2f * (2.4f * lu - lv);
        rv += .02f / .3f * (1.8f * ru - rv);
        assert(std::fabs(lu) <= 80 && std::fabs(ru) <= 80);
    }
    assert(near(lv, 100, .1f) && near(rv, 100, .1f));
    std::cout << "PASS: feedback direction, saturation, anti-windup, stop/reset, "
                 "invalid inputs, derivative, wheel scaling and simulated convergence\n";
}
