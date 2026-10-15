#include "SpeedLoop.h"
#include "LinePid.h"
#include "Route.h"
#include <cassert>
#include <iostream>
#include <limits>

drive::WheelSpeedControl controller() {
    return drive::WheelSpeedControl({{1, 0.1f, 0}, 1}, {{1, 0.1f, 0}, 1});
}

int main() {
    auto pid = controller();
    drive::SpeedLoop loop(pid, 60, 120);
    assert(loop.valid());
    loop.start(0, 0, 0, 0);
    assert(loop.targets(100, 100, 0));
    assert(!loop.due(19) && loop.due(20));
    assert(loop.tick(20, 2, 2, 0)); // Inner update between outer samples at 0/40 ms.
    assert(loop.leftPwm() > 0 && loop.rightPwm() > 0);
    assert(loop.targets(0, 100, 21));
    assert(loop.leftPwm() == 0); // Immediate, not delayed until next speed tick.
    assert(loop.tick(40, 2, 4, 0));
    assert(loop.leftPwm() == 0 && loop.rightPwm() > 0); // No zero-target feedforward.
    for (uint32_t t = 60; t <= 1500; t += 20) assert(loop.tick(t, 2, t / 10, 0));
    // A deliberately stopped wheel is exempt; restarting receives a fresh timeout.
    assert(loop.targets(100, 100, 1500));
    for (uint32_t t = 1520; t < 2500; t += 20) assert(loop.tick(t, 2, t / 10, 0));
    assert(!loop.tick(2500, 2, 250, 0));
    assert(!loop.active() && loop.leftPwm() == 0 && loop.rightPwm() == 0);
    // Right failure is not masked by left-wheel motion either.
    loop.start(0, 0, 0, 0); loop.targets(100, 100, 0);
    for (uint32_t t = 20; t < 1000; t += 20) assert(loop.tick(t, t / 10, 0, 0));
    assert(!loop.tick(1000, 100, 0, 0));
    // Wraparound of both the timebase and encoder count is a normal positive increment.
    loop.start(UINT32_MAX - 9, UINT32_MAX - 1, UINT32_MAX - 1, UINT32_MAX - 2);
    loop.targets(100, 100, UINT32_MAX - 9);
    assert(loop.tick(10, 0, 0, 1));
    assert(loop.leftPwm() > 0);
    assert(!loop.tick(161, 1, 1, 1)); // Overlong actual interval fails closed.
    loop.start(0, 0, 0, 0); loop.targets(100, 100, 0);
    assert(!loop.tick(20, 2, 2, 9)); // Invalid transitions.
    loop.start(0, 0, 0, 0);
    assert(!loop.targets(std::numeric_limits<float>::quiet_NaN(), 1, 0));
    auto invalidPid = drive::WheelSpeedControl({{0, 0, 0}, 1}, {{0, 0, 0}, 1});
    drive::SpeedLoop invalid(invalidPid, 60, 120);
    assert(!invalid.valid());

    // Exercise the actual Route -> turn -> straight sequence with both controllers.
    route::Controller route;
    route::Config cfg; cfg.wheelTrackMm = 120; cfg.turnSign = 1;
    assert(route.start(route::Task::Lap, cfg, 0, 0, 0));
    line::Controller outer({0.01f, 0, 0}, 45, 30);
    loop.start(0, 0, 0, 0);
    const float correction = outer.update(1000, 0.04f) / 45 * 50;
    loop.targets(100 + correction, 100 - correction, 0);
    assert(loop.tick(20, 2, 2, 0));
    assert(loop.leftPwm() > loop.rightPwm());
    route.update(0xe7, 21, 200, 200); route.update(0xe7, 22, 300, 300);
    assert(route.update(0, 40, 1000, 1000).status == route::Status::Follow);
    assert(route.update(0, 80, 1000, 1000).status == route::Status::Turn);
    loop.stop(); outer.reset();
    assert(!loop.active() && loop.leftPwm() == 0);
    assert(loop.tick(100, 1000, 1000, 0)); // Disabled speed loop cannot overwrite turn PWM.
    for (uint32_t t : {120u, 160u, 200u}) route.update(0xe7, t, 920, 1080);
    assert(route.status() == route::Status::Follow);
    loop.start(200, 920, 1080, 0); loop.targets(100, 100, 200);
    assert(loop.tick(220, 922, 1082, 0)); // No turn counts or derivative history carried over.
    assert(loop.leftPwm() == loop.rightPwm());
    loop.stop();
    assert(loop.tick(240, 924, 1084, 0));
    assert(loop.leftPwm() == 0 && loop.rightPwm() == 0);
    std::cout << "PASS: zero targets, independent stalls, scheduling, wrap, invalid inputs, route cascade/reset\n";
}
