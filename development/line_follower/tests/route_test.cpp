#include "Route.h"
#include <cassert>
#include <iostream>
#include <limits>

static uint32_t primeLeg(route::Controller &c, uint32_t t,
                         float &left, float &right) {
    // Clear the previous cross/start mark after 160mm, then arm detection.
    left += 170; right += 170; t += 40;
    assert(c.update(0x18, t, left, right).status == route::Status::Follow);
    left += 30; right += 30; t += 40;
    assert(c.update(0x18, t, left, right).status == route::Status::Follow);
    return t;
}

static route::Result reachMarker(route::Controller &c, uint32_t &t,
                                 float &left, float &right) {
    // Target point is detected inside the configured 780..1140mm window.
    left += 700; right += 700; t += 40;
    assert(c.update(0x18, t, left, right).status == route::Status::Follow);
    t += 40;
    return c.update(0x00, t, left, right);
}

static route::Result quarterTurn(route::Controller &c, uint32_t &t,
                                 float &left, float &right,
                                 const route::Config &cfg) {
    const float arc = cfg.wheelTrackMm * route::kPi / 4.0f;
    left -= arc * 0.8f;
    right += arc * 0.8f;
    route::Result result;
    for (int sample = 0; sample < cfg.acquireSamples; ++sample) {
        t += 40;
        result = c.update(0xe7, t, left, right); // Centered black probes.
    }
    assert(result.status == route::Status::Follow);
    return result;
}

int main() {
    route::Controller c;
    route::Config cfg;
    cfg.wheelTrackMm = 120;
    cfg.turnSign = 1;
    route::Config bad = cfg;
    bad.wheelTrackMm = 0;
    assert(!c.start(route::Task::AB, bad, 0, 0, 0));
    bad = cfg; bad.turnSign = 0;
    assert(c.start(route::Task::AB, bad, 0, 0, 0));
    assert(!c.start(route::Task::Lap, bad, 0, 0, 0));

    // Ignore A's starting cross, approach slowly, then stop at B.
    float left = 0, right = 0;
    uint32_t t = 0;
    assert(c.start(route::Task::AB, cfg, t, left, right));
    t = primeLeg(c, t, left, right);
    left += 610; right += 610; t += 40;
    const auto approach = c.update(0x18, t, left, right);
    assert(approach.status == route::Status::Follow && approach.approachingPoint);
    t += 40;
    assert(c.update(0x00, t, left, right).status == route::Status::Follow);
    t += 40;
    const auto atB = c.update(0x00, t, left, right);
    assert(atB.status == route::Status::Complete && atB.legsCompleted == 1);
    assert(atB.stopPoint == route::StopPoint::B);
    assert(atB.leftPwm == 0 && atB.rightPwm == 0);
    assert(c.update(0xff, t + 1, left, right).status == route::Status::Complete);

    // Reject a premature all-black patch instead of treating it as a point.
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    t = primeLeg(c, 0, left = 0, right = 0);
    left += 300; right += 300; t += 40;
    assert(c.update(0x00, t, left, right).status == route::Status::Fault);

    // A four-leg lap turns at three corners and stops on returning to A.
    assert(c.start(route::Task::Lap, cfg, 0, 0, 0));
    left = right = 0; t = 0;
    for (int leg = 0; leg < 4; ++leg) {
        t = primeLeg(c, t, left, right);
        auto marker = reachMarker(c, t, left, right);
        assert(marker.legsCompleted == leg + 1);
        if (leg < 3) {
            assert(marker.status == route::Status::Turn);
            assert(marker.leftPwm < 0 && marker.rightPwm > 0);
            quarterTurn(c, t, left, right, cfg);
        } else {
            assert(marker.status == route::Status::Complete);
            assert(marker.stopPoint == route::StopPoint::A);
            assert(marker.leftPwm == 0 && marker.rightPwm == 0);
        }
    }

    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0xff, 50, 50, 50).status == route::Status::Fault);
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0x18, 12000, 0, 0).status == route::Status::Fault);
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0x18, 100, std::numeric_limits<float>::quiet_NaN(), 0).status == route::Status::Fault);
    std::cout << "PASS: start-cross guard, approach, B/A stop, right-angle reacquire, loss and timeout\n";
}
