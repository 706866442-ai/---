#include "Route.h"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    route::Controller c;
    route::Config cfg;
    cfg.wheelTrackMm = 120; cfg.turnSign = 1;
    route::Config bad = cfg;
    bad.wheelTrackMm = 0;
    assert(!c.start(route::Task::AB, bad, 0, 0, 0));
    bad = cfg; bad.turnSign = 0;
    assert(c.start(route::Task::AB, bad, 0, 0, 0));
    assert(!c.start(route::Task::Lap, bad, 0, 0, 0));
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    c.update(0xe7, 60, 200, 200);
    assert(c.update(0x18, 100, 400, 400).status == route::Status::Follow);
    assert(c.update(0x00, 200, 900, 900).status == route::Status::Follow);
    const auto atB = c.update(0x00, 240, 920, 920);
    assert(atB.status == route::Status::Complete && atB.legsCompleted == 1);
    assert(atB.leftPwm == 0 && atB.rightPwm == 0);
    assert(c.update(0xff, 300, 950, 950).status == route::Status::Complete);

    assert(c.start(route::Task::Lap, cfg, 0, 0, 0));
    c.update(0xe7, 20, 200, 200); c.update(0xe7, 60, 300, 300);
    c.update(0x00, 100, 900, 900);
    auto turning = c.update(0x00, 140, 900, 900);
    assert(turning.status == route::Status::Turn);
    assert(turning.leftPwm < 0 && turning.rightPwm > 0);
    const float arc = cfg.wheelTrackMm * route::kPi / 4;
    for (int n=0;n<3;++n) {
        turning = c.update(0xe7, 180+n*40, 900-arc*.8f, 900+arc*.8f);
    }
    assert(turning.status == route::Status::Follow);
    assert(c.update(0xff, 350, 900, 900).status == route::Status::Fault);
    assert(c.update(0x18, 400, 900, 900).leftPwm == 0);

    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0x00, 50, 50, 50).status == route::Status::Follow);
    assert(c.update(0x00, 90, 200, 200).status == route::Status::Fault);
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0x18, 9000, 900, 900).status == route::Status::Follow);
    assert(c.update(0x18, 12000, 900, 900).status == route::Status::Fault);
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0x18, 100, std::numeric_limits<float>::quiet_NaN(), 0).status == route::Status::Fault);

    assert(c.start(route::Task::Lap, cfg, 0, 0, 0));
    float left = 0, right = 0;
    uint32_t time = 0;
    for (int leg = 0; leg < 4; ++leg) {
        left += 200; right += 200; time += 40; c.update(0xe7, time, left, right);
        left += 100; right += 100; time += 40; c.update(0xe7, time, left, right);
        left += 600; right += 600; time += 40;
        assert(c.update(0x00, time, left, right).status == route::Status::Follow);
        time += 40;
        const route::Result marker = c.update(0x00, time, left, right);
        assert(marker.legsCompleted == leg + 1);
        if (leg < 3) {
            assert(marker.status == route::Status::Turn);
            left -= arc * .8f; right += arc * .8f;
            for (int sample=0;sample<3;++sample) {
                time += 40;
                turning = c.update(0xe7, time, left, right);
            }
            assert(turning.status == route::Status::Follow);
        } else {
            assert(marker.status == route::Status::Complete);
            assert(marker.leftPwm == 0 && marker.rightPwm == 0);
        }
    }
    // A start mark cannot be mistaken for an endpoint without departure samples.
    assert(c.start(route::Task::AB, cfg, 0, 0, 0));
    assert(c.update(0, 40, 900, 900).status == route::Status::Fault);
    // Joining halfway along an edge and choosing a specific number of legs.
    auto partial = cfg; partial.firstLegMm = 500; partial.stopAfterLegs = 1;
    assert(c.start(route::Task::Lap, partial, 0, 0, 0));
    c.update(0xe7, 40, 150, 150); c.update(0xe7, 80, 200, 200);
    assert(c.update(0xe7, 120, 400, 400).approachingPoint);
    c.update(0, 160, 450, 450);
    assert(c.update(0, 200, 450, 450).status == route::Status::Complete);
    // One wheel cannot indefinitely mask a turn stall.
    assert(c.start(route::Task::Lap, cfg, 0, 0, 0));
    c.update(0xe7, 40, 200, 200); c.update(0xe7, 80, 300, 300);
    c.update(0, 120, 900, 900); c.update(0, 160, 900, 900);
    assert(c.update(0xe7, 960, 890, 900).status == route::Status::Fault);
    std::cout << "PASS: route configuration, B stop, turn/reacquire, loss, marker fault, timeout\n";
}
