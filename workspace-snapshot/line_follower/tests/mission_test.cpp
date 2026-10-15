#include "Mission.h"
#include <cassert>
#include <iostream>
#include <limits>

mission::Config config() {
    mission::Config c; c.route.wheelTrackMm = 120; c.route.turnSign = 1; return c;
}
int main() {
    mission::Controller c; mission::Plan p; auto cfg = config(); mission::Input in;
    assert(!c.start(p, cfg, in));
    p.add(mission::Action::FollowDistance, 100);
    parking::Plan park; assert(parking::reverseBay(park, 120, 120, 1));
    assert(mission::appendParking(p, park));
    assert(c.start(p, cfg, in));
    mission::Output last;
    // Simulated wheel travel: tests the sequence and feedback, not physical vehicle accuracy.
    for (uint32_t now = 20; now < 20000 && c.status() == mission::Status::Running; now += 20) {
        if (last.command == mission::Command::Follow) { in.leftMm += 10; in.rightMm += 10; }
        if (last.command == mission::Command::Pwm) {
            in.leftMm += last.leftPwm * 0.02f; in.rightMm += last.rightPwm * 0.02f;
        }
        in.now = now; in.digital = 0xe7; in.grayFresh = now % 40 == 0;
        const auto out = c.update(in);
        if (out.command != mission::Command::Hold) last = out;
    }
    assert(c.status() == mission::Status::Complete && last.command == mission::Command::Stop);
    c.stop(); assert(c.status() == mission::Status::Idle);
    in = mission::Input{}; assert(c.start(p, cfg, in));
    in.now = 151; assert(c.update(in).command == mission::Command::Stop);
    assert(c.status() == mission::Status::Fault && c.error() == mission::Error::Sensor);
    // Full target selection mapping when joining BC and first reaching C.
    for (uint8_t value = 1; value <= 4; ++value) {
        p = mission::Plan{}; p.add(mission::Action::ReadRoman, 0, 0, 0, 1000);
        p.add(mission::Action::Route, 500, 0, 0, 30000);
        in = mission::Input{}; assert(c.start(p, cfg, in));
        for (uint8_t seq = 0; seq < 3; ++seq) {
            vision::Frame f; f.kind = vision::Kind::Roman; f.value = value;
            f.sequence = seq; f.confidence = 90; f.receivedMs = in.now = 20 + seq * 20;
            c.receive(f, in.now);
        }
        assert(c.target() == value - 1);
        assert(c.update(in).command == mission::Command::Stop);
        assert(c.needsGray());
        float l = 0, r = 0;
        const int legs = ((value - 1 - 2 + 4) % 4) + 1;
        for (int leg = 0; leg < legs; ++leg) {
            const float distance = leg == 0 ? 500 : 1000;
            for (float delta : {160.0f, 40.0f, distance - 250}) {
                l += delta; r += delta; in.leftMm = l; in.rightMm = r;
                in.now += 40; in.grayFresh = true; in.digital = 0xe7; c.update(in);
            }
            in.digital = 0;
            in.now += 40; c.update(in); in.now += 40; auto out = c.update(in);
            if (leg + 1 < legs) {
                assert(out.command == mission::Command::Pwm);
                l -= 80; r += 80; in.leftMm = l; in.rightMm = r; in.digital = 0xe7;
                for (int n = 0; n < 3; ++n) { in.now += 40; c.update(in); }
            } else assert(c.status() == mission::Status::Complete);
        }
    }
    p = mission::Plan{}; p.add(mission::Action::ReadRoman, 0, 0, 0, 1000);
    in = mission::Input{}; assert(c.start(p, cfg, in));
    in.now = 1000; c.update(in);
    assert(c.status() == mission::Status::Fault && c.error() == mission::Error::Vision);
    p = mission::Plan{}; p.add(mission::Action::Move, 100, 100);
    in = mission::Input{}; assert(c.start(p, cfg, in));
    for (int n = 1; n <= 40; ++n) { in.now = n * 20; in.leftMm += 1; c.update(in); }
    assert(c.status() == mission::Status::Fault); // one wheel blocked
    std::cout << "PASS: approach/turn/reverse/settle, target A-D, stale gray, vision timeout, single-wheel failure\n";
}
