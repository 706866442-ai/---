#include "VisionProtocol.h"
#include <cassert>
#include <iostream>

bool send(vision::Parser &p, uint8_t seq, uint8_t value, uint32_t now,
          vision::Frame &out, bool corrupt = false) {
    uint8_t b[] = {0xa5, 1, 1, seq, value, 90, 0};
    b[6] = vision::crc8(b + 1, 5) ^ (corrupt ? 1 : 0);
    bool result = false;
    for (uint8_t byte : b) result = p.feed(byte, now, out);
    return result;
}
int main() {
    const uint8_t standard[] = {'1','2','3','4','5','6','7','8','9'};
    assert(vision::crc8(standard, 9) == 0xf4);
    vision::Parser p; vision::Frame f; vision::StableResult stable;
    assert(!p.feed(0x13, 0, f));
    assert(!send(p, 254, 1, 0, f, true));
    assert(send(p, 254, 1, 1, f)); assert(stable.accept(f, 1, vision::Kind::Roman) == 0);
    assert(!send(p, 254, 1, 2, f)); // replay cannot build confidence
    assert(send(p, 255, 1, 100, f)); assert(stable.accept(f, 100, vision::Kind::Roman) == 0);
    assert(send(p, 0, 1, 200, f)); assert(stable.accept(f, 200, vision::Kind::Roman) == 1);
    assert(!send(p, 255, 1, 201, f)); // old sequence
    assert(send(p, 1, 0, 202, f)); assert(stable.accept(f, 202, vision::Kind::Roman) == 0);
    assert(send(p, 2, 2, 210, f)); assert(stable.accept(f, 600, vision::Kind::Roman) == 0);
    // Interrupted packet must not be combined with a fresh suffix.
    p.reset(); p.feed(0xa5, 0, f); p.feed(1, 1, f); p.feed(1, 200, f);
    assert(send(p, 1, 4, 220, f));
    f.confidence = 20; assert(stable.accept(f, 220, vision::Kind::Roman) == 0);
    for (uint8_t v = 1; v <= 4; ++v) assert(vision::romanPoint(v) == v - 1);
    assert(vision::romanPoint(0) == -1);
    std::cout << "PASS: CRC vector, corrupt/truncated/replayed frames, sequence wrap, stale/unknown/confidence\n";
}
