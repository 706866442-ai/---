#include "Telemetry.h"
#include <cassert>
#include <iostream>
int main() {
    telemetry::Queue<2> q;
    telemetry::Sample a, b, out; a.ms = 1; b.ms = 2;
    assert(!q.pop(out)); assert(q.push(a)); assert(q.push(b));
    assert(!q.push(a) && q.dropped == 1);
    assert(q.pop(out) && out.ms == 1);
    a.ms = 3; assert(q.push(a));
    assert(q.pop(out) && out.ms == 2);
    assert(q.pop(out) && out.ms == 3);
    assert(!q.pop(out));
    std::cout << "PASS: bounded telemetry queue, FIFO, overflow accounting, ring wrap\n";
}
