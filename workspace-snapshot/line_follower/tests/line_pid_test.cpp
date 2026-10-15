#include "LinePid.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace line;
static bool near(float a, float b, float tolerance = 0.001f) {
    return std::fabs(a - b) < tolerance;
}

int main() {
    // Pure proportional steering: error 20 with kp=1 -> correction 20.
    Controller p({1, 0, 0}, 45, 30);
    assert(near(p.update(20, 0.02f), 20));
    // Large errors saturate at the output limit in both directions.
    assert(near(p.update(200, 0.02f), 45));
    assert(near(p.update(-200, 0.02f), -45));

    // Integral corrects a sustained offset, and reset clears the accumulator.
    Controller pi({1, 2, 0}, 45, 30);
    for (int i = 0; i < 100; ++i) pi.update(5, 0.02f);
    assert(pi.update(5, 0.02f) > 5); // Integral adds to the P term.
    pi.reset();
    Controller fresh({1, 2, 0}, 45, 30);
    // Reset behaves exactly like a fresh controller: no stale integral.
    assert(near(pi.update(5, 0.02f), fresh.update(5, 0.02f)));

    // Derivative reacts to changing error, not to a constant error. A tiny kp
    // keeps the controller valid (kp or ki must be positive) while the output
    // is essentially all D.
    Controller pd({1e-6f, 0, 1}, 45, 30);
    assert(near(pd.update(0, 0.02f), 0)); // No history yet.
    assert(near(pd.update(0, 0.02f), 0)); // Constant -> zero derivative.
    assert(pd.update(10, 0.02f) > 0);     // Rising error -> positive D.
    pd.reset();
    assert(near(pd.update(0, 0.02f), 0));
    assert(pd.update(-10, 0.02f) < 0);    // Falling error -> negative D.

    // Invalid periods reset history and return zero; no stale derivative.
    Controller q({1, 0, 0}, 45, 30);
    q.update(10, 0.02f);
    assert(near(q.update(10, 0.0f), 0));
    assert(near(q.update(10, -1.0f), 0));
    assert(near(q.update(10, 3.0f), 0));
    assert(near(q.update(10, 0.02f), 10)); // History was cleared by reset.

    // A zero-gain controller is invalid and always returns zero.
    Controller zero({0, 0, 0}, 45, 30);
    assert(!zero.valid());
    assert(near(zero.update(10, 0.02f), 0));

    // Sustained saturation must not leave a hidden integral after recentering.
    Controller antiwindup({1, 10, 0}, 45, 30);
    for (int i = 0; i < 500; ++i) assert(near(antiwindup.update(100, 0.04f), 45));
    assert(near(antiwindup.update(0, 0.04f), 0));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Controller invalid({nan, 0, 0});
    assert(!invalid.valid());
    assert(near(q.update(10, nan), 0));
    // Same elapsed time at two valid sampling rates accumulates the same I.
    Controller slow({0, 1, 0}, 45, 30), fast({0, 1, 0}, 45, 30);
    float a = 0, b = 0;
    for (int i = 0; i < 10; ++i) a = slow.update(10, 0.04f);
    for (int i = 0; i < 20; ++i) b = fast.update(10, 0.02f);
    assert(near(a, b));

    std::cout << "PASS: proportional, saturation, integral, derivative, "
                 "reset, invalid inputs and validity\n";
}
