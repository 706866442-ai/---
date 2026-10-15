#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace parking {
constexpr float kPi = 3.14159265359f;
struct Segment {
    float leftMm, rightMm;
    Segment(float l=0, float r=0) : leftMm(l), rightMm(r) {}
};
struct Plan { Segment steps[3]; size_t count = 0; };
inline bool positive(float x) { return std::isfinite(x) && x > 0; }
inline Segment motion(float distance, float yawRad, float track) {
    // Positive yaw turns left; positive distance moves forward.
    return {distance - yawRad * track / 2, distance + yawRad * track / 2};
}
inline bool reverseBay(Plan &out, float track, float depth, int side) {
    out = Plan{};
    if (!positive(track) || !positive(depth) || depth > 1000 ||
        (side != 1 && side != -1)) return false;
    // Right bay: turn LEFT away from the bay, then reverse into it, nose out.
    out.steps[0] = motion(0, side * kPi / 2, track);
    out.steps[1] = motion(-depth, 0, track);
    out.count = 2;
    return true;
}
inline bool parallelBay(Plan &out, float track, float radius, float angleDeg,
                        float trim, int side) {
    out = Plan{};
    if (!positive(track) || !positive(radius) || radius <= track / 2 ||
        !positive(angleDeg) || angleDeg > 90 || !std::isfinite(trim) ||
        std::fabs(trim) > 500 || (side != 1 && side != -1)) return false;
    const float theta = angleDeg * kPi / 180;
    out.steps[0] = motion(-radius * theta, side * theta, track);
    out.steps[1] = motion(-radius * theta, -side * theta, track);
    out.count = 2;
    if (std::fabs(trim) > 0.01f) out.steps[out.count++] = motion(trim, 0, track);
    return true;
}

enum class Status { Idle, Running, Complete, Invalid, Timeout, Stalled, WrongDirection };
struct Output {
    int left, right;
    Output(int l=0, int r=0) : left(l), right(r) {}
};

// Position feedback with ratio synchronisation, not a tuned wheel-speed PID.
// Caller supplies signed cumulative wheel distances and services update every 20ms.
class Controller {
public:
    Status status = Status::Idle;
    size_t step = 0;
    bool start(const Plan &p, uint32_t now, float left, float right) {
        stop();
        if (p.count == 0 || p.count > 3 || !std::isfinite(left) || !std::isfinite(right)) {
            status = Status::Invalid; return false;
        }
        for (size_t i = 0; i < p.count; ++i) {
            if (!std::isfinite(p.steps[i].leftMm) || !std::isfinite(p.steps[i].rightMm) ||
                std::fabs(p.steps[i].leftMm) > 1500 || std::fabs(p.steps[i].rightMm) > 1500) {
                status = Status::Invalid; return false;
            }
        }
        plan = p; step = 0; started = now; status = Status::Running;
        beginStep(now, left, right); return true;
    }
    void stop() { status = Status::Idle; }
    Output update(uint32_t now, float left, float right) {
        if (status != Status::Running) return {};
        if (!std::isfinite(left) || !std::isfinite(right)) return fail(Status::Invalid);
        if (now - started >= 30000 || now - stepStarted >= 8000) return fail(Status::Timeout);
        if (settling) {
            if (now - settledAt >= 300) {
                if (++step == plan.count) status = Status::Complete;
                else beginStep(now, left, right);
            }
            return {};
        }
        const Segment target = plan.steps[step];
        const float l = left - originL, r = right - originR;
        if ((target.leftMm > 0 && l < -5) || (target.leftMm < 0 && l > 5) ||
            (target.rightMm > 0 && r < -5) || (target.rightMm < 0 && r > 5))
            return fail(Status::WrongDirection);
        const bool doneL = reached(target.leftMm, l), doneR = reached(target.rightMm, r);
        if (doneL && doneR) { settling = true; settledAt = now; return {}; }
        if (std::fabs(left - progressL) >= 0.5f) { progressL = left; movedL = now; }
        if (std::fabs(right - progressR) >= 0.5f) { progressR = right; movedR = now; }
        if ((!doneL && now - movedL >= 800) || (!doneR && now - movedR >= 800))
            return fail(Status::Stalled);
        const float longest = std::fmax(std::fabs(target.leftMm), std::fabs(target.rightMm));
        float sync = 0;
        if (std::fabs(target.leftMm) > 3 && std::fabs(target.rightMm) > 3)
            sync = (l / target.leftMm - r / target.rightMm) * 50;
        return {command(target.leftMm, l, longest, sync, doneL),
                command(target.rightMm, r, longest, -sync, doneR)};
    }
private:
    Plan plan{};
    uint32_t started=0, stepStarted=0, movedL=0, movedR=0, settledAt=0;
    float originL=0, originR=0, progressL=0, progressR=0;
    bool settling=false;
    static bool reached(float target, float actual) {
        return std::fabs(target) <= 3 || (target >= 0 ? actual >= target - 3 : actual <= target + 3);
    }
    static int command(float target, float actual, float longest, float correction, bool done) {
        if (done) return 0;
        float pwm = std::fmin(75 * std::fabs(target) / longest,
                             38 + 0.6f * std::fabs(target - actual)) - correction;
        pwm = std::fmax(38, std::fmin(90, pwm));
        return (target >= 0 ? 1 : -1) * static_cast<int>(pwm);
    }
    Output fail(Status reason) { status = reason; return {}; }
    void beginStep(uint32_t now, float left, float right) {
        originL = progressL = left; originR = progressR = right;
        stepStarted = movedL = movedR = now; settling = false;
    }
};
} // namespace parking
