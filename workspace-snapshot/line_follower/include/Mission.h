#pragma once
#include "Parking.h"
#include "Route.h"
#include "VisionProtocol.h"

namespace mission {
enum class Action : uint8_t { FollowDistance, Move, Route, ReadRoman };
struct Step {
    Action action = Action::Move;
    float first = 0, second = 0; // follow mm; move left/right mm; route firstLeg mm
    uint8_t legs = 0;
    uint32_t timeoutMs = 12000;
};
struct Plan {
    Step steps[12];
    uint8_t count = 0;
    bool add(Action action, float first = 0, float second = 0, uint8_t legs = 0,
             uint32_t timeoutMs = 12000) {
        if (count == 12) return false;
        Step &s = steps[count++]; s.action = action; s.first = first;
        s.second = second; s.legs = legs; s.timeoutMs = timeoutMs; return true;
    }
};
struct Config {
    route::Config route;
    uint8_t nextPoint = 2; // A=0..D=3, after joining from garage; must be verified.
    int pointStep = 1; // progression A B C D (+1) or A D C B (-1)
    uint32_t totalTimeoutMs = 90000;
};
enum class Status : uint8_t { Idle, Running, Complete, Fault };
enum class Command : uint8_t { Stop, Hold, Follow, Pwm };
enum class Error : uint8_t { None, Configuration, Timeout, Sensor, Feedback, Motion, Vision };
struct Input {
    uint32_t now = 0;
    float leftMm = 0, rightMm = 0;
    bool grayFresh = false;
    uint8_t digital = 0xff;
};
struct Output {
    Command command = Command::Stop;
    int leftPwm = 0, rightPwm = 0;
    float scale = 1;
};

// Hardware-independent sequencer. Every stage boundary emits Stop and starts
// a new encoder origin; callers must reset their outer/speed loops on Stop/Pwm.
class Controller {
public:
    bool start(const Plan &plan, const Config &config, const Input &in) {
        stop();
        if (!plan.count || plan.count > 12 || config.nextPoint > 3 ||
            (config.pointStep != 1 && config.pointStep != -1) ||
            config.totalTimeoutMs == 0 || config.totalTimeoutMs > 120000 ||
            !std::isfinite(in.leftMm) || !std::isfinite(in.rightMm)) return false;
        bool romanBefore = false;
        for (uint8_t i = 0; i < plan.count; ++i) {
            const Step &s = plan.steps[i];
            if (!std::isfinite(s.first) || !std::isfinite(s.second) ||
                s.timeoutMs == 0 || s.timeoutMs > 120000) return false;
            switch (s.action) {
                case Action::FollowDistance:
                    if (s.first <= 0 || s.first > 2000) return false; break;
                case Action::Move:
                    if (std::fabs(s.first) > 1500 || std::fabs(s.second) > 1500 ||
                        (std::fabs(s.first) <= 3 && std::fabs(s.second) <= 3)) return false; break;
                case Action::Route: {
                    if (s.legs > 4 || (s.legs == 0 && !romanBefore)) return false;
                    route::Controller check;
                    auto c = config.route; c.firstLegMm = s.first;
                    c.stopAfterLegs = s.legs ? s.legs : 4; c.maxTaskMs = s.timeoutMs;
                    if (!check.start(route::Task::Lap, c, in.now, in.leftMm, in.rightMm)) return false;
                    break;
                }
                case Action::ReadRoman: romanBefore = true; break;
                default: return false;
            }
        }
        plan_ = plan; config_ = config; started_ = in.now;
        state_ = Status::Running; begin(in); return state_ == Status::Running;
    }
    void stop() {
        state_ = Status::Idle; index_ = 0; target_ = -1; error_ = Error::None;
        parking_.stop(); route_.stop(); stable_.reset();
    }
    bool needsGray() const {
        return state_ == Status::Running &&
            (plan_.steps[index_].action == Action::FollowDistance || plan_.steps[index_].action == Action::Route);
    }
    bool waitingVision() const { return state_ == Status::Running && plan_.steps[index_].action == Action::ReadRoman; }
    void receive(const vision::Frame &frame, uint32_t now) {
        if (!waitingVision()) return;
        if (frame.kind != vision::Kind::Roman) return;
        const uint8_t result = stable_.accept(frame, now, vision::Kind::Roman);
        target_ = result ? vision::romanPoint(result) : -1;
    }
    Status status() const { return state_; }
    Error error() const { return error_; }
    uint8_t phase() const { return index_; }
    int target() const { return target_; }
    Output update(const Input &in) {
        if (state_ != Status::Running) return {};
        const Step &s = plan_.steps[index_];
        if (!std::isfinite(in.leftMm) || !std::isfinite(in.rightMm)) return fail(Error::Feedback);
        if (in.now - started_ >= config_.totalTimeoutMs || in.now - phaseStarted_ >= s.timeoutMs)
            return fail(s.action == Action::ReadRoman ? Error::Vision : Error::Timeout);
        if (needsGray() && in.now - lastGray_ > 150) return fail(Error::Sensor);
        if (in.grayFresh) lastGray_ = in.now;
        if (s.action == Action::ReadRoman) {
            if (target_ >= 0) return advance(in);
            return {};
        }
        if (s.action == Action::Move) {
            if (in.now - lastMotion_ < 20) return hold();
            if (in.now - lastMotion_ > 150) return fail(Error::Timeout);
            lastMotion_ = in.now;
            const auto out = parking_.update(in.now, in.leftMm, in.rightMm);
            if (parking_.status == parking::Status::Complete) return advance(in);
            if (parking_.status != parking::Status::Running) return fail(Error::Motion);
            return pwm(out.left, out.right);
        }
        if (!in.grayFresh) return hold();
        if (s.action == Action::Route) {
            const auto out = route_.update(in.digital, in.now, in.leftMm, in.rightMm);
            if (out.status == route::Status::Complete) return advance(in);
            if (out.status == route::Status::Fault) return fail(Error::Sensor);
            if (out.status == route::Status::Turn) return pwm(out.leftPwm, out.rightPwm);
            if (in.digital == 0) return follow(0.6f); // permitted by Route's mark validation
            return follow(out.approachingPoint ? 0.6f : 1.0f);
        }
        if (in.digital == 0xff) return fail(Error::Sensor);
        const float l = in.leftMm - originLeft_, r = in.rightMm - originRight_;
        const float progress = (l + r) * 0.5f;
        if (in.digital == 0) {
            if (leftStartMark_ || progress > std::fmin(100.0f, s.first * 0.25f)) return fail(Error::Sensor);
        } else leftStartMark_ = true;
        if (l < -10 || r < -10 || (l + r) * 0.5f > s.first + 120) return fail(Error::Feedback);
        if (in.leftMm - progressLeft_ >= 0.5f) { progressLeft_ = in.leftMm; leftMoved_ = in.now; }
        if (in.rightMm - progressRight_ >= 0.5f) { progressRight_ = in.rightMm; rightMoved_ = in.now; }
        if (in.now - leftMoved_ >= 1000 || in.now - rightMoved_ >= 1000) return fail(Error::Feedback);
        if ((l + r) * 0.5f >= s.first) return advance(in);
        return follow(in.digital == 0 || s.first - progress < 100 ? 0.6f : 1.0f);
    }
private:
    Plan plan_{}; Config config_{};
    Status state_ = Status::Idle; Error error_ = Error::None;
    bool leftStartMark_ = false;
    uint8_t index_ = 0; int target_ = -1;
    uint32_t started_ = 0, phaseStarted_ = 0, lastGray_ = 0, lastMotion_ = 0;
    uint32_t leftMoved_ = 0, rightMoved_ = 0;
    float originLeft_ = 0, originRight_ = 0, progressLeft_ = 0, progressRight_ = 0;
    parking::Controller parking_; route::Controller route_; vision::StableResult stable_;
    static Output hold() { Output o; o.command = Command::Hold; return o; }
    static Output follow(float scale) { Output o; o.command = Command::Follow; o.scale = scale; return o; }
    static Output pwm(int l, int r) { Output o; o.command = Command::Pwm; o.leftPwm = l; o.rightPwm = r; return o; }
    Output fail(Error e) { error_ = e; state_ = Status::Fault; parking_.stop(); route_.stop(); return {}; }
    Output advance(const Input &in) {
        parking_.stop(); route_.stop();
        if (++index_ == plan_.count) state_ = Status::Complete;
        else begin(in);
        return {};
    }
    void begin(const Input &in) {
        phaseStarted_ = lastGray_ = lastMotion_ = leftMoved_ = rightMoved_ = in.now;
        leftStartMark_ = false;
        originLeft_ = progressLeft_ = in.leftMm; originRight_ = progressRight_ = in.rightMm;
        const Step &s = plan_.steps[index_];
        if (s.action == Action::Move) {
            parking::Plan p; p.steps[0] = parking::Segment(s.first, s.second); p.count = 1;
            if (!parking_.start(p, in.now, in.leftMm, in.rightMm)) fail(Error::Configuration);
        } else if (s.action == Action::Route) {
            auto c = config_.route; c.firstLegMm = s.first; c.maxTaskMs = s.timeoutMs;
            c.stopAfterLegs = s.legs ? s.legs :
                static_cast<uint8_t>(((target_ - config_.nextPoint) * config_.pointStep + 4) % 4 + 1);
            if (!route_.start(route::Task::Lap, c, in.now, in.leftMm, in.rightMm)) fail(Error::Configuration);
        } else if (s.action == Action::ReadRoman) { stable_.reset(); target_ = -1; }
    }
};

// Geometry supplied by measurements, never guessed by the sequencer.
inline bool appendParking(Plan &out, const parking::Plan &p) {
    if (!p.count || p.count > 3) return false;
    for (size_t i = 0; i < p.count; ++i)
        if (!out.add(Action::Move, p.steps[i].leftMm, p.steps[i].rightMm, 0, 10000)) return false;
    return true;
}
} // namespace mission
