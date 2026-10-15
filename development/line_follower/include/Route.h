#pragma once
#include <cmath>
#include <cstdint>

namespace route {
constexpr float kPi = 3.14159265359f;

enum class Status : uint8_t { Idle, Follow, Turn, Complete, Fault };
enum class Task : uint8_t { AB, Lap };
enum class StopPoint : uint8_t { None, B, A };

struct Config {
    float sideMm = 1000.0f;
    float wheelTrackMm = 0.0f;
    int turnSign = 0; // +1 left, -1 right; must be checked on the course
    uint32_t maxTaskMs = 30000;
    float startClearMm = 160.0f;       // Drive clear of the A start cross first.
    float approachWindowMm = 220.0f;  // Reduce speed before the target point.
    float markerOvershootMm = 140.0f; // Stop safely if the expected cross was missed.
    uint8_t markerSamples = 2;        // Full-width black cross must be stable.
    uint8_t acquireSamples = 3;       // Center-line reacquisition after a turn.
};

struct Result {
    Status status = Status::Idle;
    int leftPwm = 0;
    int rightPwm = 0;
    uint8_t legsCompleted = 0;
    StopPoint stopPoint = StopPoint::None;
    bool approachingPoint = false;
};

class Controller {
public:
    bool start(Task task, const Config &config, uint32_t now,
               float leftMm, float rightMm) {
        stop();
        if (!std::isfinite(config.sideMm) || config.sideMm < 500 ||
            config.sideMm > 2000 ||
            !std::isfinite(config.wheelTrackMm) || config.wheelTrackMm < 50 ||
            config.wheelTrackMm > 400 ||
            !std::isfinite(config.startClearMm) ||
            config.startClearMm < 50 || config.startClearMm > 400 ||
            !std::isfinite(config.approachWindowMm) ||
            config.approachWindowMm < 80 || config.approachWindowMm > 400 ||
            !std::isfinite(config.markerOvershootMm) ||
            config.markerOvershootMm < 40 || config.markerOvershootMm > 300 ||
            config.markerSamples == 0 || config.markerSamples > 5 ||
            config.acquireSamples == 0 || config.acquireSamples > 8 ||
            (task == Task::Lap && config.turnSign != 1 && config.turnSign != -1) ||
            !std::isfinite(leftMm) || !std::isfinite(rightMm) ||
            config.maxTaskMs == 0) return false;
        task_ = task;
        config_ = config;
        started_ = now;
        leftOrigin_ = leftMm;
        rightOrigin_ = rightMm;
        phaseStarted_ = now;
        status_ = Status::Follow;
        return true;
    }

    void stop() {
        status_ = Status::Idle;
        legsCompleted_ = 0;
        markerCandidate_ = 0;
        clearCandidate_ = 0;
        markerArmed_ = false;
        approachingPoint_ = false;
        stopPoint_ = StopPoint::None;
    }

    Status status() const { return status_; }

    // digital: 1=white, 0=black, bit0=probe1. A point marker must make
    // every probe report black (0x00) for consecutive samples. The first
    // cross at A is ignored until the sensor clears the configured start zone.
    Result update(uint8_t digital, uint32_t now, float leftMm, float rightMm) {
        if (status_ == Status::Idle || status_ == Status::Complete ||
            status_ == Status::Fault) return result();
        if (!std::isfinite(leftMm) || !std::isfinite(rightMm) ||
            now - started_ >= config_.maxTaskMs) return fail();

        if (status_ == Status::Follow) return follow(digital, now, leftMm, rightMm);
        return turn(digital, now, leftMm, rightMm);
    }

private:
    Task task_ = Task::AB;
    Config config_{};
    Status status_ = Status::Idle;
    uint32_t started_ = 0, phaseStarted_ = 0;
    float leftOrigin_ = 0, rightOrigin_ = 0;
    uint8_t legsCompleted_ = 0;
    uint8_t markerCandidate_ = 0, clearCandidate_ = 0;
    bool markerArmed_ = false, approachingPoint_ = false;
    StopPoint stopPoint_ = StopPoint::None;

    float progress(float leftMm, float rightMm) const {
        return ((leftMm - leftOrigin_) + (rightMm - rightOrigin_)) * 0.5f;
    }

    Result follow(uint8_t digital, uint32_t now, float leftMm, float rightMm) {
        const float distance = progress(leftMm, rightMm);
        if (now - phaseStarted_ >= 12000 || distance < -30 ||
            distance > config_.sideMm + config_.markerOvershootMm ||
            digital == 0xff) return fail();

        approachingPoint_ = distance >= config_.sideMm - config_.approachWindowMm;
        if (!markerArmed_) {
            markerCandidate_ = 0;
            if (distance >= config_.startClearMm && digital != 0x00) {
                if (++clearCandidate_ >= 2) markerArmed_ = true;
            } else {
                clearCandidate_ = 0;
            }
            return result();
        }

        if (digital == 0x00) {
            if (distance < config_.sideMm - config_.approachWindowMm)
                return fail(); // Unexpected cross before the target point.
            if (++markerCandidate_ >= config_.markerSamples) {
                markerCandidate_ = 0;
                ++legsCompleted_;
                stopPoint_ = task_ == Task::AB ? StopPoint::B :
                    (legsCompleted_ == 4 ? StopPoint::A : StopPoint::None);
                if (stopPoint_ != StopPoint::None) {
                    status_ = Status::Complete; // Motor output is zero on completion.
                } else {
                    status_ = Status::Turn;
                    phaseStarted_ = now;
                    leftOrigin_ = leftMm;
                    rightOrigin_ = rightMm;
                }
            }
        } else {
            markerCandidate_ = 0;
        }
        return result();
    }

    Result turn(uint8_t digital, uint32_t now, float leftMm, float rightMm) {
        // In-place quarter turn: encoder travel determines angle; gray probes
        // confirm that the new straight line has been reacquired.
        if (now - phaseStarted_ >= 3500) return fail();
        const float arc = config_.wheelTrackMm * kPi / 4.0f;
        const float leftTravel = leftMm - leftOrigin_;
        const float rightTravel = rightMm - rightOrigin_;
        const float turnProgress = config_.turnSign * (rightTravel - leftTravel) * 0.5f;
        if (turnProgress < -10 || turnProgress > arc * 1.30f) return fail();

        const uint8_t black = static_cast<uint8_t>(~digital);
        const bool centered = black != 0 && black != 0xff &&
            (black & 0x18u) != 0 && (black & 0x81u) == 0;
        if (turnProgress >= arc * 0.78f && centered) {
            if (++clearCandidate_ >= config_.acquireSamples) {
                clearCandidate_ = 0;
                markerCandidate_ = 0;
                markerArmed_ = false; // Clear the old intersection/start mark.
                status_ = Status::Follow;
                phaseStarted_ = now;
                leftOrigin_ = leftMm;
                rightOrigin_ = rightMm;
            }
        } else {
            clearCandidate_ = 0;
        }
        return result();
    }

    Result fail() {
        status_ = Status::Fault;
        approachingPoint_ = false;
        return result();
    }

    Result result() const {
        Result out;
        out.status = status_;
        out.legsCompleted = legsCompleted_;
        out.stopPoint = stopPoint_;
        out.approachingPoint = approachingPoint_;
        if (status_ == Status::Turn) {
            out.leftPwm = -config_.turnSign * 55;
            out.rightPwm = config_.turnSign * 55;
        }
        return out;
    }
};
} // namespace route