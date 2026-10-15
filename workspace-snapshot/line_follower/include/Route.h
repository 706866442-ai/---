#pragma once
#include <cmath>
#include <cstdint>

namespace route {
constexpr float kPi = 3.14159265359f;

enum class Status : uint8_t { Idle, Follow, Turn, Complete, Fault };
enum class Task : uint8_t { AB, Lap };

struct Config {
    float sideMm = 1000.0f;
    float wheelTrackMm = 0.0f;
    int turnSign = 0; // +1 left, -1 right; must be measured on the course
    uint32_t maxTaskMs = 30000;
    float firstLegMm = 0; // 0 uses sideMm; nonzero supports joining mid-edge.
    uint8_t stopAfterLegs = 0; // 0 uses Task; otherwise 1..4.

};

struct Result {
    Status status = Status::Idle;
    int leftPwm = 0;
    int rightPwm = 0;
    uint8_t legsCompleted = 0;
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
            (task == Task::Lap && config.turnSign != 1 && config.turnSign != -1) ||
            !std::isfinite(leftMm) || !std::isfinite(rightMm) ||
            config.maxTaskMs == 0 || config.maxTaskMs > 120000 ||
            !std::isfinite(config.firstLegMm) || config.firstLegMm < 0 ||
            (config.firstLegMm > 0 && (config.firstLegMm < 200 || config.firstLegMm > 2000)) ||
            config.stopAfterLegs > 4 ||
            (config.stopAfterLegs > 1 && config.turnSign != 1 && config.turnSign != -1)) return false;
        task_ = task; config_ = config; started_ = now;
        leftOrigin_ = leftMm; rightOrigin_ = rightMm;
        phaseStarted_ = now; status_ = Status::Follow;
        armed_ = false; departureSamples_ = 0; approaching_ = false;
        return true;
    }

    void stop() { status_ = Status::Idle; legsCompleted_ = 0; candidate_ = 0; }
    Status status() const { return status_; }

    // digital: 1=white, 0=black, bit0=probe1. Motors remain stopped on
    // Complete/Fault; no blind recovery when the line disappears.
    Result update(uint8_t digital, uint32_t now, float leftMm, float rightMm) {
        if (status_ == Status::Idle || status_ == Status::Complete ||
            status_ == Status::Fault) return result();
        if (!std::isfinite(leftMm) || !std::isfinite(rightMm) ||
            now - started_ >= config_.maxTaskMs) return fail();

        if (status_ == Status::Follow) {
            if (now - phaseStarted_ >= 12000) return fail();
            const float l = leftMm - leftOrigin_;
            const float r = rightMm - rightOrigin_;
            const float progress = (l + r) * 0.5f;
            const float length = legsCompleted_ == 0 && config_.firstLegMm > 0
                ? config_.firstLegMm : config_.sideMm;
            const float window = std::fmin(160.0f, length * 0.25f);
            approaching_ = progress >= length - window;
            if (digital != 0 && digital != 0xff) {
                if (departureSamples_ < 2) ++departureSamples_;
                if (departureSamples_ >= 2 && progress >= std::fmin(160.0f, length * 0.25f)) armed_ = true;
            } else if (!armed_) departureSamples_ = 0;
            if (l < -30 || r < -30 || progress > length + 120)
                return fail();
            if (digital == 0xff) return fail();
            // A perpendicular crossing covers all probes. Require two
            // consecutive samples and travel near the end of the leg.
            if (digital == 0x00 && armed_ && progress >= length - window) {
                if (++candidate_ >= 2) {
                    candidate_ = 0;
                    ++legsCompleted_;
                    if (legsCompleted_ >= (config_.stopAfterLegs ? config_.stopAfterLegs : (task_ == Task::AB ? 1 : 4))) {
                        status_ = Status::Complete;
                    } else {
                        status_ = Status::Turn;
                        phaseStarted_ = now;
                        leftOrigin_ = leftMm; rightOrigin_ = rightMm;
                        lastTurnLeft_ = leftMm; lastTurnRight_ = rightMm;
                        movedLeftMs_ = movedRightMs_ = now;
                    }
                }
            } else {
                candidate_ = 0;
                // The crossing might not produce 0x00 on this sensor/track.
                // Stop for calibration instead of guessing a timed corner.
                if (progress >= length + 80) return fail();
                // Allow a short all-black start mark at A. Else a crossing
                // this early is ambiguous and must stop the vehicle.
                if (digital == 0x00 && progress > 100) return fail();
            }
            return result();
        }

        // In-place quarter turn. Require ~70 degrees of wheel travel before
        // accepting the new centered line, avoiding immediate reacquisition
        // of the old line. Give up beyond 110 degrees.
        if (now - phaseStarted_ >= 3500) return fail();
        if (config_.turnSign * (lastTurnLeft_ - leftMm) >= 0.5f) {
            lastTurnLeft_ = leftMm; movedLeftMs_ = now;
        }
        if (config_.turnSign * (rightMm - lastTurnRight_) >= 0.5f) {
            lastTurnRight_ = rightMm; movedRightMs_ = now;
        }
        if (now - movedLeftMs_ >= 800 || now - movedRightMs_ >= 800) return fail();
        const float arc = config_.wheelTrackMm * kPi / 4.0f;
        const float l = leftMm - leftOrigin_;
        const float r = rightMm - rightOrigin_;
        if (config_.turnSign * l > 5 || config_.turnSign * r < -5) return fail();
        const float turnProgress = (config_.turnSign * (r - l)) * 0.5f;
        if (turnProgress < -10 || turnProgress > arc * 1.30f) return fail();
        const uint8_t black = static_cast<uint8_t>(~digital);
        const bool centered = black != 0 && black != 0xff &&
            (black & 0x18u) != 0 && (black & 0x81u) == 0;
        if (turnProgress >= arc * 0.78f && -config_.turnSign * l >= arc * 0.65f &&
            config_.turnSign * r >= arc * 0.65f && centered) {
            if (++candidate_ >= 3) {
                candidate_ = 0;
                status_ = Status::Follow;
                armed_ = false; departureSamples_ = 0; approaching_ = false;
                phaseStarted_ = now;
                leftOrigin_ = leftMm; rightOrigin_ = rightMm;
            }
        } else candidate_ = 0;
        return result();
    }

private:
    Task task_ = Task::AB;
    Config config_{};
    Status status_ = Status::Idle;
    uint32_t started_ = 0, phaseStarted_ = 0;
    float leftOrigin_ = 0, rightOrigin_ = 0;
    uint8_t legsCompleted_ = 0, candidate_ = 0, departureSamples_ = 0;
    bool armed_ = false, approaching_ = false;
    float lastTurnLeft_ = 0, lastTurnRight_ = 0;
    uint32_t movedLeftMs_ = 0, movedRightMs_ = 0;

    Result fail() { status_ = Status::Fault; return result(); }
    Result result() const {
        Result out;
        out.status = status_;
        out.legsCompleted = legsCompleted_;
        out.approachingPoint = approaching_;
        if (status_ == Status::Turn) {
            out.leftPwm = -config_.turnSign * 65;
            out.rightPwm = config_.turnSign * 65;
        }
        return out;
    }
};
} // namespace route
