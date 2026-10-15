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
};

struct Result {
    Status status = Status::Idle;
    int leftPwm = 0;
    int rightPwm = 0;
    uint8_t legsCompleted = 0;
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
            config.maxTaskMs == 0) return false;
        task_ = task; config_ = config; started_ = now;
        leftOrigin_ = leftMm; rightOrigin_ = rightMm;
        phaseStarted_ = now; status_ = Status::Follow;
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
            if (l < -30 || r < -30 || progress > config_.sideMm + 120)
                return fail();
            if (digital == 0xff) return fail();
            // A perpendicular crossing covers all probes. Require two
            // consecutive samples and travel near the end of the leg.
            if (digital == 0x00 && progress >= config_.sideMm - 160) {
                if (++candidate_ >= 2) {
                    candidate_ = 0;
                    ++legsCompleted_;
                    if (task_ == Task::AB || legsCompleted_ == 4) {
                        status_ = Status::Complete;
                    } else {
                        status_ = Status::Turn;
                        phaseStarted_ = now;
                        leftOrigin_ = leftMm; rightOrigin_ = rightMm;
                    }
                }
            } else {
                candidate_ = 0;
                // The crossing might not produce 0x00 on this sensor/track.
                // Stop for calibration instead of guessing a timed corner.
                if (progress >= config_.sideMm + 80) return fail();
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
        const float arc = config_.wheelTrackMm * kPi / 4.0f;
        const float l = leftMm - leftOrigin_;
        const float r = rightMm - rightOrigin_;
        const float turnProgress = (config_.turnSign * (r - l)) * 0.5f;
        if (turnProgress < -10 || turnProgress > arc * 1.30f) return fail();
        const uint8_t black = static_cast<uint8_t>(~digital);
        const bool centered = black != 0 && black != 0xff &&
            (black & 0x18u) != 0 && (black & 0x81u) == 0;
        if (turnProgress >= arc * 0.78f && centered) {
            if (++candidate_ >= 3) {
                candidate_ = 0;
                status_ = Status::Follow;
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
    uint8_t legsCompleted_ = 0, candidate_ = 0;

    Result fail() { status_ = Status::Fault; return result(); }
    Result result() const {
        Result out;
        out.status = status_;
        out.legsCompleted = legsCompleted_;
        if (status_ == Status::Turn) {
            out.leftPwm = -config_.turnSign * 65;
            out.rightPwm = config_.turnSign * 65;
        }
        return out;
    }
};
} // namespace route
