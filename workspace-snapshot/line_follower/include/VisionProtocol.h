#pragma once
#include <cstdint>
#include <cstddef>

namespace vision {
enum class Kind : uint8_t { Roman = 1, Color = 2, Shape = 3 };
struct Frame {
    Kind kind = Kind::Roman;
    uint8_t sequence = 0, value = 0, confidence = 0;
    uint32_t receivedMs = 0;
};
// Proposed v1 wire format: A5 01 kind sequence value confidence CRC8.
// CRC polynomial 0x07, initial 0, covers bytes 1..5. No pin assumptions.
inline uint8_t crc8(const uint8_t *data, size_t size) {
    uint8_t crc = 0;
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    return crc;
}
class Parser {
public:
    void reset() { count_ = 0; for (bool &v : seen_) v = false; }
    bool feed(uint8_t byte, uint32_t now, Frame &out) {
        if (count_ && now - lastByte_ > 100) count_ = 0;
        lastByte_ = now;
        if (!count_) { if (byte == 0xa5) bytes_[count_++] = byte; return false; }
        bytes_[count_++] = byte;
        if (count_ != 7) return false;
        count_ = 0;
        const uint8_t kind = bytes_[2], value = bytes_[4];
        if (bytes_[1] != 1 || kind < 1 || kind > 3 || bytes_[5] > 100 ||
            value > (kind == 1 ? 4 : (kind == 2 ? 2 : 3)) ||
            crc8(bytes_ + 1, 5) != bytes_[6]) {
            ++invalidFrames;
            if (byte == 0xa5) bytes_[count_++] = byte;
            return false;
        }
        const uint8_t delta = static_cast<uint8_t>(bytes_[3] - sequences_[kind - 1]);
        if (seen_[kind - 1] && (delta == 0 || delta >= 128)) { ++replayedFrames; return false; }
        seen_[kind - 1] = true; sequences_[kind - 1] = bytes_[3];
        out.kind = static_cast<Kind>(kind); out.sequence = bytes_[3];
        out.value = value; out.confidence = bytes_[5]; out.receivedMs = now;
        return true;
    }
    uint32_t invalidFrames = 0, replayedFrames = 0;
private:
    uint8_t bytes_[7] = {}, count_ = 0, sequences_[3] = {};
    bool seen_[3] = {};
    uint32_t lastByte_ = 0;
};
class StableResult {
public:
    void reset() { value_ = count_ = 0; seen_ = false; }
    uint8_t accept(const Frame &f, uint32_t now, Kind expected) {
        if (f.kind != expected) return 0;
        if (now - f.receivedMs > 300 || f.value == 0 || f.confidence < 80) { reset(); return 0; }
        if (seen_ && f.sequence == sequence_) return 0;
        if (!seen_ || now - lastMs_ > 300 || f.value != value_) count_ = 0;
        value_ = f.value; sequence_ = f.sequence; seen_ = true; lastMs_ = now;
        if (count_ < 3) ++count_;
        return count_ >= 3 ? value_ : 0;
    }
private:
    uint8_t value_ = 0, count_ = 0, sequence_ = 0;
    bool seen_ = false;
    uint32_t lastMs_ = 0;
};
// Roman values 1..4 encode I, III, V, VII -> point indices A=0..D=3.
inline int romanPoint(uint8_t value) { return value >= 1 && value <= 4 ? value - 1 : -1; }
// Red=1 reverse bay, yellow=2 parallel bay; circle=1 BC, triangle=2 CD, square=3 DA.
} // namespace vision
