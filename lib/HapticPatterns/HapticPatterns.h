#pragma once

#include <stdint.h>

namespace haptics
{
constexpr unsigned MOTOR_COUNT = 3;
constexpr uint32_t BREATH_CYCLE_MS = 12000UL;
constexpr uint32_t DEFAULT_P1_DURATION_MS = 5000UL;
constexpr uint32_t P1_MOTOR_ON_MS = 200UL;
constexpr uint16_t DEFAULT_FOLLOW_UP_CYCLES = 1; // One 12-second follow-up cycle.
constexpr uint32_t MIN_BURST_MS = 60UL;

enum class Protocol : uint8_t { IDLE, P1_FLUTTER, P1_FOLLOW_UP, BOOT_TEST };

// Logical order: Motor 1 left, Motor 2 center/P6, Motor 3 right.
// Levels are thousandths of the configured driver amplitude (0..1000).
struct Frame
{
    uint16_t level[MOTOR_COUNT] = {0, 0, 0};
};

// Generates amplitudes only; hardware transport and detector confidence are
// separate responsibilities. No sleeps, I2C operations, or catch-up bursts.
class Patterns
{
public:
    bool startBootTest(uint32_t now);
    bool startP1(uint32_t now, uint32_t seed,
                 uint32_t durationMs = DEFAULT_P1_DURATION_MS);
    bool startFollowUp(uint32_t now, uint16_t cycles = DEFAULT_FOLLOW_UP_CYCLES);
    const Frame &update(uint32_t now);
    void stop();
    bool active() const { return protocol_ != Protocol::IDLE; }
    Protocol protocol() const { return protocol_; }
    const Frame &frame() const { return frame_; }
    uint32_t elapsedMs(uint32_t now) const;
    uint32_t durationMs() const { return durationMs_; }

private:
    Frame frame_;
    Protocol protocol_ = Protocol::IDLE;
    uint32_t startedMs_ = 0;
    uint32_t durationMs_ = 0;
    uint32_t randomState_ = 1;
    uint8_t p1Sequence_[MOTOR_COUNT] = {};
    uint32_t p1Slot_ = UINT32_MAX;

    uint32_t randomBetween(uint32_t minimum, uint32_t maximum);
    void generateP1Sequence();
    void updateFlutter(uint32_t now, uint32_t elapsed);
    void updateSweep(uint32_t elapsed);
};
} // namespace haptics
