#pragma once

#include <stdint.h>

namespace haptics
{
constexpr unsigned MOTOR_COUNT = 3;
constexpr uint32_t BREATH_CYCLE_MS = 10000UL;
constexpr uint32_t DEFAULT_P1_DURATION_MS = 5000UL;
constexpr uint16_t DEFAULT_P2_CYCLES = 1; // One 10-second cycle for testing.
constexpr uint32_t MIN_BURST_MS = 60UL;
constexpr uint32_t MAX_BURST_MS = 250UL;
constexpr uint32_t MIN_GAP_MS = 50UL;
constexpr uint32_t MAX_GAP_MS = 450UL;

enum class Protocol : uint8_t { IDLE, P1_FLUTTER, P2_SWEEP, BOOT_TEST };

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
    bool startP2(uint32_t now, uint16_t cycles = DEFAULT_P2_CYCLES);
    const Frame &update(uint32_t now);
    void stop();
    bool active() const { return protocol_ != Protocol::IDLE; }
    Protocol protocol() const { return protocol_; }
    const Frame &frame() const { return frame_; }
    uint32_t elapsedMs(uint32_t now) const;
    uint32_t durationMs() const { return durationMs_; }

private:
    struct Tap
    {
        bool on = false;
        uint32_t since = 0;
        uint32_t duration = 0;
    };
    Tap taps_[MOTOR_COUNT];
    Frame frame_;
    Protocol protocol_ = Protocol::IDLE;
    uint32_t startedMs_ = 0;
    uint32_t durationMs_ = 0;
    uint32_t randomState_ = 1;

    uint32_t randomBetween(uint32_t minimum, uint32_t maximum);
    void updateFlutter(uint32_t now, uint32_t elapsed);
    void updateSweep(uint32_t elapsed);
};
} // namespace haptics
