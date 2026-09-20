#pragma once

#include <stdint.h>

// Independent of Arduino so elapsed-time and reference behavior can be tested
// with simulated readings through PlatformIO's native test environment.
namespace rmssd
{
constexpr uint32_t WINDOW_MS = 300000UL;
constexpr uint32_t SAMPLE_MS = 5000UL;
constexpr unsigned CAPACITY = 60;
constexpr unsigned MIN_SAMPLES = 36; // 36 complete qualifying 5-second blocks
constexpr uint32_t FREEZE_HOLD_MS = 10000UL;
constexpr uint32_t RECOVERY_HOLD_MS = 60000UL;
constexpr uint32_t POST_EXERCISE_RECOVERY_MS = 120000UL;
constexpr uint32_t MAX_REPORT_GAP_MS = 2500UL;
constexpr uint32_t P1_HOLD_MS = 30000UL;
constexpr uint32_t P2_HOLD_MS = 120000UL;

enum class Source : uint8_t { NONE = 0, LONG_TERM = 1, SHORT_TERM = 2, BOTH = 3 };
const char *sourceName(Source source);

struct Hold
{
    bool active = false;
    uint32_t since = 0;
    void update(bool condition, uint32_t now);
    uint32_t elapsed(uint32_t now) const;
    bool reached(uint32_t now, uint32_t duration) const;
};

struct Input
{
    uint32_t now;
    float current;
    bool signalGood; // fresh, valid RMSSD and LMS-HR
    bool lowMotion;
    bool stable;
    bool interventionActive;
    bool sessionAvailable;
    float sessionBaseline; // PoC substitute for the future daily baseline
};

struct Events
{
    bool froze = false;
    bool resumed = false;
    Source p1 = Source::NONE;
    Source p2 = Source::NONE;
};

class Monitor
{
public:
    Events update(const Input &input);
    // Call at IMU cadence, so movement between reports also breaks holds.
    void observeMotion(uint32_t now, bool lowMotion);
    // Contact changes discard the rolling history, but retain a frozen episode
    // until qualified recovery. A dropout must not re-arm an intervention.
    void contactChanged();
    void interruptContinuity();

    bool shortAvailable() const { return shortReady_; }
    float shortReference() const { return shortValue_; }
    bool frozen() const { return frozen_; }
    bool preliminary() const { return preliminary_.active; }
    bool postExerciseRecovery() const { return motionRecovery_; }
    unsigned sampleCount() const { return count_; }
    uint32_t validDurationMs() const { return count_ * SAMPLE_MS; }
    bool p1Triggered() const { return p1Fired_; }
    bool p2Triggered() const { return p2Fired_; }
    Source p1Source() const { return p1Source_; }
    Source p2Source() const { return p2Source_; }
    uint32_t p1LongHoldMs(uint32_t now) const { return p1Long_.elapsed(now); }
    uint32_t p1ShortHoldMs(uint32_t now) const { return p1Short_.elapsed(now); }
    uint32_t p2LongHoldMs(uint32_t now) const { return p2Long_.elapsed(now); }
    uint32_t p2ShortHoldMs(uint32_t now) const { return p2Short_.elapsed(now); }
    uint32_t recoveryHoldMs(uint32_t now) const { return recovery_.elapsed(now); }
    static float deviation(float current, float reference);

private:
    struct Sample { float value; uint32_t time; };
    Sample samples_[CAPACITY] = {};
    unsigned head_ = 0;
    unsigned count_ = 0;
    float shortValue_ = 0.0f;
    bool shortReady_ = false;
    bool frozen_ = false;
    bool p1Fired_ = false;
    bool p2Fired_ = false;
    Source p1Source_ = Source::NONE;
    Source p2Source_ = Source::NONE;
    bool motionRecovery_ = false;
    Hold lowMotion_;
    Hold sampleBlock_;
    Hold preliminary_;
    Hold recovery_;
    Hold p1Long_, p1Short_, p2Long_, p2Short_;
    bool haveReport_ = false;
    uint32_t lastReport_ = 0;

    void expire(uint32_t now);
    void calculateMedian();
    void append(float value, uint32_t now);
    void freeze(Events &events);
};
} // namespace rmssd
