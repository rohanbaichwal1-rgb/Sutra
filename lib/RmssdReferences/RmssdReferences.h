#pragma once

#include <stdint.h>

// Independent of Arduino so elapsed-time and reference behavior can be tested
// with simulated readings through PlatformIO's native test environment.
namespace rmssd
{
constexpr uint32_t WINDOW_MS = 300000UL;
constexpr uint32_t SAMPLE_MS = 5000UL;
constexpr unsigned CAPACITY = 60;
constexpr unsigned MIN_SAMPLES = 12; // 12 complete qualifying 5-second blocks = 60 s
constexpr uint32_t RECOVERY_HOLD_MS = 60000UL;
constexpr uint32_t POST_EXERCISE_RECOVERY_MS = 60000UL;
constexpr uint32_t MAX_REPORT_GAP_MS = 2500UL;
constexpr uint32_t P1_HOLD_MS = 30000UL;
constexpr uint32_t P1_FOLLOW_UP_DELAY_MS = 5000UL;
constexpr uint8_t P1_MAX_CONSECUTIVE_BLANK_DEV = 10;
// Prototype test thresholds: percentage drop below each available reference.
constexpr float P1_DROP_PCT = 15.0f;

enum class FollowUp : uint8_t { IDLE, PENDING, ISSUED, CANCELLED };
enum class FollowUpEvent : uint8_t { NONE, START, CANCELLED };
const char *followUpName(FollowUp state);

enum class Source : uint8_t { NONE = 0, LONG_TERM = 1, SHORT_TERM = 2, BOTH = 3,
                              SESSION = 4, LONG_AND_SESSION = 5, SHORT_AND_SESSION = 6, ALL = 7 };
enum class Motion : uint8_t { LOW_MOTION, MODERATE_MOTION, HIGH_MOTION };
const char *sourceName(Source source);

enum class CollectionGate : uint8_t
{
    OPEN, SIGNAL_OR_IMU, MOTION, POST_EXERCISE, UNSTABLE,
    INTERVENTION
};
const char *collectionGateName(CollectionGate gate);

struct Hold
{
    bool active = false;
    uint32_t since = 0;
    uint8_t blankCount = 0;
    void update(bool condition, uint32_t now);
    void updateP1(bool condition, bool blankDev, uint32_t now);
    void reset();
    uint32_t elapsed(uint32_t now) const;
    bool reached(uint32_t now, uint32_t duration) const;
};

struct Input
{
    uint32_t now;
    float current;
    bool signalGood; // fresh, valid RMSSD and LMS-HR
    Motion motion;
    bool stable; // fixed resting baseline/rearm only, not live short adaptation
    bool interventionActive;
    bool longTermAvailable;
    float longTermBaseline; // persisted seven-session prototype baseline
    bool sessionAvailable;
    float sessionBaseline; // frozen median from the current startup
};

struct Events
{
    bool resumed = false;
    Source p1 = Source::NONE;
};

class Monitor
{
public:
    Events update(const Input &input);
    // Call every loop after current sensor/report processing; consumes each
    // start/cancellation once. Does not collect baselines or advance P1 holds.
    FollowUpEvent pollFollowUp(const Input &input);
    FollowUp followUp() const { return followUp_; }
    bool followUpNeedsPolling() const { return followUp_ == FollowUp::PENDING || followUpCancellation_; }
    // Runtime haptics call this only after the five-second P1 pattern has
    // stopped successfully.  The five-second breathing delay begins here,
    // not at physiological P1 detection.
    void armFollowUpCountdown(uint32_t now);
    // Runtime haptics call this when the full P1 intervention has ended,
    // either after the breathing pattern or an unrecoverable hardware stop.
    // A new episode is rearmed only after all P1 persistence holds are zero.
    void completeP1Intervention();
    // A successfully completed 12-second follow-up ends its physiological
    // episode immediately. A still-low DEV starts a fresh 30-second hold.
    void completeP1FollowUp();
    void cancelFollowUp();
    // Call at IMU cadence, so HIGH movement between reports breaks holds.
    // MODERATE motion preserves normal P1 counting.
    void observeMotion(uint32_t now, Motion motion);
    // Contact changes discard rolling history, but retain fired episode latches
    // until qualified recovery. A dropout must not re-arm an intervention.
    void contactChanged();
    void interruptContinuity(bool resetP1 = true);
    // A brief missing DEV reading interrupts collection/recovery, but leaves
    // an already-running P1 hold for updateP1()'s bounded blank tolerance.
    void interruptSignalContinuity();

    bool shortAvailable() const { return shortReady_; }
    float shortReference() const { return shortValue_; }
    bool postExerciseRecovery() const { return motionRecovery_; }
    unsigned sampleCount() const { return count_; }
    uint32_t validDurationMs() const { return count_ * SAMPLE_MS; }
    CollectionGate collectionGate() const { return collectionGate_; }
    // Resting-data quality only, independent of drop/episode state.
    CollectionGate restingGate() const { return restingGate_; }
    uint32_t sampleBlockMs(uint32_t now) const { return sampleBlock_.elapsed(now); }
    uint32_t collectionBreaks() const { return collectionBreaks_; }
    uint32_t motionRecoveryRemainingMs(uint32_t now) const;
    bool p1Triggered() const { return p1Fired_; }
    Source p1Source() const { return p1Source_; }
    uint32_t p1LongHoldMs(uint32_t now) const { return p1Long_.elapsed(now); }
    uint32_t p1ShortHoldMs(uint32_t now) const { return p1Short_.elapsed(now); }
    uint32_t p1SessionHoldMs(uint32_t now) const { return p1Session_.elapsed(now); }
    uint32_t recoveryHoldMs(uint32_t now) const { return recovery_.elapsed(now); }
    static float deviation(float current, float reference);

private:
    struct Sample { float value; uint32_t time; };
    Sample samples_[CAPACITY] = {};
    unsigned head_ = 0;
    unsigned count_ = 0;
    float shortValue_ = 0.0f;
    bool shortReady_ = false;
    bool p1Fired_ = false;
    Source p1Source_ = Source::NONE;
    bool interventionComplete_ = false;
    FollowUp followUp_ = FollowUp::IDLE;
    bool followUpCancellation_ = false;
    uint32_t followUpSince_ = 0;
    uint32_t followUpLastCheck_ = 0;
    bool motionRecovery_ = false;
    // Arm only after the first complete resting collection block, so boot
    // transients and initial sensor placement cannot start a recovery wait.
    bool motionRecoveryArmed_ = false;
    uint32_t recoveryLowMs_ = 0;
    uint32_t lastMotionMs_ = 0;
    bool haveMotion_ = false;
    Motion previousMotion_ = Motion::LOW_MOTION;
    Hold sampleBlock_;
    CollectionGate collectionGate_ = CollectionGate::SIGNAL_OR_IMU;
    CollectionGate restingGate_ = CollectionGate::SIGNAL_OR_IMU;
    uint32_t collectionBreaks_ = 0;
    Hold recovery_;
    Hold p1Long_, p1Short_, p1Session_;
    float lastLongBaseline_ = 0.0f;
    float lastSessionBaseline_ = 0.0f;
    bool haveReport_ = false;
    uint32_t lastReport_ = 0;

    void expire(uint32_t now);
    void calculateMedian();
    void append(float value, uint32_t now);
    void checkFollowUp(const Input &input);
    bool p1CountersReset() const;
    void resetP1Episode();
};
} // namespace rmssd
