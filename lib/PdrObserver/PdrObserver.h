#pragma once
#include <stdint.h>
#include <math.h>

// PPG respiration context only. This module has no trigger or actuator interface.
namespace pdr {
constexpr unsigned BEAT_CAPACITY = 256;
constexpr uint32_t WINDOW_MS = 45000;
constexpr uint32_t GRID_INTERVAL_MS = 250; // 4 Hz
constexpr unsigned GRID_SIZE = WINDOW_MS / GRID_INTERVAL_MS;
constexpr uint32_t ANALYSIS_INTERVAL_MS = 2000;
constexpr uint32_t MAX_GAP_MS = 2500;
constexpr float MIN_RATE = 6.0f, MAX_RATE = 30.0f; // 0.10-0.50 Hz
constexpr float RATE_STEP = 0.5f, MAX_DISAGREEMENT = 2.0f; // breaths/minute
constexpr float MIN_QUALITY = 0.50f;
constexpr float MIN_COVERAGE = 0.65f;
constexpr float RIAV_MIN_STD = 0.005f, RIFV_MIN_STD = 0.003f;
enum class Status : uint8_t { WARMUP, SIGNAL, MOTION, GAP, WEAK, DISAGREE, VALID, STALE };
const char *statusName(Status status);
enum class ChannelStatus : uint8_t { NOT_ANALYZED, LOW_MODULATION, LOW_PERIODICITY, NO_FIT, OK };
const char *channelStatusName(ChannelStatus status);
enum class ResetReason : uint8_t { NONE, CONTACT, SIGNAL_INPUT, MOTION, BEAT_GAP, BEAT_STALE,
                                   CONTEXT_GAP, TIMESTAMP, DATA_LOSS, QUEUE_OVERFLOW };
const char *resetReasonName(ResetReason reason);
struct ChannelDiagnostics {
    float candidate = NAN, quality = NAN, modulation = NAN; // SD / mean, not percent
    ChannelStatus status = ChannelStatus::NOT_ANALYZED;
};
struct Context {
    bool signalGood, lowMotion;
};
struct Result {
    uint32_t time = 0;
    float rate = NAN, riav = NAN, rifv = NAN;
    float quality = NAN; // Combined score only when valid; see per-channel diagnostics
    ChannelDiagnostics av, fv;
    float coverage = NAN; // Fraction of grid covered at the last analysis
    uint32_t windowMs = 0, beatAgeMs = UINT32_MAX; // One-time warmup; coverage remains rolling
    uint32_t lastBeatGapMs = 0; // Latest observed stale/missing-beat gap; not a reset
    uint32_t resets = 0, resetTime = 0, resetGapMs = 0;
    ResetReason lastReset = ResetReason::NONE;
    bool valid = false;
    Status status = Status::WARMUP;
};
class Observer {
public:
    void addBeat(uint32_t time, float amplitude, float ibi);
    void contactChanged(uint32_t now = 0);
    void dataLost(uint32_t now = 0, ResetReason reason = ResetReason::DATA_LOSS);
    void update(uint32_t now, const Context &context);
    const Result &result() const { return result_; }
private:
    struct Beat { uint32_t time; float amplitude, ibi; bool continuous; };
    Beat beats_[BEAT_CAPACITY] = {};
    unsigned head_ = 0, count_ = 0;
    Result result_;

bool haveUpdate_ = false, haveAnalysis_ = false;
uint32_t lastUpdate_ = 0, lastAnalysis_ = 0;
    // =====================================================
    // Stable PDR output
    // =====================================================

    float stableRate_ = NAN;
    float stableQuality_ = NAN;
    uint32_t lastValidRateMs_ = 0;

    // Hold the last valid PDR for 10 seconds
    static constexpr uint32_t PDR_HOLD_MS = 10000;

    // Exponential smoothing:
    // 75% previous value + 25% new value
    static constexpr float PDR_SMOOTH_ALPHA = 0.25f;


    bool windowStarted_ = false, acceptingBeats_ = true;
    bool breakPending_ = true, skipNextBeat_ = false;
    uint32_t windowStart_ = 0;
    // Scratch belongs to this observer, not the small Arduino loop stack.
    float amplitudeGrid_[GRID_SIZE], ibiGrid_[GRID_SIZE];
    bool gridValid_[GRID_SIZE], fvGridValid_[GRID_SIZE];
    void invalidate(Status reason);
    void clearHistory(Status reason, ResetReason detail, uint32_t now, uint32_t gapMs = 0);
    bool estimate(uint32_t now);
};
}
