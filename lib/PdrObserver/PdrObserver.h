#pragma once
#include <stdint.h>

// PPG respiration context only. This module has no trigger or actuator interface.
namespace pdr {
constexpr unsigned BEAT_CAPACITY = 256;
constexpr unsigned GRID_SIZE = 240;
constexpr uint32_t WINDOW_MS = 60000;
constexpr uint32_t MAX_GAP_MS = 2500;
constexpr uint32_t SUPPORT_MS = 20000;
constexpr unsigned BASELINE_SAMPLES = 36; // complete 5-second blocks = 180 s
constexpr float MIN_QUALITY = 70;
enum class Status : uint8_t { WARMUP, SIGNAL, MOTION, GAP, WEAK, DISAGREE, VALID, STALE };
enum class Sync : uint8_t { NONE, OBSERVING, UNKNOWN, RATE_MATCH, NO_RATE_MATCH };
const char *statusName(Status status);
const char *syncName(Sync sync);
struct Context {
    bool signalGood, lowMotion, restingAllowed, baselineAllowed, interventionActive;
};
struct Result {
    uint32_t time = 0, baselineValidMs = 0, supportMs = 0;
    float rate = 0, quality = 0, baseline = 0, deviation = 0;
    bool valid = false, baselineReady = false, support = false;
    uint8_t sources = 0; // bit 0 RIAV, bit 1 RIFV
    Status status = Status::WARMUP;
    Sync sync = Sync::NONE;
};
class Observer {
public:
    void addBeat(uint32_t time, float amplitude, float ibi);
    void contactChanged();
    void dataLost();
    void beginP2(uint32_t now);
    void endP2(bool completed);
    void update(uint32_t now, const Context &context);
    const Result &result() const { return result_; }
private:
    struct Beat { uint32_t time; float amplitude, ibi; };
    Beat beats_[BEAT_CAPACITY] = {};
    unsigned head_ = 0, count_ = 0;
    Result result_;
    float baselineSamples_[BASELINE_SAMPLES] = {};
    unsigned baselineCount_ = 0;
    float rateHistory_[10] = {};
    unsigned rateCount_ = 0, rateHead_ = 0;
    bool baselineBlock_ = false, supportHold_ = false;
    uint32_t baselineSince_ = 0, supportSince_ = 0;
    bool haveUpdate_ = false, haveAnalysis_ = false;
    uint32_t lastUpdate_ = 0, lastAnalysis_ = 0;
    bool p2Active_ = false, p2PreviousValid_ = false, p2PreviousMatch_ = false;
    bool p2LostData_ = false;
    uint32_t p2Start_ = 0, p2GoodMs_ = 0, p2MatchMs_ = 0;
    // Scratch belongs to this observer, not the small Arduino loop stack.
    float amplitudeGrid_[GRID_SIZE], ibiGrid_[GRID_SIZE];
    bool gridValid_[GRID_SIZE];
    void invalidate(Status reason);
    void clearHistory(Status reason);
    bool estimate(uint32_t now, uint32_t window);
    bool stableRate();
};
}
