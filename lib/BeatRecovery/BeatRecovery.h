#pragma once
#include <stdint.h>

namespace beats {
// Independent of Arduino; candidates are morphology-qualified optical crests.
// Candidate timestamps NEVER refresh the accepted-beat freshness clock.
class Recovery {
public:
    static constexpr unsigned REQUIRED = 4;
    static constexpr uint32_t STALE_MS = 3000;
    void reset() { *this = Recovery{}; }
    bool fresh(uint32_t now) const {
        return haveAccepted_ && now - acceptedMs_ < STALE_MS;
    }
    uint32_t acceptedAge(uint32_t now) const {
        return haveAccepted_ ? now - acceptedMs_ : UINT32_MAX;
    }
    // A reconstructed acquisition may seed HR history but cannot make it fresh.
    void seedRhythm(uint32_t peakMs) {
        haveRhythm_ = true;
        haveAccepted_ = false;
        acceptedMs_ = peakMs;
        count_ = 0;
    }
    void accept(uint32_t peakMs) {
        haveRhythm_ = haveAccepted_ = true;
        acceptedMs_ = peakMs;
        count_ = 0;
    }
    void clearCandidates() { count_ = 0; }
    uint32_t interval(unsigned i) const { return intervals_[i]; }
    bool observe(uint32_t peakMs) {
        if (!haveCandidate_) {
            haveCandidate_ = true;
            candidateMs_ = peakMs;
            return false;
        }
        uint32_t ibi = peakMs - candidateMs_;
        candidateMs_ = peakMs;
        if (ibi < 300 || ibi > 1800) { count_ = 0; return false; }
        if (count_ == REQUIRED) {
            for (unsigned i = 1; i < REQUIRED; ++i) intervals_[i - 1] = intervals_[i];
            --count_;
        }
        intervals_[count_++] = ibi;
        if (!haveRhythm_ || peakMs - acceptedMs_ < STALE_MS || count_ < REQUIRED) return false;
        uint32_t lo = intervals_[0], hi = lo;
        for (unsigned i = 1; i < REQUIRED; ++i) {
            if (intervals_[i] < lo) lo = intervals_[i];
            if (intervals_[i] > hi) hi = intervals_[i];
        }
        // Four consecutive intervals within 15%; no half/double reconstruction.
        return hi * 100UL <= lo * 115UL;
    }
private:
    uint32_t acceptedMs_ = 0, candidateMs_ = 0;
    uint32_t intervals_[REQUIRED] = {};
    unsigned count_ = 0;
    bool haveRhythm_ = false, haveAccepted_ = false, haveCandidate_ = false;
};
} // namespace beats
