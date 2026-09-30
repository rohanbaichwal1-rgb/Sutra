#pragma once
#include <stdint.h>
#include <math.h>

namespace pdr {
// Passive observer of the EXISTING detector's threshold regions. It never
// selects/accepts beats. RIAV is the crest minus its preceding local trough,
// not the rectified crest relative to an arbitrary filter zero.
class PulseAmplitude {
public:
    void reset() { *this = PulseAmplitude{}; }
    void observe(float sample, uint32_t time, bool inRegion) {
        if (!isfinite(sample)) { reset(); return; }
        if (!inRegion) {
            if (wasInRegion_) haveTrough_ = false;
            if (!haveTrough_ || sample < trough_ || time-troughTime_ > 1800) {
                trough_ = sample; troughTime_ = time; haveTrough_ = true;
            }
        } else {
            if (!wasInRegion_) {
                regionTrough_ = haveTrough_ && time-troughTime_ <= 1800 ? trough_ : NAN;
                crest_ = -INFINITY;
                amplitude_ = NAN;
            }
            if (sample > crest_) {
                crest_ = sample;
                amplitude_ = isfinite(regionTrough_) && sample > regionTrough_ ?
                    sample-regionTrough_ : NAN;
            }
        }
        wasInRegion_ = inRegion;
    }
    float amplitude() const { return amplitude_; }
private:
    float trough_ = 0, regionTrough_ = NAN, crest_ = -INFINITY, amplitude_ = NAN;
    uint32_t troughTime_ = 0;
    bool haveTrough_ = false, wasInRegion_ = false;
};
}
