#include "PdrObserver.h"
#include <math.h>
#include <algorithm>

namespace pdr {
namespace {
constexpr float PI = 3.14159265358979323846f;
struct Channel { float rate = 0, quality = 0; bool usable = false; };

// Detrend and fit sinusoids in the respiratory band. Explained weighted
// variance is a heuristic periodicity score, NOT a clinical confidence value.
Channel fit(float *x, const bool *valid, unsigned n, float maxRate, float minStd) {
    Channel out;
    float weights[GRID_SIZE];
    for (unsigned i = 0; i < n; ++i)
        weights[i] = 0.5f - 0.5f*cosf(2*PI*i/(n-1));
    float sum = 0, st = 0, stt = 0, sxt = 0;
    unsigned count = 0;
    for (unsigned i = 0; i < n; ++i) if (valid[i]) {
        const float t = (float)i / n - 0.5f;
        sum += x[i]; st += t; stt += t*t; sxt += x[i]*t; ++count;
    }
    if (count < 2 || sum <= 0) return out;
    const float mean = sum / count;
    const float denominator = stt - st*st/count;
    const float slope = denominator > 0 ? (sxt - sum*st/count)/denominator : 0;
    float energy = 0, weightSum = 0;
    for (unsigned i = 0; i < n; ++i) if (valid[i]) {
        x[i] = (x[i] - mean - slope*((float)i/n - 0.5f - st/count))/mean;
        const float w = weights[i];
        energy += w*x[i]*x[i]; weightSum += w;
    }
    if (weightSum <= 0 || energy / weightSum < minStd*minStd) return out;
    float best = 0;
    for (float rate = 4; rate <= maxRate; rate += 0.5f) {
        const float step = 2*PI*(rate/60.0f)/4.0f;
        const float dc = cosf(step), ds = sinf(step);
        float c = 1, s = 0, xc = 0, xs = 0, cc = 0, ss = 0, cs = 0;
        for (unsigned i = 0; i < n; ++i) {
            if (valid[i]) {
                const float w = weights[i];
                xc += w*x[i]*c; xs += w*x[i]*s;
                cc += w*c*c; ss += w*s*s; cs += w*c*s;
            }
            const float next = c*dc - s*ds;
            s = s*dc + c*ds; c = next;
        }
        const float det = cc*ss - cs*cs;
        if (det <= 1e-6f) continue;
        const float fitEnergy = (ss*xc*xc + cc*xs*xs - 2*cs*xc*xs)/det;
        const float quality = fminf(100, 100*fitEnergy/energy);
        if (quality > best) { best = quality; out.rate = rate; }
    }
    out.quality = best;
    out.usable = best >= MIN_QUALITY;
    return out;
}
}

const char *statusName(Status s) {
    switch (s) {
    case Status::WARMUP: return "WARMUP";
    case Status::SIGNAL: return "SIGNAL";
    case Status::MOTION: return "MOTION";
    case Status::GAP: return "GAP";
    case Status::WEAK: return "WEAK";
    case Status::DISAGREE: return "DISAGREE";
    case Status::VALID: return "VALID";
    default: return "STALE";
    }
}
const char *syncName(Sync s) {
    switch (s) {
    case Sync::NONE: return "NONE";
    case Sync::OBSERVING: return "OBSERVING";
    case Sync::RATE_MATCH: return "RATE_MATCH";
    case Sync::NO_RATE_MATCH: return "NO_RATE_MATCH";
    default: return "UNKNOWN";
    }
}

void Observer::invalidate(Status reason) {
    result_.valid = result_.support = false;
    result_.rate = result_.quality = result_.deviation = 0;
    result_.sources = 0;
    result_.supportMs = 0;
    result_.status = reason;
    baselineBlock_ = supportHold_ = false;
    rateCount_ = rateHead_ = 0;
    p2PreviousValid_ = p2PreviousMatch_ = false;
}
void Observer::clearHistory(Status reason) {
    count_ = head_ = 0;
    invalidate(reason);
}
void Observer::contactChanged() {
    clearHistory(Status::SIGNAL);
    if (p2Active_) p2LostData_ = true;
}
void Observer::dataLost() {
    clearHistory(Status::GAP);
    if (p2Active_) p2LostData_ = true;
}
void Observer::addBeat(uint32_t time, float amplitude, float ibi) {
    if (!isfinite(amplitude) || amplitude <= 0 || !isfinite(ibi) || ibi < 300 || ibi > 1800)
        return;
    if (count_) {
        const uint32_t delta = time - beats_[(head_ + BEAT_CAPACITY - 1) % BEAT_CAPACITY].time;
        if (!delta || delta > 0x7fffffffUL) { dataLost(); return; }
        if (delta > MAX_GAP_MS) clearHistory(Status::GAP);
    }
    beats_[head_] = {time, amplitude, ibi};
    head_ = (head_ + 1) % BEAT_CAPACITY;
    if (count_ < BEAT_CAPACITY) ++count_;
}
bool Observer::estimate(uint32_t now, uint32_t window) {
    if (count_ < 2) { invalidate(Status::WARMUP); return false; }
    const unsigned oldest = (head_ + BEAT_CAPACITY - count_) % BEAT_CAPACITY;
    if (now - beats_[oldest].time < window) { invalidate(Status::WARMUP); return false; }
    const Beat &last = beats_[(head_ + BEAT_CAPACITY - 1) % BEAT_CAPACITY];
    if (now - last.time > MAX_GAP_MS) { invalidate(Status::GAP); return false; }
    const unsigned n = window / 250;
    unsigned j = 0, covered = 0;
    float sumIbi = 0;
    for (unsigned i = 0; i < n; ++i) {
        const uint32_t age = window - i*250;
        while (j + 1 < count_ && now - beats_[(oldest+j+1)%BEAT_CAPACITY].time >= age) ++j;
        gridValid_[i] = false;
        if (j + 1 >= count_) continue; // never extrapolate beyond last beat
        const Beat &a = beats_[(oldest+j)%BEAT_CAPACITY];
        const Beat &b = beats_[(oldest+j+1)%BEAT_CAPACITY];
        if (now-a.time > window) continue; // exclude pre-intervention features
        const uint32_t gap = b.time - a.time;
        if (!gap || gap > MAX_GAP_MS || gap > 1.8f*fmaxf(a.ibi, b.ibi)) continue;
        const float fraction = (float)((now-a.time)-age)/gap;
        if (fraction < 0 || fraction > 1) continue;
        amplitudeGrid_[i] = a.amplitude + fraction*(b.amplitude-a.amplitude);
        ibiGrid_[i] = a.ibi + fraction*(b.ibi-a.ibi);
        sumIbi += ibiGrid_[i];
        gridValid_[i] = true; ++covered;
    }
    if (covered < n*0.8f) { invalidate(Status::GAP); return false; }
    // Resampling cannot create beat-rate information. Stay below beat Nyquist.
    const float maxRate = fminf(30, 0.45f*60000/(sumIbi/covered));
    const Channel av = fit(amplitudeGrid_, gridValid_, n, maxRate, 0.005f);
    const Channel fv = fit(ibiGrid_, gridValid_, n, maxRate, 0.003f);
    if (!av.usable && !fv.usable) {
        invalidate(Status::WEAK);
        result_.quality = fmaxf(av.quality, fv.quality);
        return false;
    }
    if (av.usable && fv.usable && fabsf(av.rate-fv.rate) > 2) {
        invalidate(Status::DISAGREE); return false;
    }
    result_.sources = (av.usable ? 1 : 0) | (fv.usable ? 2 : 0);
    result_.rate = av.usable && fv.usable ?
        (av.rate*av.quality + fv.rate*fv.quality)/(av.quality+fv.quality) :
        av.usable ? av.rate : fv.rate;
    result_.quality = av.usable && fv.usable ? (av.quality+fv.quality)*0.5f :
        fminf(75, av.usable ? av.quality : fv.quality);
    result_.valid = true;
    result_.status = Status::VALID;
    return true;
}
bool Observer::stableRate() {
    rateHistory_[rateHead_] = result_.rate;
    rateHead_ = (rateHead_+1)%10;
    if (rateCount_ < 10) ++rateCount_;
    if (rateCount_ < 10) return false;
    float low = rateHistory_[0], high = low, sum = 0;
    for (float r : rateHistory_) { low = fminf(low,r); high = fmaxf(high,r); sum += r; }
    return high-low <= fmaxf(1, 0.1f*sum/10);
}
void Observer::beginP2(uint32_t now) {
    p2Active_ = true; p2Start_ = now;
    p2GoodMs_ = p2MatchMs_ = 0;
    p2PreviousValid_ = p2PreviousMatch_ = p2LostData_ = false;
    result_.sync = Sync::OBSERVING;
    baselineBlock_ = supportHold_ = false;
    result_.support = false; result_.supportMs = 0;
}
void Observer::endP2(bool completed) {
    if (!p2Active_) return;
    result_.sync = !completed || p2LostData_ || p2GoodMs_ < 20000 ? Sync::UNKNOWN :
        p2MatchMs_ >= 0.8f*p2GoodMs_ ? Sync::RATE_MATCH : Sync::NO_RATE_MATCH;
    p2Active_ = false;
    p2PreviousValid_ = p2PreviousMatch_ = false;
}
void Observer::update(uint32_t now, const Context &context) {
    result_.time = now;
    if (haveUpdate_ && now-lastUpdate_ > MAX_GAP_MS) dataLost();
    haveUpdate_ = true; lastUpdate_ = now;
    if (!context.signalGood || !context.lowMotion) {
        clearHistory(context.lowMotion ? Status::SIGNAL : Status::MOTION);
        return;
    }
    if (!context.baselineAllowed || !context.restingAllowed || context.interventionActive)
        baselineBlock_ = false;
    if (!context.restingAllowed || context.interventionActive) {
        supportHold_ = false; result_.support = false; result_.supportMs = 0;
    }
    if (haveAnalysis_ && now-lastAnalysis_ < 1000) return;
    const uint32_t dt = haveAnalysis_ ? now-lastAnalysis_ : 0;
    haveAnalysis_ = true; lastAnalysis_ = now;
    const uint32_t p2Elapsed = now-p2Start_;
    const bool p2Window = p2Active_ && p2Elapsed >= 30000;
    const uint32_t window = p2Window && p2Elapsed < WINDOW_MS ? (p2Elapsed/250)*250 : WINDOW_MS;
    if (!estimate(now, window)) return;
    if (p2Window) {
        const bool match = fabsf(result_.rate-6) <= 1;
        if (p2PreviousValid_ && dt <= MAX_GAP_MS) {
            p2GoodMs_ += dt;
            if (match && p2PreviousMatch_) p2MatchMs_ += dt;
        }
        p2PreviousValid_ = true; p2PreviousMatch_ = match;
    }
    const bool stable = stableRate();
    if (!result_.baselineReady) {
        const bool collect = stable && context.baselineAllowed && context.restingAllowed &&
                             !context.interventionActive && !p2Active_;
        if (!collect) baselineBlock_ = false;
        else if (!baselineBlock_) { baselineBlock_ = true; baselineSince_ = now; }
        else if (now-baselineSince_ >= 5000) {
            baselineSamples_[baselineCount_++] = result_.rate;
            result_.baselineValidMs = baselineCount_*5000;
            baselineSince_ = now;
            if (baselineCount_ == BASELINE_SAMPLES) {
                std::sort(baselineSamples_, baselineSamples_+BASELINE_SAMPLES);
                result_.baseline = (baselineSamples_[17]+baselineSamples_[18])*0.5f;
                result_.baselineReady = true;
            }
        }
    }
    result_.deviation = result_.baselineReady ? 100*(result_.rate-result_.baseline)/result_.baseline : 0;
    const bool support = result_.baselineReady && context.restingAllowed &&
        !context.interventionActive && !p2Active_ && fabsf(result_.deviation) >= 20;
    if (!support) supportHold_ = false;
    else if (!supportHold_) { supportHold_ = true; supportSince_ = now; }
    result_.supportMs = supportHold_ ? now-supportSince_ : 0;
    result_.support = supportHold_ && result_.supportMs >= SUPPORT_MS;
}
}
