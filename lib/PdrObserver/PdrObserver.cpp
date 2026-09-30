#include "PdrObserver.h"
#include <math.h>

namespace pdr {
namespace {
constexpr float PI = 3.14159265358979323846f;
struct Channel {
    float rate = NAN, quality = NAN, modulation = NAN;
    bool usable = false;
    ChannelStatus status = ChannelStatus::NO_FIT;
    ChannelDiagnostics diagnostics() const {
        ChannelDiagnostics d;
        d.candidate = rate; d.quality = quality; d.modulation = modulation; d.status = status;
        return d;
    }
};

// Masked, Hann-weighted spectrum. Quality is respiratory-band concentration,
// not the fraction of ALL (including non-respiratory) variation fitted by one
// perfect sine. A Hann main lobe spans +/-2/windowHz; do not treat its adjacent
// samples as competing breaths. This is not a calibrated confidence value.
Channel fit(float *x, const bool *valid, unsigned n, float maxRate, float minStd) {
    Channel out;
    float weights[GRID_SIZE];
    for (unsigned i = 0; i < n; ++i)
        weights[i] = 0.5f - 0.5f*cosf(2*PI*i/(n-1));
    float sum = 0, st = 0, stt = 0, sxt = 0;
    float count = 0;
    for (unsigned i = 0; i < n; ++i) if (valid[i]) {
        const float t = (float)i / n - 0.5f;
        const float w = weights[i];
        sum += w*x[i]; st += w*t; stt += w*t*t; sxt += w*x[i]*t; count += w;
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
    if (weightSum <= 0) return out;
    if (energy <= 0) {
        out.modulation = 0;
        out.quality = 0;
        out.status = ChannelStatus::LOW_MODULATION;
        return out;
    }
    // Fit weak modulation too, for diagnostics only. The acceptance gate below
    // still requires the original modulation AND periodicity thresholds.
    constexpr unsigned bins = (unsigned)((MAX_RATE-MIN_RATE)/RATE_STEP)+1;
    float power[bins] = {};
    float best = 0, outsideBest = 0;
    unsigned bestBin = 0;
    const float bandMax = fminf(MAX_RATE,maxRate);
    // Search guard frequencies too: spectral leakage from a stronger rhythm
    // outside the permitted band must not be relabelled as 6 or 30 brpm.
    for (float rate = RATE_STEP; rate <= maxRate; rate += RATE_STEP) {
        const float step = 2*PI*(rate/60.0f)*(GRID_INTERVAL_MS/1000.0f);
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
        const float normalizedPower = fmaxf(0,fitEnergy/weightSum);
        if (rate < MIN_RATE || rate > bandMax) {
            outsideBest = fmaxf(outsideBest,normalizedPower);
            continue;
        }
        const unsigned bin = (unsigned)lroundf((rate-MIN_RATE)/RATE_STEP);
        power[bin] = normalizedPower;
        if (normalizedPower > best) {
            best = normalizedPower; out.rate = rate; bestBin = bin;
        }
    }
    float bandPower = 0, lobePower = 0;
    const float lobeWidth = 2*60000.0f/WINDOW_MS;
    for (unsigned i=0;i<bins;++i) {
        bandPower += power[i];
        if (fabsf(MIN_RATE+i*RATE_STEP-out.rate) <= lobeWidth) lobePower += power[i];
    }
    // Integral of the oversampled Hann spectrum, with its 1.5-bin equivalent
    // noise bandwidth removed, estimates band-limited normalized variance.
    out.modulation = sqrtf(bandPower*RATE_STEP*(WINDOW_MS/60000.0f)/1.5f);
    out.quality = bandPower > 0 ? lobePower/bandPower : 0;
    const bool enoughModulation = out.modulation >= minStd;
    // Refine between scan bins, without shifting a peak toward a band edge.
    if (bestBin > 0 && bestBin+1 < bins && power[bestBin-1]>0 && power[bestBin+1]>0) {
        const float left=logf(power[bestBin-1]), middle=logf(best), right=logf(power[bestBin+1]);
        const float curvature=left-2*middle+right;
        if (curvature < 0) {
            const float shift=fmaxf(-0.5f,fminf(0.5f,0.5f*(left-right)/curvature));
            out.rate += shift*RATE_STEP;
        }
    }
    out.usable = enoughModulation && out.quality >= MIN_QUALITY && best >= outsideBest;
    out.status = !enoughModulation ? ChannelStatus::LOW_MODULATION :
        !isfinite(out.rate) ? ChannelStatus::NO_FIT :
        !out.usable ? ChannelStatus::LOW_PERIODICITY : ChannelStatus::OK;
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
const char *channelStatusName(ChannelStatus s) {
    switch (s) {
    case ChannelStatus::LOW_MODULATION: return "LOW_MOD";
    case ChannelStatus::LOW_PERIODICITY: return "LOW_Q";
    case ChannelStatus::NO_FIT: return "NO_FIT";
    case ChannelStatus::OK: return "OK";
    default: return "NOT_ANALYZED";
    }
}
const char *resetReasonName(ResetReason r) {
    switch (r) {
    case ResetReason::CONTACT: return "CONTACT";
    case ResetReason::SIGNAL_INPUT: return "SIGNAL_INPUT";
    case ResetReason::MOTION: return "MOTION";
    case ResetReason::BEAT_GAP: return "BEAT_GAP";
    case ResetReason::BEAT_STALE: return "BEAT_STALE";
    case ResetReason::CONTEXT_GAP: return "CONTEXT_GAP";
    case ResetReason::TIMESTAMP: return "TIMESTAMP";
    case ResetReason::DATA_LOSS: return "DATA_LOSS";
    case ResetReason::QUEUE_OVERFLOW: return "QUEUE_OVERFLOW";
    default: return "NONE";
    }
}
void Observer::invalidate(Status reason) {
    result_.valid = false;
    result_.rate = result_.riav = result_.rifv = NAN;
    result_.quality = result_.coverage = NAN;
    result_.av = result_.fv = ChannelDiagnostics{};
    result_.status = reason;
}
void Observer::clearHistory(Status reason, ResetReason detail, uint32_t now, uint32_t gapMs) {
    // Count actual discarded windows, not repeated polling of an empty buffer.
    // Preserve this evidence through subsequent SIGNAL/WARMUP reports.
    if (count_) {
        ++result_.resets;
        result_.lastReset = detail;
        result_.resetTime = now;
        result_.resetGapMs = gapMs;
    }
    count_ = head_ = 0;

// A real history reset also invalidates the held PDR.
stableRate_ = NAN;
stableQuality_ = NAN;
lastValidRateMs_ = 0;

// Integrity/contact resets discard samples, never restart the boot's clock.
breakPending_ = skipNextBeat_ = true;
result_.beatAgeMs = UINT32_MAX;

invalidate(reason);
}
void Observer::contactChanged(uint32_t now) {
    clearHistory(Status::SIGNAL, ResetReason::CONTACT, now);
}
void Observer::dataLost(uint32_t now, ResetReason reason) {
    clearHistory(Status::GAP, reason, now);
}
void Observer::addBeat(uint32_t time, float amplitude, float ibi) {
    if (!acceptingBeats_) return;
    if (!isfinite(amplitude) || amplitude <= 0 || !isfinite(ibi) || ibi < 300 || ibi > 1800)
        return;
    // The first returning IBI may span the bad interval; do not use it.
    if (skipNextBeat_) { skipNextBeat_ = false; return; }
    if (count_) {
        const uint32_t delta = time - beats_[(head_ + BEAT_CAPACITY - 1) % BEAT_CAPACITY].time;
        if (!delta || delta > 0x7fffffffUL) {
            clearHistory(Status::GAP, ResetReason::TIMESTAMP, time, delta); return;
        }
        if (delta > MAX_GAP_MS) {
            // A known beat gap is missing data, not loss of the entire window.
            // estimate() already excludes this interval from interpolation and
            // requires MIN_COVERAGE. Never carry a valid result across the gap.
            result_.lastBeatGapMs = delta;
            invalidate(Status::GAP);
        }
    }
    if (!windowStarted_) { windowStarted_ = true; windowStart_ = time; }
    beats_[head_] = {time, amplitude, ibi, !breakPending_};
    breakPending_ = false;
    head_ = (head_ + 1) % BEAT_CAPACITY;
    if (count_ < BEAT_CAPACITY) ++count_;
}
bool Observer::estimate(uint32_t now) {

    // Clear only the CURRENT analysis outputs.
    // Do not clear stableRate_ here.
    result_.valid = false;
    result_.rate = NAN;
    result_.riav = NAN;
    result_.rifv = NAN;
    result_.quality = NAN;
    result_.coverage = NAN;
    result_.av = ChannelDiagnostics{};
    result_.fv = ChannelDiagnostics{};
    result_.status = Status::WARMUP;
    constexpr uint32_t window = WINDOW_MS;
    if (!windowStarted_ || result_.windowMs < window) return false;
    if (count_ < 2) { result_.status = Status::GAP; result_.coverage = 0; return false; }
    const unsigned oldest = (head_ + BEAT_CAPACITY - count_) % BEAT_CAPACITY;
    const Beat &last = beats_[(head_ + BEAT_CAPACITY - 1) % BEAT_CAPACITY];
    if (now - last.time > MAX_GAP_MS) { invalidate(Status::GAP); return false; }
    const unsigned n = GRID_SIZE;
    unsigned coverage[2] = {};
    float sumIbi = 0;
    for (unsigned channel=0;channel<2;++channel) {
        unsigned j=0;
        for (unsigned i=0;i<n;++i) {
            const uint32_t age = window-i*GRID_INTERVAL_MS;
            // An IBI is an average over [previous crest, current crest].
            // Place it at its midpoint, independently of the AV crest time.
            auto featureTime = [channel](const Beat &b) {
                return b.time-(channel ? (uint32_t)lroundf(b.ibi*0.5f) : 0);
            };
            while (j+1<count_ && now-featureTime(beats_[(oldest+j+1)%BEAT_CAPACITY])>=age) ++j;
            bool &valid = channel ? fvGridValid_[i] : gridValid_[i];
            valid=false;
            if (j+1>=count_) continue;
            const Beat &a=beats_[(oldest+j)%BEAT_CAPACITY], &b=beats_[(oldest+j+1)%BEAT_CAPACITY];
            if (!b.continuous) continue; // Never interpolate across a known pause.
            const uint32_t beatGap=b.time-a.time;
            if (!beatGap || beatGap>MAX_GAP_MS || beatGap>1.8f*fmaxf(a.ibi,b.ibi)) continue;
            const uint32_t aTime=featureTime(a), bTime=featureTime(b);
            const uint32_t gap=bTime-aTime;
            if (!gap || gap>MAX_GAP_MS || gap>0x7fffffffUL) continue;
            const uint32_t aAge=now-aTime;
            if (aAge<age || aAge-age>gap) continue; // no extrapolation or unsigned underflow
            const float fraction=(float)(aAge-age)/gap;
            if (channel) ibiGrid_[i]=a.ibi+fraction*(b.ibi-a.ibi);
            else {
                amplitudeGrid_[i]=a.amplitude+fraction*(b.amplitude-a.amplitude);
                sumIbi += a.ibi+fraction*(b.ibi-a.ibi);
            }
            valid=true; ++coverage[channel];
        }
    }
    const unsigned covered=coverage[0]<coverage[1] ? coverage[0] : coverage[1];
    result_.coverage=(float)covered/n;
    if (covered<n*MIN_COVERAGE) { result_.status=Status::GAP; return false; }
    // Include guard frequencies up to the conservative beat Nyquist limit.
    const float maxRate=fminf(60,0.45f*60000/(sumIbi/coverage[0]));
    const Channel av=fit(amplitudeGrid_,gridValid_,n,maxRate,RIAV_MIN_STD);
    const Channel fv=fit(ibiGrid_,fvGridValid_,n,maxRate,RIFV_MIN_STD);
    // ---------------------------------------------------------
// Save current RIAV / RIFV diagnostics
// ---------------------------------------------------------

result_.status = Status::WEAK;

result_.av = av.diagnostics();
result_.fv = fv.diagnostics();

result_.riav = av.usable ? av.rate : NAN;
result_.rifv = fv.usable ? fv.rate : NAN;


// ---------------------------------------------------------
// Calculate RAW PDR from RIAV / RIFV
// ---------------------------------------------------------

float rawRate = NAN;
float rawQuality = NAN;

if (av.usable && fv.usable)
{
    // Both channels are usable.
    // Only combine them when they agree.
    if (fabsf(av.rate - fv.rate) <= MAX_DISAGREEMENT)
    {
        const float totalQuality =
            av.quality + fv.quality;

        if (totalQuality > 0.0f)
        {
            rawRate =
                (av.rate * av.quality +
                 fv.rate * fv.quality)
                / totalQuality;

            rawQuality =
                0.5f *
                (av.quality + fv.quality);
        }
    }
    else
    {
        // Both channels are individually usable,
        // but they disagree with each other.
        result_.status = Status::DISAGREE;
    }
}
else if (av.usable)
{
    // RIAV alone is usable.
    rawRate = av.rate;
    rawQuality = av.quality;
}
else if (fv.usable)
{
    // RIFV alone is usable.
    rawRate = fv.rate;
    rawQuality = fv.quality;
}


// ---------------------------------------------------------
// NEW VALID PDR FOUND
// ---------------------------------------------------------

if (isfinite(rawRate))
{
    if (!isfinite(stableRate_))
    {
        // First valid PDR value.
        stableRate_ = rawRate;
    }
    else
    {
        // Smooth the new PDR.
        //
        // 75% previous value
        // 25% new value
        stableRate_ =
            ((1.0f - PDR_SMOOTH_ALPHA) * stableRate_) +
            (PDR_SMOOTH_ALPHA * rawRate);
    }

    stableQuality_ = rawQuality;
    lastValidRateMs_ = now;

    result_.rate = stableRate_;
    result_.quality = stableQuality_;

    result_.valid = true;
    result_.status = Status::VALID;

    return true;
}


// ---------------------------------------------------------
// CURRENT ANALYSIS FAILED
//
// Do NOT immediately throw away a recently valid PDR.
// Hold it for up to 10 seconds.
// ---------------------------------------------------------

if (isfinite(stableRate_) &&
    lastValidRateMs_ != 0 &&
    (uint32_t)(now - lastValidRateMs_) <= PDR_HOLD_MS)
{
    result_.rate = stableRate_;
    result_.quality = stableQuality_;

    result_.valid = true;
    result_.status = Status::VALID;

    return true;
}


// ---------------------------------------------------------
// No valid PDR for too long
// ---------------------------------------------------------

stableRate_ = NAN;
stableQuality_ = NAN;
lastValidRateMs_ = 0;

result_.rate = NAN;
result_.quality = NAN;
result_.valid = false;

// Keep WEAK or DISAGREE reason set above.
return false;
}
void Observer::update(uint32_t now, const Context &context) {
    result_.time = now;
    // Warm up once from the first usable beat, saturating even across millis wrap.
    if (windowStarted_ && result_.windowMs < WINDOW_MS) {
        const uint32_t elapsed = now-windowStart_;
        result_.windowMs = elapsed < WINDOW_MS ? elapsed : WINDOW_MS;
    }
    if (haveUpdate_ && now-lastUpdate_ > MAX_GAP_MS)
        clearHistory(Status::GAP, ResetReason::CONTEXT_GAP, now, now-lastUpdate_);
    if (!context.signalGood || !context.lowMotion) {
        if (acceptingBeats_) {
            // A beat event can arrive just before its bad-context notification.
            // Retract unconfirmed recent samples, retain the preceding good history.
            while (count_ && (!haveUpdate_ ||
                   now-beats_[(head_+BEAT_CAPACITY-1)%BEAT_CAPACITY].time <= now-lastUpdate_)) {
                head_ = (head_+BEAT_CAPACITY-1)%BEAT_CAPACITY;
                --count_;
            }
            breakPending_ = skipNextBeat_ = true;
        }
        acceptingBeats_ = false;
        haveUpdate_ = true; lastUpdate_ = now;
        result_.beatAgeMs = count_ ? now-beats_[(head_+BEAT_CAPACITY-1)%BEAT_CAPACITY].time : UINT32_MAX;
        invalidate(context.lowMotion ? Status::SIGNAL : Status::MOTION);
        return;
    }
    haveUpdate_ = true; lastUpdate_ = now;
    acceptingBeats_ = true;
    result_.beatAgeMs = count_ ? now-beats_[(head_+BEAT_CAPACITY-1)%BEAT_CAPACITY].time : UINT32_MAX;
    // Freshness still invalidates output immediately. Retain surrounding beats:
    // the 45 s rolling window and coverage gate prevent reuse of arbitrarily old
    // data, while the interpolation gap guard never fills the missing segment.
    if (count_ && now-beats_[(head_+BEAT_CAPACITY-1)%BEAT_CAPACITY].time > MAX_GAP_MS) {
        result_.lastBeatGapMs = result_.beatAgeMs;
        invalidate(Status::GAP);
        return;
    }
    if (windowStarted_ && breakPending_) {
        invalidate(result_.windowMs < WINDOW_MS ? Status::WARMUP : Status::GAP);
        result_.coverage = 0;
        return;
    }
    if (haveAnalysis_ && now-lastAnalysis_ < ANALYSIS_INTERVAL_MS) return;
    haveAnalysis_ = true; lastAnalysis_ = now;
    estimate(now);
}
}
