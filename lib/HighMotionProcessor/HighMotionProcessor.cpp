#include "HighMotionProcessor.h"
#include <math.h>
#include <algorithm>

namespace highmotion {
constexpr unsigned Processor::TAPS;
constexpr unsigned Processor::AXES;
constexpr unsigned Processor::IMU_CAPACITY;
constexpr unsigned Processor::PPG_CAPACITY;
constexpr unsigned Processor::IBI_CAPACITY;
namespace {
float clamp(float x, float lo, float hi) { return std::max(lo, std::min(x, hi)); }
float alpha(float dt, float seconds) { return dt / (std::max(seconds, 0.001f) + dt); }
bool after(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) >= 0; }
}
const char *reasonName(Reason r) {
    static const char *names[] = {"NONE", "INACTIVE", "WARMUP", "ALIGNMENT", "TIMING",
        "NUMERIC", "AMPLITUDE", "MORPHOLOGY", "MOTION_COUPLING", "INTERVAL",
        "ACQUIRING", "STALE", "QUEUE_FULL"};
    unsigned i = static_cast<unsigned>(r);
    return i < sizeof(names)/sizeof(names[0]) ? names[i] : "UNKNOWN";
}
Processor::Processor(const Config &config) : config_(config), periodUs_(config.initialSamplePeriodUs) {}
void Processor::resetSignal(uint32_t nowUs) {
    for (unsigned a = 0; a < AXES; ++a) {
        referenceDc_[a] = refPower_[a] = crossPower_[a] = 0;
        for (unsigned t = 0; t < TAPS; ++t) weights_[a][t] = refs_[a][t] = 0;
    }
    initialized_ = rising_ = haveBeat_ = false;
    haveSampleClock_ = false;
    ibiHead_ = ibiCount_ = 0;
    rawPower_ = cleanPower_ = predictedPower_ = dc_ = low_ = amplitude_ = hr_ = 0;
    pulsePower_ = peak_ = 0;
    started_ = nowUs;
    result_.ready = result_.valid = result_.aligned = result_.adapting = false;
    result_.hr = result_.ibiMs = result_.confidence = result_.coefficientNorm = 0;
}
void Processor::setActive(bool highMotion, bool contact, uint32_t nowUs) {
    // Keep the independent shadow filter warm across classifier transitions.
    // Contact loss still discards all physiological history immediately.
    if (contact != contact_) {
        resetSignal(nowUs);
        ppgCount_ = ppgHead_ = 0;
        contact_ = contact;
        haveSampleClock_ = false;
        result_.reason = contact ? Reason::WARMUP : Reason::INACTIVE;
    }
    result_.active = highMotion && contact;
}
void Processor::beginBatch(uint32_t receivedUs, unsigned count) {
    batchTime_ = receivedUs - config_.ppgLatencyUs;
    batchCount_ = count;
    batchIndex_ = 0;
    if (!rateStarted_) { rateStarted_ = true; rateStart_ = receivedUs; }
    uint32_t elapsed = receivedUs - rateStart_;
    // Count received samples independently, including outside HIGH motion.
    if (elapsed >= 2000000 && rateSamples_) {
        float measured = static_cast<float>(elapsed) / rateSamples_;
        if (measured >= 2500 && measured <= 20000) periodUs_ = measured;
        rateSamples_ = 0;
        rateStart_ = receivedUs;
    }
    rateSamples_ += count;
    batchReliable_ = count > 0 && count <= config_.maxEstimatedBatch &&
        isfinite(periodUs_) && periodUs_ >= 2500 && periodUs_ <= 20000;
    result_.samplePeriodUs = periodUs_;
}
void Processor::pushPpg(float raw) {
    unsigned index = batchIndex_++;
    if (!contact_ || index >= batchCount_) return;
    if (ppgCount_ == PPG_CAPACITY) {
        resetSignal(batchTime_);
        ppgCount_ = ppgHead_ = 0;
        reject(Reason::QUEUE_FULL);
        return;
    }
    // Anchor the newest FIFO sample to receipt; sensor latency is unknown.
    uint32_t time = batchTime_ - static_cast<uint32_t>((batchCount_ - index - 1) * periodUs_);
    bool reliable = batchReliable_;
    if (haveSampleClock_ && reliable) {
        uint32_t predicted = sampleClock_ + static_cast<uint32_t>(periodUs_);
        int32_t phaseError = static_cast<int32_t>(time - predicted);
        if (fabsf(static_cast<float>(phaseError)) <= config_.timestampJitterUs) {
            float correction = clamp(static_cast<float>(phaseError),
                -periodUs_ * config_.timestampCorrectionFraction,
                 periodUs_ * config_.timestampCorrectionFraction);
            time = predicted + static_cast<int32_t>(correction);
        } else reliable = false; // real discontinuity; do not bridge unknown data
    }
    sampleClock_ = time;
    haveSampleClock_ = reliable;
    ppg_[(ppgHead_ + ppgCount_) % PPG_CAPACITY] = {time, raw, reliable};
    ++ppgCount_;
}
void Processor::pushImu(uint32_t readMidpointUs, float ax, float ay, float az, bool valid) {
    Imu sample = {readMidpointUs - config_.imuLatencyUs, {ax, ay, az},
        valid && isfinite(ax) && isfinite(ay) && isfinite(az)};
    if (imuCount_) {
        const auto &last = imu_[(imuHead_ + imuCount_ - 1) % IMU_CAPACITY];
        if (!after(sample.time, last.time) || sample.time == last.time) return;
    }
    if (imuCount_ == IMU_CAPACITY) { imuHead_ = (imuHead_ + 1) % IMU_CAPACITY; --imuCount_; }
    imu_[(imuHead_ + imuCount_) % IMU_CAPACITY] = sample;
    ++imuCount_;
}
bool Processor::interpolate(uint32_t time, float *axis) const {
    for (unsigned i = 1; i < imuCount_; ++i) {
        const auto &left = imu_[(imuHead_ + i - 1) % IMU_CAPACITY];
        const auto &right = imu_[(imuHead_ + i) % IMU_CAPACITY];
        if (!after(time, left.time) || !after(right.time, time)) continue;
        uint32_t gap = right.time - left.time;
        if (!left.valid || !right.valid || !gap || gap > config_.maxImuGapUs) return false;
        float fraction = static_cast<float>(time - left.time) / gap;
        for (unsigned a = 0; a < AXES; ++a) axis[a] = left.axis[a] + fraction * (right.axis[a] - left.axis[a]);
        return true;
    }
    return false;
}
void Processor::reject(Reason reason, bool invalidate) {
    ++result_.rejected;
    if (invalidate || !result_.valid) result_.reason = reason;
    result_.lastRejection = reason;
    if (invalidate) result_.valid = false;
}
void Processor::poll(uint32_t nowUs) {
    while (ppgCount_) {
        const Ppg sample = ppg_[ppgHead_];
        float axis[AXES];
        bool aligned = interpolate(sample.time, axis);
        bool expired = after(nowUs, sample.time) && nowUs - sample.time > config_.alignmentWaitUs;
        if (!aligned && !expired && sample.reliable) break;
        ppgHead_ = (ppgHead_ + 1) % PPG_CAPACITY;
        --ppgCount_;
        if (!aligned || !sample.reliable) {
            resetSignal(sample.time);
            ++result_.alignmentDrops;
            reject(sample.reliable ? Reason::ALIGNMENT : Reason::TIMING);
        } else process(sample, axis);
    }
    if (contact_ && haveBeat_ && nowUs - lastBeat_ > config_.staleUs) {
        // Losing rhythm confidence is not a numerical/filter failure.
        // Reacquire measured beats without repeatedly restarting filter warmup.
        haveBeat_ = false;
        ibiCount_ = ibiHead_ = 0;
        amplitude_ = hr_ = 0;
        result_.valid = result_.ready = false;
        result_.hr = result_.ibiMs = result_.confidence = 0;
        result_.reason = Reason::STALE;
    }
}
void Processor::process(const Ppg &sample, const float *axis) {
    if (!isfinite(sample.raw)) { resetSignal(sample.time); reject(Reason::NUMERIC); return; }
    if (initialized_ && (!after(sample.time, lastSample_) || sample.time == lastSample_ ||
                        sample.time - lastSample_ > config_.maxPpgGapUs)) {
        resetSignal(sample.time);
        reject(Reason::TIMING);
        return;
    }
    if (!initialized_) {
        dc_ = sample.raw;
        for (unsigned a = 0; a < AXES; ++a) referenceDc_[a] = axis[a];
        lastSample_ = previousTime_ = troughTime_ = sample.time;
        previous_ = trough_ = 0;
        initialized_ = true;
        result_.aligned = true;
        return;
    }
    float dt = (sample.time - lastSample_) * 1e-6f;
    lastSample_ = sample.time;
    dc_ += alpha(dt, config_.ppgDcSeconds) * (sample.raw - dc_);
    float raw = sample.raw - dc_, predicted = 0, energy = 0;
    for (unsigned a = 0; a < AXES; ++a) {
        referenceDc_[a] += alpha(dt, config_.referenceDcSeconds) * (axis[a] - referenceDc_[a]);
        for (unsigned t = TAPS - 1; t > 0; --t) refs_[a][t] = refs_[a][t-1];
        refs_[a][0] = (axis[a] - referenceDc_[a]) * config_.referenceScale;
        for (unsigned t = 0; t < TAPS; ++t) {
            energy += refs_[a][t] * refs_[a][t];
            predicted += weights_[a][t] * refs_[a][t];
        }
    }
    float clean = raw - predicted;
    if (!isfinite(clean) || !isfinite(energy)) { resetSignal(sample.time); reject(Reason::NUMERIC); return; }
    result_.adapting = energy > config_.motionEnergyFloor;
    float norm = 0;
    for (unsigned a = 0; a < AXES; ++a) for (unsigned t = 0; t < TAPS; ++t) {
        float step = result_.adapting ? config_.learningRate * clamp(clean, -config_.maxAdaptError,
            config_.maxAdaptError) * refs_[a][t] / std::max(config_.normalization + energy, 0.001f) : 0;
        weights_[a][t] = clamp(weights_[a][t] * std::max(0.0f, 1 - config_.leakagePerSecond * dt) +
            clamp(step, -config_.maxCoefficientStep, config_.maxCoefficientStep),
            -config_.coefficientBound, config_.coefficientBound);
        norm += weights_[a][t] * weights_[a][t];
    }
    result_.coefficientNorm = sqrtf(norm);
    float aPower = alpha(dt, 1.0f);
    rawPower_ += aPower * (raw * raw - rawPower_);
    cleanPower_ += aPower * (clean * clean - cleanPower_);
    predictedPower_ += aPower * (predicted * predicted - predictedPower_);
    for (unsigned a = 0; a < AXES; ++a) {
        refPower_[a] += aPower * (refs_[a][0] * refs_[a][0] - refPower_[a]);
        crossPower_[a] += aPower * (clean * refs_[a][0] - crossPower_[a]);
    }
    low_ += alpha(dt, config_.lowPassSeconds) * (-clean - low_);
    pulsePower_ += aPower * (low_ * low_ - pulsePower_);
    if (!isfinite(low_) || !isfinite(cleanPower_) || !isfinite(norm)) {
        resetSignal(sample.time); reject(Reason::NUMERIC); return;
    }
    result_.aligned = true;
    if (sample.time - started_ < config_.warmupUs) {
        previous_ = trough_ = low_;
        previousTime_ = troughTime_ = sample.time;
        result_.reason = Reason::WARMUP;
        return;
    }
    if (rising_) {
        if (sample.time - troughTime_ > config_.maxRiseUs) {
            rising_ = false;
            reject(Reason::MORPHOLOGY, false);
        }
    }
    if (rising_) {
        if (low_ > peak_) { peak_ = low_; peakTime_ = sample.time; }
        float drop = std::max(config_.peakDropAmplitudeFraction * (peak_ - trough_),
                             config_.peakDropRmsFraction * sqrtf(pulsePower_));
        if (peak_ - low_ >= std::max(0.001f, drop)) {
            candidate(peakTime_, peak_ - trough_, peakTime_ - troughTime_);
            rising_ = false;
        }
    } else if (low_ > previous_) {
        float slopeFloor = std::max(config_.minRiseSlope, config_.noiseSlopeMultiplier * sqrtf(pulsePower_));
        if (!rising_ && (low_ - previous_) / dt > slopeFloor) {
            trough_ = previous_; troughTime_ = previousTime_; rising_ = true;
            peak_ = low_; peakTime_ = sample.time;
        }
    }
    previous_ = low_;
    previousTime_ = sample.time;
}
float Processor::medianIbi() const {
    float sorted[IBI_CAPACITY];
    for (unsigned i = 0; i < ibiCount_; ++i) sorted[i] = ibis_[i];
    std::sort(sorted, sorted + ibiCount_);
    return ibiCount_ ? sorted[ibiCount_ / 2] : 0;
}
void Processor::candidate(uint32_t time, float amplitude, uint32_t riseUs) {
    result_.candidateAmplitude = amplitude;
    result_.candidateFloor = std::max(config_.minPulseAmplitude, config_.noiseMultiplier * sqrtf(pulsePower_));
    result_.candidateRiseMs = riseUs * 0.001f;
    float correlation = 0;
    for (unsigned a = 0; a < AXES; ++a)
        correlation = std::max(correlation, fabsf(crossPower_[a]) / sqrtf(std::max(1.0f, cleanPower_ * refPower_[a])));
    float cancellation = sqrtf(predictedPower_ / std::max(1.0f, rawPower_));
    result_.residualCorrelation = correlation;
    result_.cancellationRatio = cancellation;
    if (riseUs < config_.minRiseUs || riseUs > config_.maxRiseUs) { reject(Reason::MORPHOLOGY, false); return; }
    if (amplitude < result_.candidateFloor ||
        (amplitude_ > 0 && (amplitude < amplitude_ * config_.minAmplitudeRatio ||
                           amplitude > amplitude_ * config_.maxAmplitudeRatio))) {
        reject(Reason::AMPLITUDE, false); return;
    }
    if (correlation > config_.maxResidualCorrelation || cancellation > config_.maxCancellationRatio) {
        reject(Reason::MOTION_COUPLING); return;
    }
    float confidence = clamp(1 - correlation, 0, 1) * clamp(1 - 0.5f * cancellation, 0, 1);
    if (!haveBeat_) { lastBeat_ = time; haveBeat_ = true; result_.reason = Reason::ACQUIRING; return; }
    uint32_t interval = time - lastBeat_;
    if (interval < config_.minIbiUs) { reject(Reason::INTERVAL, false); return; }
    float median = medianIbi();
    if (interval > config_.maxIbiUs || (median > 0 && fabsf(interval - median) > config_.intervalTolerance * median)) {
        // Start a new acquisition chain; never split or reconstruct a missed interval.
        ibiCount_ = ibiHead_ = 0;
        result_.ready = false;
        lastBeat_ = time;
        reject(Reason::INTERVAL);
        return;
    }
    lastBeat_ = time;
    ibis_[ibiHead_] = static_cast<float>(interval);
    ibiHead_ = (ibiHead_ + 1) % IBI_CAPACITY;
    if (ibiCount_ < IBI_CAPACITY) ++ibiCount_;
    ++result_.accepted;
    amplitude_ = amplitude_ > 0 ? 0.85f * amplitude_ + 0.15f * amplitude : amplitude;
    result_.ready = ibiCount_ >= std::max(2u, std::min(IBI_CAPACITY, static_cast<unsigned>(config_.acquisitionIntervals)));
    float target = 60000000.0f / medianIbi();
    hr_ = hr_ > 0 ? hr_ + alpha(interval * 1e-6f, config_.hrSmoothingSeconds) * (target - hr_) : target;
    result_.hr = hr_;
    result_.ibiMs = interval * 0.001f;
    result_.confidence = confidence;
    result_.valid = result_.ready && confidence >= config_.minConfidence;
    result_.reason = result_.valid ? Reason::NONE : Reason::ACQUIRING;
}
Result Processor::read(uint32_t nowUs) const {
    Result out = result_;
    if (!out.active) { out.valid = false; out.reason = Reason::INACTIVE; }
    if (out.active && haveBeat_ && nowUs - lastBeat_ > config_.staleUs) {
        out.valid = out.ready = false;
        out.reason = Reason::STALE;
    }
    if (!out.valid) out.hr = out.ibiMs = NAN;
    return out;
}
float Processor::selectHr(float original, bool highMotion, uint32_t nowUs) const {
#if HIGH_MOTION_OUTPUT_OVERRIDE
    Result out = read(nowUs);
    if (highMotion) return out.valid ? out.hr : NAN;
#else
    (void)highMotion; (void)nowUs;
#endif
    return original;
}
void Processor::recordProcessingTime(uint32_t elapsedUs) {
    result_.maxProcessingUs = std::max(result_.maxProcessingUs, elapsedUs);
}
}
