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
    initialized_ = haveBeat_ = false;
    haveSampleClock_ = false;
    ibiHead_ = ibiCount_ = 0;
    rawPower_ = cleanPower_ = predictedPower_ = dc_ = cleanDc_ = low_ = amplitude_ = 0;
    beatHr_ = trackedIbiUs_ = 0;
    pulsePower_ = peak_ = 0;
    for (unsigned i = 0; i < W1; ++i) w1Buffer_[i] = 0.0f;
    for (unsigned i = 0; i < W2; ++i) w2Buffer_[i] = 0.0f;
    w1Index_ = w2Index_ = 0;
    w1Sum_ = w2Sum_ = zMean_ = 0.0f;
    pulseRegionActive_ = false;
    pulseRegionStartUs_ = 0;
    pulseRegionLastAboveUs_ = 0;
    regionArea_ = regionMomentUs_ = 0;
    lastEstimateUs_ = 0;
    started_ = nowUs;
    result_.ready = result_.valid = result_.aligned = result_.adapting = false;
    result_.hr = result_.ibiMs = result_.confidence = result_.coefficientNorm = 0;
    result_.rawIbiMs = result_.correctedIbiMs = result_.trackedIbiMs =
        result_.beatAgeMs = 0;
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
    // The FIFO reception timestamp is still useful for oversized batches.
    // Do not discard an entire processing history merely because a batch
    // contains more samples than the preferred batch size.
    batchReliable_ = count > 0 && isfinite(periodUs_) &&
        periodUs_ >= 2500 && periodUs_ <= 20000;
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
        } else {
            // A delayed FIFO receipt can shift the timestamp by >20ms.
            // Keep real elapsed time where it advances, but do not send the
            // following samples backwards. Recovery happens in process().
            if (!after(time, predicted)) time = predicted;
        }
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

    switch (reason) {
        case Reason::AMPLITUDE:
            ++result_.amplitudeRejects;
            break;

        case Reason::MORPHOLOGY:
            ++result_.morphologyRejects;
            break;

        case Reason::INTERVAL:
            ++result_.intervalRejects;
            break;

        case Reason::MOTION_COUPLING:
            ++result_.motionRejects;
            break;

        case Reason::TIMING:
        case Reason::ALIGNMENT:
            ++result_.timingRejects;
            break;

        default:
            break;
    }

    if (invalidate || !result_.valid)
        result_.reason = reason;

    result_.lastRejection = reason;

    if (invalidate)
        result_.valid = false;
}
void Processor::poll(uint32_t nowUs) {
    while (ppgCount_) {
        const Ppg sample = ppg_[ppgHead_];
        float axis[AXES];
        const bool aligned = sample.reliable && interpolate(sample.time, axis);
        const bool expired = after(nowUs, sample.time) &&
                             nowUs - sample.time > config_.alignmentWaitUs;
        if (!aligned && !expired && sample.reliable) break;
        ppgHead_ = (ppgHead_ + 1) % PPG_CAPACITY;
        --ppgCount_;
        if (!aligned) {
            // Use the nearest available valid accelerometer sample if no
            // bracketing pair arrived in time. Sample-and-hold is less exact
            // than interpolation, but avoids repeatedly destroying an
            // established beat history during ordinary FIFO/IMU jitter.
            bool found = false;
            uint32_t closestDelta = 0xFFFFFFFFu;
            for (unsigned i = 0; i < imuCount_; ++i) {
                const Imu &imu = imu_[(imuHead_ + i) % IMU_CAPACITY];
                if (!imu.valid) continue;
                const uint32_t delta = after(sample.time, imu.time)
                    ? sample.time - imu.time : imu.time - sample.time;
                if (!found || delta < closestDelta) {
                    closestDelta = delta;
                    for (unsigned a = 0; a < AXES; ++a) axis[a] = imu.axis[a];
                    found = true;
                }
            }
            if (!found) {
                // No usable motion reference: subtract no new artifact.
                // Keep PPG and heartbeat timing alive until IMU resumes.
                for (unsigned a = 0; a < AXES; ++a) axis[a] = referenceDc_[a];
            }
        }
        process(sample, axis);
        result_.aligned = aligned;
    }
    // Display the latest beat-derived HR between pulses, with the existing 10s limit.
    if (contact_ && result_.valid &&
        nowUs - lastEstimateUs_ > config_.staleUs) {
        result_.valid = result_.ready = false;
        result_.hr = result_.ibiMs = result_.confidence = 0;
        // Do not show the old value, but preserve rhythm as a reacquisition
        // prior. A single short crest after a gap must not seed 158 BPM.
        result_.reason = Reason::STALE;
    }
    // If no qualified candidate has appeared for a long time, restart just
    // the beat-acquisition history; do not discard the adapted NLMS filter.
    if (contact_ && haveBeat_ && nowUs - lastBeat_ > config_.staleUs) {
        haveBeat_ = false;
        // Keep the previously established median for the next acquisition.
        // Only a physical contact change resets that history.
        if (!result_.valid) result_.reason = Reason::STALE;
    }
}
void Processor::process(const Ppg &sample, const float *axis) {
    if (!isfinite(sample.raw)) { resetSignal(sample.time); reject(Reason::NUMERIC); return; }
    if (initialized_ && (!after(sample.time, lastSample_) || sample.time == lastSample_)) {
        // Duplicate/out-of-order timestamps are not new optical information.
        // Ignore that sample without erasing the established heartbeat.
        return;
    }
    if (initialized_ && sample.time - lastSample_ > config_.maxPpgGapUs) {
        // A FIFO delay is a gap, not proof that every past heartbeat and
        // every adapted NLMS coefficient was wrong. Restart only an open
        // pulse region; keep the running rhythm and adaptive filter.
        pulseRegionActive_ = false;
    }
    if (!initialized_) {
        dc_ = sample.raw;
        for (unsigned a = 0; a < AXES; ++a) referenceDc_[a] = axis[a];
        lastSample_ = sample.time;
        initialized_ = true;
        result_.aligned = true;
        return;
    }
    // Bound the IIR update for a delayed sample; the actual timestamp is
    // retained for IBI calculation rather than compressing the time axis.
    const float dt = std::min(0.05f, (sample.time - lastSample_) * 1e-6f);
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
    // Permit full-size motion-artifact removal; the old 0.8 x RMS cap
    // routinely left most large movement transients in the optical signal.
    const float predictionLimit = config_.predictionRmsLimit *
        sqrtf(std::max(1.0f, std::max(rawPower_, 0.1f * raw * raw)));
    predicted = clamp(predicted, -predictionLimit, predictionLimit);
    float clean = raw - predicted;
    if (!isfinite(clean) || !isfinite(energy)) { resetSignal(sample.time); reject(Reason::NUMERIC); return; }
    result_.adapting = energy > config_.motionEnergyFloor;
    float norm = 0;
    // Normalize cancellation adaptation to elapsed sample time.
    const float rateScale = clamp(dt / 0.010f, 0.25f, 1.5f);
    for (unsigned a = 0; a < AXES; ++a) for (unsigned t = 0; t < TAPS; ++t) {
        float step = result_.adapting ? rateScale * config_.learningRate *
            clamp(clean, -config_.maxAdaptError, config_.maxAdaptError) *
            refs_[a][t] / std::max(config_.normalization + energy, 0.001f) : 0;
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
    // Two-stage optical baseline removal (PPG DC and post-NLMS high-pass)
    // followed by low-pass smoothing (~3.5 Hz at default settings).
    // Attenuates slow strap shifts and high-frequency spikes before timing.
    cleanDc_ += alpha(dt, config_.residualDcSeconds) * (clean - cleanDc_);
    const float opticalPulse = -(clean - cleanDc_);
    low_ += alpha(dt, config_.lowPassSeconds) * (opticalPulse - low_);
    pulsePower_ += aPower * (low_ * low_ - pulsePower_);
    if (!isfinite(low_) || !isfinite(cleanPower_) || !isfinite(norm)) {
        resetSignal(sample.time); reject(Reason::NUMERIC); return;
    }
    result_.aligned = true;

    // Independent Elgendi-style pulse-region detector at the observed
    // ~100 Hz PPG processing rate; this does NOT touch original LMS state.
    const float positivePulse = std::max(0.0f, low_); // already inverted
    const float z = positivePulse * positivePulse;
    w1Sum_ += z - w1Buffer_[w1Index_];
    w1Buffer_[w1Index_] = z;
    w1Index_ = (w1Index_ + 1) % W1;
    w2Sum_ += z - w2Buffer_[w2Index_];
    w2Buffer_[w2Index_] = z;
    w2Index_ = (w2Index_ + 1) % W2;
    zMean_ += 0.001f * (z - zMean_);

    // Fill FIR windows during warmup, but do not emit partial pulse regions.
    if (sample.time - started_ < config_.warmupUs) {
        pulseRegionActive_ = false;
        result_.reason = Reason::WARMUP;
        return;
    }

    const bool aboveThreshold =
        (w1Sum_ / static_cast<float>(W1)) >
        (w2Sum_ / static_cast<float>(W2) + config_.envelopeOffset * zMean_);

    // Keep Elgendi's adjacent-crest segmentation, but do not discard a
    // whole optical section when walking movement keeps the envelope high.
    // End a region either after a sustained envelope dip or after roughly
    // one predicted beat period. The latter provides regular resynchronization.
    if (aboveThreshold) {
        if (!pulseRegionActive_) {
            pulseRegionActive_ = true;
            pulseRegionStartUs_ = sample.time;
            pulseRegionLastAboveUs_ = sample.time;
            peak_ = positivePulse;
            peakTime_ = sample.time;
            regionArea_ = regionMomentUs_ = 0.0f;
        }
        pulseRegionLastAboveUs_ = sample.time;
        regionArea_ += positivePulse;
        regionMomentUs_ += positivePulse *
            static_cast<float>(sample.time - pulseRegionStartUs_);
        if (positivePulse > peak_) {
            peak_ = positivePulse;
            peakTime_ = sample.time;
        }
    }

    if (pulseRegionActive_) {
        const bool closedByEnvelope = !aboveThreshold &&
            sample.time - pulseRegionLastAboveUs_ >= 85000UL;
        // A coherent pulse can finish even without the envelope crossing
        // fully below its background (common during sustained movement).
        const float period = trackedIbiUs_ > 0.0f
    ? trackedIbiUs_ : 700000.0f;

const uint32_t regionLimit = static_cast<uint32_t>(
    clamp(0.70f * period, 300000.0f, 850000.0f));
        const bool closedByTime = sample.time - pulseRegionStartUs_ >= regionLimit;
        if (closedByEnvelope || closedByTime) {
            pulseRegionActive_ = false;
            const uint32_t widthUs = pulseRegionLastAboveUs_ - pulseRegionStartUs_;
            if (widthUs >= 20000UL && peak_ >= config_.minPulseAmplitude) {
                // Favor the true crest over a motion-skewed region centroid.
                const float peakOffset = static_cast<float>(peakTime_ - pulseRegionStartUs_);
                const float centroidOffset = regionArea_ > 0.0f
                    ? regionMomentUs_ / regionArea_ : peakOffset;
                const uint32_t beatTime = pulseRegionStartUs_ + static_cast<uint32_t>(
                    0.80f * peakOffset + 0.20f * centroidOffset + 0.5f);
                candidate(beatTime, peak_, beatTime - pulseRegionStartUs_);
            }
        }
    }
}
float Processor::medianIbi() const {
    float sorted[IBI_CAPACITY];
    for (unsigned i = 0; i < ibiCount_; ++i) sorted[i] = ibis_[i];
    std::sort(sorted, sorted + ibiCount_);
    if (!ibiCount_) return 0.0f;
    if (ibiCount_ % 2u) return sorted[ibiCount_ / 2u];
    return 0.5f * (sorted[ibiCount_ / 2u - 1u] + sorted[ibiCount_ / 2u]);
}
float Processor::recentMedianIbi(float *relativeRange) const {
    // Prefer the newest three corrected intervals when HR changes rapidly.
    // The median suppresses a single premature optical crest without any
    // extra rejection gate or withholding the displayed HR.
    const unsigned count = std::min(3u, ibiCount_);
    if (!count) return 0.0f;
    float recent[3];
    for (unsigned i = 0; i < count; ++i) {
        const unsigned index = (ibiHead_ + IBI_CAPACITY - 1u - i) % IBI_CAPACITY;
        recent[i] = ibis_[index];
    }
    std::sort(recent, recent + count);
    const float median = (count % 2u) ? recent[count / 2u]
        : 0.5f * (recent[count / 2u - 1u] + recent[count / 2u]);
    if (relativeRange) *relativeRange = (recent[count - 1u] - recent[0]) /
        std::max(median, 1.0f);
    return median;
}
void Processor::candidate(uint32_t time, float amplitude, uint32_t riseUs) {
    result_.candidateAmplitude = amplitude;
    result_.candidateRiseMs = riseUs * 0.001f;
    result_.candidateFloor = config_.minPulseAmplitude;

    float correlation = 0;
    for (unsigned a = 0; a < AXES; ++a)
        correlation = std::max(correlation, fabsf(crossPower_[a]) /
            sqrtf(std::max(1.0f, cleanPower_ * refPower_[a])));
    const float cancellation = sqrtf(predictedPower_ / std::max(1.0f, rawPower_));
    result_.residualCorrelation = correlation;
    result_.cancellationRatio = cancellation;
    // For this readings-first test: don't reject weak shape or motion coupling.
    // The region must still have some pulse amplitude to generate a candidate.
    if (amplitude < config_.minPulseAmplitude) {
        reject(Reason::AMPLITUDE, false);
        return;
    }

    const float confidence = clamp(1.0f - correlation, 0.0f, 1.0f) *
                             clamp(1.0f - 0.5f * cancellation, 0.0f, 1.0f);
    if (!haveBeat_) {
        lastBeat_ = time;
        haveBeat_ = true;
        if (!result_.valid) result_.reason = Reason::ACQUIRING;
        return;
    }
    const uint32_t interval = time - lastBeat_;
    result_.rawIbiMs = interval * 0.001f;
    if (interval < config_.minIbiUs) {
        // Preserve the existing physical refractory period. A closer crest
        // does not move the timing anchor or affect HR.
        reject(Reason::INTERVAL, false);
        return;
    }

    // Critical repeatability fix: advance on EVERY physically plausible
    // Elgendi crest. The previous 0.72 * median rejection could lock onto
    // every other real heartbeat after a run of missed detections.
    lastBeat_ = time;
    const float previousIbi = medianIbi();
    float rateInterval = static_cast<float>(interval);

    // If a motion-distorted signal skips an optical beat, divide the
    // measured interval by the closest integer number of beat periods.
    // The previous code blended 75% of the old IBI into each reconstruction,
    // which could retain a wrong (often half-rate) cadence for many seconds.
    const float rhythmPrior = trackedIbiUs_ > 0.0f ? trackedIbiUs_ : previousIbi;
    if (rhythmPrior > 0.0f && interval > 1.55f * rhythmPrior) {
        const unsigned periods = std::min(6u, std::max(2u,
            static_cast<unsigned>(static_cast<float>(interval) / rhythmPrior + 0.5f)));
        rateInterval = static_cast<float>(interval) / static_cast<float>(periods);
    } else if (interval > config_.maxIbiUs && rhythmPrior <= 0.0f) {
        reject(Reason::INTERVAL, false);
        return;
    }

    result_.correctedIbiMs = rateInterval * 0.001f;
    ibis_[ibiHead_] = rateInterval;
    ibiHead_ = (ibiHead_ + 1u) % IBI_CAPACITY;
    if (ibiCount_ < IBI_CAPACITY) ++ibiCount_;
    const float longMedianIbi = medianIbi();
    const float shortMedianIbi = recentMedianIbi();
    // Current beat plus the recent three and five beat windows. The latest
    // beat affects the tracker without forcing a sudden output jump.
    // This also follows real exercise transitions faster than the old
    // stability-weighted median, which often stuck at its old cadence.
    const float rateIbi = 0.25f * rateInterval + 0.55f * shortMedianIbi +
                          0.20f * longMedianIbi;
    const float elapsedSeconds = std::min(1.2f, rateInterval * 1e-6f);
    const float smooth = alpha(elapsedSeconds, config_.hrSmoothingSeconds);
    trackedIbiUs_ = trackedIbiUs_ > 0.0f
        ? trackedIbiUs_ + smooth * (rateIbi - trackedIbiUs_)
        : rateIbi;
    beatHr_ = 60000000.0f / trackedIbiUs_;
    ++result_.accepted;
    amplitude_ = amplitude_ > 0.0f
        ? 0.85f * amplitude_ + 0.15f * amplitude : amplitude;
    lastEstimateUs_ = time;
    result_.ready = result_.valid = true;
    result_.hr = beatHr_;
    // Report the period actually used for independent HIGH HR; this is
    // distinct from original LMS IBI published elsewhere by main.cpp.
    result_.ibiMs = trackedIbiUs_ * 0.001f;
    result_.trackedIbiMs = result_.ibiMs;
    result_.beatAgeMs = 0;
    result_.confidence = confidence;
    result_.reason = Reason::NONE;
}
Result Processor::read(uint32_t nowUs) const {
    Result out = result_;
    out.beatAgeMs = haveBeat_ ? static_cast<float>(nowUs - lastBeat_) * 0.001f : NAN;
    if (!out.active) { out.valid = false; out.reason = Reason::INACTIVE; }
    if (out.active && out.valid &&
        nowUs - lastEstimateUs_ > config_.staleUs) {
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
float Processor::selectIbi(float original, bool highMotion, uint32_t nowUs) const {
#if HIGH_MOTION_OUTPUT_OVERRIDE
    Result out = read(nowUs);
    if (highMotion) return out.valid ? out.ibiMs : NAN;
#else
    (void)highMotion; (void)nowUs;
#endif
    return original;
}
void Processor::recordProcessingTime(uint32_t elapsedUs) {
    result_.maxProcessingUs = std::max(result_.maxProcessingUs, elapsedUs);
}
}
