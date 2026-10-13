#pragma once
#include <stdint.h>

#ifndef HIGH_MOTION_PROCESSOR_ENABLED
#define HIGH_MOTION_PROCESSOR_ENABLED 1
#endif
#ifndef HIGH_MOTION_OUTPUT_OVERRIDE
#define HIGH_MOTION_OUTPUT_OVERRIDE 1
#endif
#ifndef HIGH_MOTION_DEBUG
#define HIGH_MOTION_DEBUG 1
#endif
#if (HIGH_MOTION_PROCESSOR_ENABLED != 0 && HIGH_MOTION_PROCESSOR_ENABLED != 1) || \
    (HIGH_MOTION_OUTPUT_OVERRIDE != 0 && HIGH_MOTION_OUTPUT_OVERRIDE != 1) || \
    (HIGH_MOTION_DEBUG != 0 && HIGH_MOTION_DEBUG != 1)
#error "HIGH_MOTION switches must be 0 or 1"
#endif

namespace highmotion {
// All parameters are experimental and apply only to this processor.
struct Config {
    float learningRate = 0.015f;
    float coefficientBound = 30.0f;
    float leakagePerSecond = 0.02f;
    float normalization = 25.0f;
    float referenceScale = 1000.0f; // accelerometer g -> milli-g
    float referenceDcSeconds = 0.8f;
    float ppgDcSeconds = 0.8f;
    float lowPassSeconds = 0.025f;
    float maxAdaptError = 150.0f;
    float maxCoefficientStep = 0.1f;
    float motionEnergyFloor = 4.0f;
    float minPulseAmplitude = 3.0f;
    float noiseMultiplier = 0.65f;
    float minRiseSlope = 20.0f; // PPG counts/second; exclude slow baseline recovery
    float noiseSlopeMultiplier = 2.0f;
    float peakDropAmplitudeFraction = 0.15f; // confirm crest after a meaningful fall
    float peakDropRmsFraction = 0.10f;
    float minAmplitudeRatio = 0.35f;
    float maxAmplitudeRatio = 2.8f;
    float maxResidualCorrelation = 0.85f;
    float maxCancellationRatio = 0.95f;
    float intervalTolerance = 0.25f;
    float hrSmoothingSeconds = 2.0f;
    float minConfidence = 0.45f;
    uint32_t minIbiUs = 300000;
    uint32_t maxIbiUs = 1500000;
    uint32_t minRiseUs = 30000;
    uint32_t maxRiseUs = 450000;
    uint32_t staleUs = 2500000;
    uint32_t warmupUs = 2000000;
    uint32_t maxImuGapUs = 30000;
    uint32_t maxPpgGapUs = 40000;
    uint32_t alignmentWaitUs = 40000;
    uint32_t imuLatencyUs = 0; // unknown sensor pipeline delay; calibrate offline
    uint32_t ppgLatencyUs = 0;
    float initialSamplePeriodUs = 10000.0f; // observed stream, not sensor ODR
    uint8_t maxEstimatedBatch = 4;
    uint8_t acquisitionIntervals = 4;
    uint32_t timestampJitterUs = 20000; // tolerated host/FIFO receipt phase error
    float timestampCorrectionFraction = 0.05f; // bounded sample-clock phase correction
};

enum class Reason : uint8_t {
    NONE, INACTIVE, WARMUP, ALIGNMENT, TIMING, NUMERIC, AMPLITUDE,
    MORPHOLOGY, MOTION_COUPLING, INTERVAL, ACQUIRING, STALE, QUEUE_FULL
};
const char *reasonName(Reason reason);

struct Result {
    bool active = false, ready = false, valid = false;
    bool aligned = false, adapting = false, timestampsEstimated = true;
    float hr = 0, ibiMs = 0, confidence = 0, coefficientNorm = 0;
    float samplePeriodUs = 10000;
    float candidateAmplitude = 0, candidateFloor = 0, candidateRiseMs = 0;
    float residualCorrelation = 0, cancellationRatio = 0;
    uint32_t accepted = 0, rejected = 0, alignmentDrops = 0;
    uint32_t maxProcessingUs = 0;
    Reason reason = Reason::INACTIVE;
    Reason lastRejection = Reason::NONE;
};

class Processor {
public:
    static constexpr unsigned TAPS = 4, AXES = 3;
    explicit Processor(const Config &config = Config());
    void setActive(bool highMotion, bool contact, uint32_t nowUs);
    void beginBatch(uint32_t receivedUs, unsigned count);
    void pushPpg(float raw);
    void pushImu(uint32_t readMidpointUs, float ax, float ay, float az, bool valid);
    void poll(uint32_t nowUs);
    Result read(uint32_t nowUs) const;
    // Selection affects presentation only; callers must keep physiological inputs separate.
    float selectHr(float original, bool highMotion, uint32_t nowUs) const;
    void recordProcessingTime(uint32_t elapsedUs);
private:
    struct Imu { uint32_t time; float axis[AXES]; bool valid; };
    struct Ppg { uint32_t time; float raw; bool reliable; };
    static constexpr unsigned IMU_CAPACITY = 32, PPG_CAPACITY = 16, IBI_CAPACITY = 7;
    Config config_;
    Result result_;
    Imu imu_[IMU_CAPACITY]{};
    Ppg ppg_[PPG_CAPACITY]{};
    unsigned imuHead_ = 0, imuCount_ = 0, ppgHead_ = 0, ppgCount_ = 0;
    float weights_[AXES][TAPS]{}, refs_[AXES][TAPS]{}, referenceDc_[AXES]{};
    float rawPower_ = 0, cleanPower_ = 0, predictedPower_ = 0;
    float refPower_[AXES]{}, crossPower_[AXES]{};
    float dc_ = 0, low_ = 0, previous_ = 0, trough_ = 0, amplitude_ = 0;
    float pulsePower_ = 0, peak_ = 0;
    uint32_t peakTime_ = 0;
    float ibis_[IBI_CAPACITY]{}, hr_ = 0;
    unsigned ibiCount_ = 0, ibiHead_ = 0;
    uint32_t started_ = 0, lastSample_ = 0, troughTime_ = 0, previousTime_ = 0, lastBeat_ = 0;
    bool initialized_ = false, rising_ = false, haveBeat_ = false;
    bool contact_ = false, haveSampleClock_ = false;
    uint32_t sampleClock_ = 0;
    uint32_t batchTime_ = 0, rateStart_ = 0, rateSamples_ = 0;
    unsigned batchCount_ = 0, batchIndex_ = 0;
    bool batchReliable_ = false, rateStarted_ = false;
    float periodUs_;
    void resetSignal(uint32_t nowUs);
    bool interpolate(uint32_t time, float *axis) const;
    void process(const Ppg &ppg, const float *axis);
    void candidate(uint32_t time, float amplitude, uint32_t riseUs);
    void reject(Reason reason, bool invalidate = true);
    float medianIbi() const;
};
}
