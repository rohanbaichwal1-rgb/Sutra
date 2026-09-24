/*
  Sutra - ESP32-S3 + MAX30101 + BMI270
  LMS/Elgendi corrected build - pulse-region peak selection
  Dual detector:
    1. Reference detector on raw green
    2. LMS + Elgendi detector

  IMPORTANT:
  The original working MAX30100 code was ~600 Hz.
  This ESP32-S3 version uses sampleAverage = 2.

  The Elgendi windows CAN be calculated from the actual measured
  LMS sample rate (see updateElgendiWindows()) rather than assuming
  the MAX30101's configured 400 Hz is the LMS rate. However, for
  THIS build the windows are deliberately LOCKED at W1=11/W2=67
  (tuned for the measured ~100 Hz stream) and updateElgendiWindows()
  is NOT called from loop() - measuredLmsRateHz is diagnostic-only.
  See the "LOCKED 100 Hz ELGENDI CONFIGURATION" comment in setup().

  BUG FIXES IN THIS BUILD (see inline comments at each site):
  1. LMS adaptation is now gated off when the motion reference is
     inside a noise floor, so the filter no longer random-walks on
     spurious correlation with near-zero motion (previously drifted
     by hundreds of units over a ~10 min session with the wearer
     essentially still, which corrupted the cleaned PPG signal and
     kept RMSSD from ever stabilizing -> baseline calibration stuck).
  2. LMS_REFRACTORY_MS lowered from 400ms to 300ms so it matches
     MIN_IBI_MS / HR_MAX_PLAUSIBLE_BPM (200 BPM) instead of silently
     capping detectable HR at 150 BPM regardless of signal quality.
  3. pulseAboveThreshold now compares the W1 envelope value that was
     actually co-timed with the candidate peak (t-1) against its
     co-timed threshold, instead of mixing in the current sample's
     envelope/threshold, removing a 1-sample timing skew.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "DigitalFilter.h"
#include <7Semi_BMI270.h>
#include <RmssdReferences.h>
#include <BeatRecovery.h>
#include <HapticPatterns.h>
#include "HapticWireBus.h"
#include "NvsBaselineStore.h"
#include "BleTelemetry.h"
#include "PdrService.h"
#include <esp_system.h>

rmssd::Monitor referenceMonitor;
personal::Collector sessionCollector;
personal::History personalBaseline;
NvsBaselineStore baselineStore;

void resetElgendiState();
enum MotionState
{
    MOTION_LOW,
    MOTION_MODERATE,
    MOTION_HIGH
};

// Motion state globals are declared here because the LMS/IBI functions
// below use motionState before the BMI270 section appears later in the file.
MotionState motionState = MOTION_LOW;
MotionState motionCandidateState = MOTION_LOW;
uint32_t motionCandidateSinceMs = 0;
uint32_t motionStateEnteredMs = 0;

rmssd::Motion getReferenceMotion()
{
    return motionState == MOTION_HIGH ? rmssd::Motion::HIGH_MOTION : motionState == MOTION_MODERATE ? rmssd::Motion::MODERATE_MOTION
                                                                                                    : rmssd::Motion::LOW_MOTION;
}

MAX30105 particleSensor;
BMI270_7Semi imu;

// ============================================================
// SENSOR CONFIG
// ============================================================

#define SAMPLE_RATE_HZ 400
#define I2C_SDA 5
#define I2C_SCL 6
#define I2C_SPEED 400000UL

#define REPORTING_PERIOD_MS 1000

uint32_t tsLastReport = 0;

// ============================================================
// REFERENCE DETECTOR
// ============================================================

DigitalFilter smoothFilter(
    DigitalFilter::FilterType::IIR_LOWPASS,
    SAMPLE_RATE_HZ,
    6.0f);

DigitalFilter baselineFilter(
    DigitalFilter::FilterType::IIR_LOWPASS,
    SAMPLE_RATE_HZ,
    0.5f);

const float PEAK_HYSTERESIS = 8.0f;
const uint32_t MIN_EXTREME_INTERVAL_MS = 150;
const uint32_t BEAT_REFRACTORY_MS = 400;

const uint32_t MIN_IBI_MS = 300;
const uint32_t MAX_IBI_MS = 2000;

const float BPM_EMA_ALPHA = 0.2f;

const float MIN_ABS_AMPLITUDE_FLOOR = 8.0f;
const float ADAPTIVE_AMPLITUDE_FRACTION = 0.45f;
const float ADAPTIVE_AMPLITUDE_ALPHA = 0.25f;

const uint8_t PLAUSIBILITY_SEED_BEATS = 3;
const float PLAUSIBILITY_TOLERANCE = 0.35f;
const uint32_t STALE_RESEED_MS = 3000;

const float FINGER_PRESENT_THRESHOLD = 5000.0f;
const uint32_t CONTACT_SETTLE_MS = 1500;

bool fingerPresent = false;
uint32_t contactSettleUntilMs = 0;

bool seekingPeak = true;
float extremeVal = 0.0f;
bool haveExtreme = false;

uint32_t lastExtremeConfirmMs = 0;
bool haveLastExtremeConfirm = false;

float lastTroughAc = 0.0f;
bool haveTrough = false;

uint32_t lastBeatMs = 0;
bool haveLastBeat = false;

float emaIBI = 0.0f;
bool haveEmaIBI = false;

uint16_t acceptedBeatCount = 0;

float runningAmplitude = 0.0f;
bool haveRunningAmplitude = false;

float currentBPM = 0.0f;
uint32_t currentIBI = 0;

float smoothedBPM = 0.0f;
bool haveSmoothedBPM = false;

void resetReferenceBeatState()
{
    seekingPeak = true;
    extremeVal = 0.0f;
    haveExtreme = false;

    haveLastExtremeConfirm = false;

    lastTroughAc = 0.0f;
    haveTrough = false;

    haveLastBeat = false;

    emaIBI = 0.0f;
    haveEmaIBI = false;

    acceptedBeatCount = 0;

    runningAmplitude = 0.0f;
    haveRunningAmplitude = false;

    currentBPM = 0.0f;
    currentIBI = 0;

    haveSmoothedBPM = false;
    smoothedBPM = 0.0f;
}

// ============================================================
// LMS FILTER
// Parameters copied from the WORKING 600 Hz reference
// ============================================================

const float DC_REMOVER_ALPHA = 0.95f;

// NLMS adaptation gain. Kept responsive enough for real movement,
// while the fast-motion gate below prevents adaptation on resting IMU noise.
const float LMS_MU = 0.003f;
const float LMS_DENOM_EPS = 1.0f;

// The previous +/-100 range was far larger than required by the measured
// motion reference and allowed a bad transient to corrupt the PPG for a long
// time. Keep the adaptive coefficient bounded.
const float LMS_WEIGHT_MIN = -25.0f;
const float LMS_WEIGHT_MAX = 25.0f;

// Leakage term: without this, small residual correlation between
// motion and PPG lets the weight integrate slowly forever (we saw
// it drift 0 -> -330 over ~10 minutes in a 15-min bench test and
// never settle). Leakage pulls the weight back toward zero each
// update unless the gradient term keeps actively fighting it,
// which bounds long-run drift while still letting real, sustained
// motion-noise correlation adapt normally.
// PLACEHOLDER - tune against real sessions. Time constant at
// ~100Hz sample rate: tau_seconds ~= 1 / (LMS_MU_RATE_HZ * LMS_LEAK).
// Separate leakage for movement and rest:
// - during real movement, keep learned cancellation long enough to work;
// - at rest, return the coefficient to zero quickly so old movement cannot
//   distort the clean PPG after the hand becomes still.
const float LMS_LEAK_ACTIVE = 0.0002f;
const float LMS_LEAK_REST = 0.0050f;

// Resting log showed MotionRef roughly within a few thousandths of g.
// This floor is used as an additional safety check, NOT as the only motion
// detector. Real adaptation is controlled by the fast BMI270 motion gate.
const float MOTION_REF_NOISE_FLOOR = 0.0040f; // g

const float LMS_MOTION_SCALE = 1000.0f;  // g -> milli-g
const float LMS_MAX_ADAPT_ERROR = 80.0f; // bound PPG gradient shock
const float LMS_MAX_UPDATE = 0.08f;      // bound one-sample weight step

// Fast LMS motion gate. This is intentionally more responsive than the
// displayed LOW/MODERATE/HIGH state machine. Sitting data was around
// ~0.001-0.003 g accel std and ~0.6 deg/s gyro RMS, so these thresholds
// leave margin above the measured rest noise while responding quickly
// when the user actually starts moving.
const float LMS_FAST_ACC_STD_ON = 0.0060f;  // g
const float LMS_FAST_ACC_STD_OFF = 0.0040f; // g
const float LMS_FAST_GYRO_ON = 2.0f;        // deg/s
const float LMS_FAST_GYRO_OFF = 1.2f;       // deg/s

#define LMS_FAST_MOTION_ON_MS 200UL
#define LMS_FAST_MOTION_OFF_MS 600UL

float g_rawDC = 0.0f;
bool g_rawDcInitialized = false;

float g_lmsWeight = 0.0f;

float g_latestMotionRef = 0.0f;

// Fast, debounced motion gate used ONLY by LMS adaptation.
// It turns on quickly when movement begins and turns off more slowly,
// preventing chatter around the threshold.
bool g_lmsFastMotion = false;
uint32_t g_lmsFastMotionCandidateSinceMs = 0;
bool g_lmsFastMotionCandidate = false;

// ============================================================
// EFFECTIVE LMS SAMPLE-RATE MEASUREMENT
// ============================================================

uint32_t lmsSamplesThisSecond = 0;
uint32_t lmsTotalSamples = 0;

float measuredLmsRateHz = SAMPLE_RATE_HZ;

uint32_t rateWindowStartMs = 0;

// ============================================================
// ELGENDI PARAMETERS
//
// Working reference:
//
//   600 Hz
//   W1 = 67
//   W2 = 400
//   beta = 0.02
//
// Time equivalents:
//
//   W1 = 111 ms
//   W2 = 667 ms
//
// We calculate the windows from the ACTUAL LMS rate.
// ============================================================

const float ELGENDI_W1_SEC = 0.111f;
const float ELGENDI_W2_SEC = 0.667f;

const float ELGENDI_BETA = 0.02f;

int W1_SAMPLES = 11;
int W2_SAMPLES = 67;

// Maximum storage.
// This is large enough for the expected rates.
#define MAX_ELGENDI_W1 100
#define MAX_ELGENDI_W2 700

float w1Buf[MAX_ELGENDI_W1] = {0};
float w2Buf[MAX_ELGENDI_W2] = {0};

int w1Idx = 0;
int w2Idx = 0;

float w1Sum = 0.0f;
float w2Sum = 0.0f;

float zMeanEMA = 0.0f;

// ============================================================
// ELGENDI STATE
// ============================================================

bool inBlock = false;
int blockLen = 0;

bool pulseRegionActive = false;
float pulseRegionPeakZ = 0.0f;
uint32_t pulseRegionPeakMs = 0;

float lmsRunningPeakAmplitude = 0.0f;
bool lmsHaveRunningPeakAmplitude = false;

uint32_t lastCleanedBeatMs = 0;
beats::Recovery lmsRecovery;
static_assert(STALE_RESEED_MS == beats::Recovery::STALE_MS, "Keep freshness timeouts aligned");

// Local-peak history used by the Elgendi detector.
bool g_haveZHistory = false;
float g_zMinus2 = 0.0f;
float g_zMinus1 = 0.0f;
float g_thrMinus1 = 0.0f;
float g_maPeakMinus1 = 0.0f; // BUG FIX: W1 envelope co-timed with g_zMinus1
uint32_t g_tMinus1 = 0;

uint32_t latestCleanedIBI = 0;
bool newCleanedIBIAvailable = false;

float lmsHR_smoothed = 0.0f;
uint32_t lmsBeatCount = 0;

// Robust recent-IBI history. This is used only to reject obvious
// double-counted and missed beats without locking the detector to one
// previous IBI.
#define LMS_IBI_HISTORY_SIZE 7
uint32_t lmsIbiHistory[LMS_IBI_HISTORY_SIZE] = {0};
uint8_t lmsIbiHistoryCount = 0;
uint8_t lmsIbiHistoryHead = 0;

float medianRecentLmsIbi()
{
    if (lmsIbiHistoryCount == 0)
        return 0.0f;

    uint32_t v[LMS_IBI_HISTORY_SIZE];
    for (uint8_t i = 0; i < lmsIbiHistoryCount; i++)
    {
        uint8_t idx =
            (lmsIbiHistoryHead + LMS_IBI_HISTORY_SIZE -
             lmsIbiHistoryCount + i) %
            LMS_IBI_HISTORY_SIZE;
        v[i] = lmsIbiHistory[idx];
    }

    for (uint8_t i = 1; i < lmsIbiHistoryCount; i++)
    {
        uint32_t key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key)
        {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = key;
    }

    return (float)v[lmsIbiHistoryCount / 2];
}

void rememberLmsIbi(uint32_t ibi)
{
    lmsIbiHistory[lmsIbiHistoryHead] = ibi;
    lmsIbiHistoryHead =
        (lmsIbiHistoryHead + 1) % LMS_IBI_HISTORY_SIZE;

    if (lmsIbiHistoryCount < LMS_IBI_HISTORY_SIZE)
        lmsIbiHistoryCount++;
}

// Reject only patterns that are strongly characteristic of a
// dicrotic/notch double beat or a missed beat. A large genuine HR
// change is still allowed once the history is short or the candidate
// is not an extreme ratio.
bool isRobustLmsIbiCandidate(uint32_t ibiMs)
{
    if (lmsIbiHistoryCount < 4)
        return true;

    float med = medianRecentLmsIbi();
    if (med <= 0.0f)
        return true;

    // The latest Polar comparison showed the remaining MAX30101 errors
    // were isolated short/long beat detections. Use a robust local window.
    // During HIGH motion we widen it slightly so a genuine rapid HR change
    // during exercise is not rejected just because the previous rhythm was
    // slower.
    float lowRatio = (motionState == MOTION_HIGH) ? 0.68f : 0.75f;
    float highRatio = (motionState == MOTION_HIGH) ? 1.42f : 1.30f;

    if ((float)ibiMs < lowRatio * med)
        return false;

    if ((float)ibiMs > highRatio * med)
        return false;

    return true;
}

// During initial acquisition only, a long interval can mean the optical
// detector saw every second pulse. For rhythm seeding we may use half of that
// interval if it lands in the normal low-motion acquisition range. This value
// is NEVER sent to RMSSD because it is reconstructed rather than directly
// observed beat-to-beat timing.
// Resting acquisition / half-rate recovery limits.
// Declared before getAcquisitionRhythmIbi() because that function uses them.
const uint32_t LMS_LOW_MOTION_ACQ_MIN_IBI_MS = 430UL;  // ~140 bpm
const uint32_t LMS_LOW_MOTION_ACQ_MAX_IBI_MS = 1100UL; // ~55 bpm
const uint32_t LMS_HALF_RATE_MIN_MS = 1100UL;
const uint32_t LMS_HALF_RATE_MAX_MS = 1800UL;

bool getAcquisitionRhythmIbi(uint32_t measuredIbi, uint32_t &rhythmIbi, bool &reconstructed)
{
    rhythmIbi = measuredIbi;
    reconstructed = false;

    if (motionState == MOTION_LOW)
    {
        if (measuredIbi >= LMS_HALF_RATE_MIN_MS &&
            measuredIbi <= LMS_HALF_RATE_MAX_MS)
        {
            uint32_t halfIbi = measuredIbi / 2UL;
            if (halfIbi >= LMS_LOW_MOTION_ACQ_MIN_IBI_MS &&
                halfIbi <= LMS_LOW_MOTION_ACQ_MAX_IBI_MS)
            {
                rhythmIbi = halfIbi;
                reconstructed = true;
                return true;
            }
        }

        if (measuredIbi < LMS_LOW_MOTION_ACQ_MIN_IBI_MS ||
            measuredIbi > LMS_LOW_MOTION_ACQ_MAX_IBI_MS)
            return false;
    }

    return measuredIbi >= MIN_IBI_MS && measuredIbi <= 1800UL;
}

const uint32_t HR_FIRST_VALUE_COUNT = 4;

const float MAX_PLAUSIBLE_CLEANED = 200.0f;

const float HR_MIN_PLAUSIBLE_BPM = 33.0f;
const float HR_MAX_PLAUSIBLE_BPM = 200.0f;

// LMS/Elgendi beat-selection protection.
// Only a refractory period is used here; there is intentionally
// NO hard comparison against the previous IBI because one bad
// first beat can otherwise lock the detector into rejecting all
// subsequent normal beats.
//
// BUG FIX: this was 400ms, which caps the max detectable rate at
// 150 BPM (60000/400) - well below HR_MAX_PLAUSIBLE_BPM (200) and
// MIN_IBI_MS (300ms -> 200 BPM). Anything between 150-200 BPM was
// structurally unreachable regardless of signal quality, and the
// two ceilings disagreed with each other. Lowered to 300ms so the
// refractory period is no longer the binding constraint - the
// local-peak + amplitude-floor + threshold checks above do the
// real reject work. NOTE: bench-test this against dicrotic-notch
// false doubles at rest (long diastole, notch well separated from
// the systolic peak) before relying on it during exercise.
const uint32_t LMS_REFRACTORY_MS = 300;
// Lowered after the sitting comparison showed a persistent half-rate lock:
// weaker true systolic pulses were being rejected, so the detector sometimes
// kept only every second pulse. The robust IBI gate below still protects
// against dicrotic-notch double counts.
const float LMS_MIN_RELATIVE_PEAK = 0.25f;

// Initial rhythm acquisition. Do not publish HR from the first one or two
// optical intervals: startup transients can be a false short beat or a
// missed-beat interval around 2x the true IBI.
#define LMS_ACQUISITION_IBIS 4

// Debug values
float dbgMAPeak = 0;
float dbgMABeat = 0;
float dbgTHR1 = 0;

int dbgLastBlockLen = 0;

// ============================================================
// CALCULATE ELGENDI WINDOWS
// ============================================================

void updateElgendiWindows()
{
    int newW1 = (int)roundf(ELGENDI_W1_SEC * measuredLmsRateHz);
    int newW2 = (int)roundf(ELGENDI_W2_SEC * measuredLmsRateHz);

    newW1 = constrain(newW1, 5, MAX_ELGENDI_W1);
    newW2 = constrain(newW2, newW1 + 5, MAX_ELGENDI_W2);

    if (newW1 != W1_SAMPLES || newW2 != W2_SAMPLES)
    {
        W1_SAMPLES = newW1;
        W2_SAMPLES = newW2;

        resetElgendiState();

        Serial.print("LMS rate = ");
        Serial.print(measuredLmsRateHz, 1);
        Serial.print(" Hz -> W1=");
        Serial.print(W1_SAMPLES);
        Serial.print(" W2=");
        Serial.println(W2_SAMPLES);
    }
}

// ============================================================
// RESET ELGENDI
// ============================================================

void resetElgendiState()
{
    for (int i = 0; i < MAX_ELGENDI_W1; i++)
        w1Buf[i] = 0.0f;

    for (int i = 0; i < MAX_ELGENDI_W2; i++)
        w2Buf[i] = 0.0f;

    w1Idx = 0;
    w2Idx = 0;

    w1Sum = 0.0f;
    w2Sum = 0.0f;

    zMeanEMA = 0.0f;

    inBlock = false;
    blockLen = 0;

    pulseRegionActive = false;
    pulseRegionPeakZ = 0.0f;
    pulseRegionPeakMs = 0;

    lmsRunningPeakAmplitude = 0.0f;
    lmsHaveRunningPeakAmplitude = false;

    lastCleanedBeatMs = 0;
    lmsRecovery.reset();

    latestCleanedIBI = 0;
    newCleanedIBIAvailable = false;

    lmsHR_smoothed = 0.0f;
    lmsBeatCount = 0;

    for (int i = 0; i < LMS_IBI_HISTORY_SIZE; i++)
        lmsIbiHistory[i] = 0;
    lmsIbiHistoryCount = 0;
    lmsIbiHistoryHead = 0;

    dbgMAPeak = 0;
    dbgMABeat = 0;
    dbgTHR1 = 0;
    dbgLastBlockLen = 0;

    g_haveZHistory = false;
    g_zMinus2 = 0.0f;
    g_zMinus1 = 0.0f;
    g_thrMinus1 = 0.0f;
    g_maPeakMinus1 = 0.0f;
    g_tMinus1 = 0;
}

// ============================================================
// LMS RESET
//
// Important when finger comes off/on.
// ============================================================

// Forward declarations: these functions are defined later in the file
// but are used by resetLMSState().
void resetLmsConvergenceState();
void resetRmssdStabilityState();
void resetRmssdSessionState();

void resetLMSState()
{
    pdrService::contactChanged(millis());
    g_rawDC = 0.0f;
    g_rawDcInitialized = false;

    g_lmsWeight = 0.0f;
    g_latestMotionRef = 0.0f;
    g_lmsFastMotion = false;
    g_lmsFastMotionCandidate = false;
    g_lmsFastMotionCandidateSinceMs = 0;

    resetLmsConvergenceState();
    resetRmssdStabilityState();
    resetRmssdSessionState();
}

// Forward declaration: defined in the RMSSD engine section below,
// but used inside detectBeatOnCleanedSignal() above that point.
bool addIbiToRmssdBuffer(uint32_t ibiMs, uint32_t nowMs);

// ============================================================
// ELGENDI BEAT DETECTOR
// ============================================================

void detectBeatOnCleanedSignal(float cleanedValue)
{
    if (!fingerPresent)
    {
        resetElgendiState();
        return;
    }

    // LMS cleaned signal is inverted relative to the positive pulse
    // lobe used by the Elgendi detector.
    float mirrored = -cleanedValue;

    if (fabsf(mirrored) > MAX_PLAUSIBLE_CLEANED)
        mirrored = 0.0f;

    float y = mirrored > 0.0f ? mirrored : 0.0f;
    float z = y * y;

    // W1 moving average.
    w1Sum -= w1Buf[w1Idx];
    w1Buf[w1Idx] = z;
    w1Sum += z;
    w1Idx++;
    if (w1Idx >= W1_SAMPLES)
        w1Idx = 0;
    float maPeak = w1Sum / (float)W1_SAMPLES;

    // W2 moving average.
    w2Sum -= w2Buf[w2Idx];
    w2Buf[w2Idx] = z;
    w2Sum += z;
    w2Idx++;
    if (w2Idx >= W2_SAMPLES)
        w2Idx = 0;
    float maBeat = w2Sum / (float)W2_SAMPLES;

    zMeanEMA += 0.001f * (z - zMeanEMA);
    float thr1 = maBeat + ELGENDI_BETA * zMeanEMA;

    dbgMAPeak = maPeak;
    dbgMABeat = maBeat;
    dbgTHR1 = thr1;

    uint32_t now = millis();
    bool aboveThreshold = (maPeak > thr1);

    // FIX: collect one strongest crest per Elgendi supra-threshold
    // pulse region. The previous implementation accepted every local
    // maximum, which allowed dicrotic/notch peaks to become separate
    // 300-400 ms IBIs.
    if (aboveThreshold)
    {
        if (!pulseRegionActive)
        {
            pulseRegionActive = true;
            pulseRegionPeakZ = 0.0f;
            pulseRegionPeakMs = now;
            blockLen = 0;
        }

        blockLen++;

        if (z > pulseRegionPeakZ)
        {
            pulseRegionPeakZ = z;
            pulseRegionPeakMs = now;
        }

        dbgLastBlockLen = blockLen;
        return;
    }

    // Threshold region ended: emit exactly one beat using the
    // strongest crest from the region.
    if (pulseRegionActive)
    {
        pulseRegionActive = false;

        float peakAmplitude = sqrtf(pulseRegionPeakZ);
        uint32_t peakMs = pulseRegionPeakMs;
        uint32_t regionDurationMs = blockLen * 10UL; // ~100 Hz stream
        blockLen = 0;

        // Reject unrealistically long threshold regions. This avoids
        // turning a merged/noisy region into a 1.2-2 s RMSSD interval.
        bool regionDurationOK =
            (regionDurationMs >= 40UL && regionDurationMs <= 1200UL);

        bool absoluteAmplitudeOK =
            (peakAmplitude >= MIN_ABS_AMPLITUDE_FLOOR);

        // Reject tiny secondary crests after the first true pulse.
        // This uses pulse morphology, NOT a previous-IBI predictor.
        bool relativeAmplitudeOK =
            (!lmsHaveRunningPeakAmplitude) ||
            (peakAmplitude >=
             LMS_MIN_RELATIVE_PEAK * lmsRunningPeakAmplitude);

        if (regionDurationOK &&
            absoluteAmplitudeOK &&
            relativeAmplitudeOK)
        {
            if (lmsRecovery.observe(peakMs))
            {
                // Replace a stale rhythm only after four consistent observed
                // candidate intervals. Keep the existing displayed-HR smoothing.
                lmsIbiHistoryCount = lmsIbiHistoryHead = 0;
                for (unsigned i = 0; i < beats::Recovery::REQUIRED; ++i)
                    rememberLmsIbi(lmsRecovery.interval(i));
                lmsRecovery.clearCandidates();
                lastCleanedBeatMs = peakMs;
                latestCleanedIBI = 0;
                newCleanedIBIAvailable = false;
                // Do not join HRV intervals across an uncertain rhythm segment.
                resetRmssdSessionState();
                resetRmssdStabilityState();
                referenceMonitor.interruptContinuity();
                sessionCollector.interrupt();
                pdrService::contactChanged(peakMs);
                lmsRunningPeakAmplitude = peakAmplitude;
                lmsHaveRunningPeakAmplitude = true;
                Serial.println("[LMS] rhythm reacquired; waiting for accepted beat");
                return; // Still stale until the next genuinely accepted beat.
            }
            bool acceptedPeak = false;
            if (lastCleanedBeatMs == 0)
            {
                // First valid crest only seeds timing.
                lastCleanedBeatMs = peakMs;
                acceptedPeak = true;
            }
            else
            {
                uint32_t sinceLast = peakMs - lastCleanedBeatMs;

                // No previous-IBI lockout is used. Only physiological
                // range + refractory safety margin remain.
                if (sinceLast >= LMS_REFRACTORY_MS &&
                    sinceLast >= MIN_IBI_MS &&
                    sinceLast <= 1800UL)
                {
                    // -------------------------------------------------
                    // INITIAL RHYTHM ACQUISITION
                    // -------------------------------------------------
                    // The previous build could publish HR after a single bad
                    // startup interval. Once that happened, the history could
                    // lock to every-second-pulse timing (~1.2-1.5 s). Build a
                    // small rhythm history first and only then expose HR/IBI.
                    if (lmsIbiHistoryCount < LMS_ACQUISITION_IBIS)
                    {
                        uint32_t rhythmIbi = sinceLast;
                        bool reconstructed = false;

                        if (getAcquisitionRhythmIbi(
                                sinceLast,
                                rhythmIbi,
                                reconstructed))
                        {
                            lastCleanedBeatMs = peakMs;
                            rememberLmsIbi(rhythmIbi);
                            acceptedPeak = !reconstructed;

                            // Do NOT send acquisition/reconstructed timing to
                            // RMSSD. HRV starts only after rhythm lock.
                            if (lmsIbiHistoryCount >= LMS_ACQUISITION_IBIS)
                            {
                                float medIbi = medianRecentLmsIbi();
                                latestCleanedIBI = (uint32_t)roundf(medIbi);
                                lmsHR_smoothed = 60000.0f / medIbi;
                                lmsBeatCount = LMS_ACQUISITION_IBIS;
                                newCleanedIBIAvailable = true;
                                if (!reconstructed) lmsRecovery.accept(peakMs);
                                else lmsRecovery.seedRhythm(peakMs);
                            }
                        }
                        else
                        {
                            // Bad startup interval. Re-seed timing at this crest
                            // without poisoning rhythm history.
                            lastCleanedBeatMs = peakMs;
                        }
                    }
                    else
                    {
                        bool robustIbiOK =
                            isRobustLmsIbiCandidate(sinceLast);

                        if (robustIbiOK)
                        {
                            latestCleanedIBI = sinceLast;
                            newCleanedIBIAvailable = true;
                            lastCleanedBeatMs = peakMs;

                            if (addIbiToRmssdBuffer(
                                    latestCleanedIBI,
                                    peakMs))
                                pdrService::beat(peakMs, peakAmplitude, latestCleanedIBI);

                            lmsRecovery.accept(peakMs);
                            acceptedPeak = true;

                            rememberLmsIbi(sinceLast);

                            float robustIbiForHr = medianRecentLmsIbi();
                            float targetHR = 60000.0f / robustIbiForHr;

                            // Allow faster tracking during movement/exercise,
                            // but keep resting output wearable-like and stable.
                            float maxStep =
                                (motionState == MOTION_HIGH) ? 5.0f : (motionState == MOTION_MODERATE) ? 3.5f
                                                                                                       : 2.5f;

                            float alpha =
                                (motionState == MOTION_HIGH) ? 0.28f : (motionState == MOTION_MODERATE) ? 0.20f
                                                                                                        : 0.15f;

                            float delta = targetHR - lmsHR_smoothed;
                            delta = constrain(delta, -maxStep, maxStep);
                            lmsHR_smoothed += alpha * delta;
                            lmsBeatCount++;
                        }
                        else
                        {
                            float medIbi = medianRecentLmsIbi();

                            // Short reject: likely dicrotic/notch; keep timing
                            // from the last accepted pulse. Long reject: likely
                            // missed beat; re-seed here so recovery is immediate.
                            if (medIbi > 0.0f &&
                                (float)sinceLast > 1.30f * medIbi)
                            {
                                lastCleanedBeatMs = peakMs;
                            }
                        }
                    }
                }
                else if (sinceLast > 1800UL)
                {
                    // Lost synchronization. Re-seed at this crest, but
                    // do not feed the long interval into RMSSD.
                    lastCleanedBeatMs = peakMs;
                }
            }

            // Track amplitude only from a valid seed or a robustly accepted
            // beat. This prevents a rejected notch from changing the
            // amplitude floor used by later pulses.
            bool amplitudeSeedOrAccepted =
                (!lmsHaveRunningPeakAmplitude) ||
                acceptedPeak;

            if (amplitudeSeedOrAccepted)
            {
                if (!lmsHaveRunningPeakAmplitude)
                {
                    lmsRunningPeakAmplitude = peakAmplitude;
                    lmsHaveRunningPeakAmplitude = true;
                }
                else
                {
                    const float AMP_EMA_ALPHA = 0.20f;
                    lmsRunningPeakAmplitude =
                        AMP_EMA_ALPHA * peakAmplitude +
                        (1.0f - AMP_EMA_ALPHA) * lmsRunningPeakAmplitude;
                }
            }
        }
    }

    dbgLastBlockLen = 0;
}

// ============================================================
// LMS VALIDITY
// ============================================================

bool isLmsHRValid()
{
    return fingerPresent && lmsBeatCount >= LMS_ACQUISITION_IBIS && lmsIbiHistoryCount >= LMS_ACQUISITION_IBIS && lmsRecovery.fresh(millis()) && lmsHR_smoothed >= HR_MIN_PLAUSIBLE_BPM && lmsHR_smoothed <= HR_MAX_PLAUSIBLE_BPM;
}

// ============================================================
// REFERENCE VALIDITY
// ============================================================

bool isReferenceHRValid()
{
    return fingerPresent && haveLastBeat && ((millis() - lastBeatMs) < STALE_RESEED_MS) && currentBPM >= HR_MIN_PLAUSIBLE_BPM && currentBPM <= HR_MAX_PLAUSIBLE_BPM;
}

// ============================================================
// RMSSD ENGINE
//
// RMSSD = sqrt(mean of squared successive IBI differences)
// over a sliding time window. Fed by the LMS/Elgendi cleaned
// IBI stream (the primary cleaned output), one push per beat.
// ============================================================

#define RMSSD_BUFFER_SIZE 200   // circular buffer capacity
#define RMSSD_WINDOW_MS 60000UL // 60s sliding window (PLACEHOLDER - client will tune)
#define RMSSD_MIN_DIFFS 5       // minimum successive-diff pairs to trust RMSSD

struct IbiSample
{
    uint32_t ibiMs;
    uint32_t tMs;
};

IbiSample rmssdBuf[RMSSD_BUFFER_SIZE];
int rmssdHead = 0;
int rmssdCount = 0;

float currentRMSSD = 0.0f;
bool rmssdValid = false;

// ============================================================
// RMSSD ARTIFACT REJECTION (Malik's rule)
//
// BUG FIX / root cause of "RMSSD never stays stable": nothing was
// screening the IBI stream before it hit the RMSSD buffer. RMSSD is
// sqrt(MEAN of SQUARED successive differences) - one missed beat
// (IBI ~2x too long) or one double-counted beat (IBI ~2x too short)
// dominates the sum-of-squares for as long as it sits in the
// RMSSD_WINDOW_MS buffer. Real session logs showed currentRMSSD
// sitting at 150-530ms (a physiologically plausible resting RMSSD is
// roughly 15-100ms) and jumping 10-30ms second to second - both are
// signatures of occasional bad IBIs, not real HRV. That jitter is
// also exactly what kept updateRmssdStability() from ever holding an
// 8ms-wide window for 10 consecutive seconds after the first (lucky)
// latch.
//
// Fix: reject a candidate IBI for RMSSD purposes if it differs from
// the last RMSSD-ACCEPTED IBI by more than RMSSD_ARTIFACT_MAX_PCT.
// This is intentionally separate from the HR path's own
// PLAUSIBILITY_TOLERANCE check - HR only needs to look reasonable
// after EMA smoothing, RMSSD needs every individual sample to be
// trustworthy because nothing smooths it before use downstream.
// A rejected beat still updates HR as before; it simply never
// reaches the RMSSD buffer.
// ============================================================

const float RMSSD_ARTIFACT_MAX_PCT = 0.20f;  // reject if >20% jump vs last accepted RMSSD IBI
const uint8_t RMSSD_ARTIFACT_SEED_BEATS = 3; // don't gate the first few beats of a session

uint32_t g_lastAcceptedRmssdIbiMs = 0;
uint16_t g_rmssdAcceptedBeatCount = 0;

bool isIbiPlausibleForRmssd(uint32_t ibiMs)
{
    // Defense in depth - caller already enforces MIN_IBI_MS/MAX_IBI_MS,
    // but RMSSD is sensitive enough that this stays as a second check.
    if (ibiMs < MIN_IBI_MS || ibiMs > MAX_IBI_MS)
        return false;

    if (g_rmssdAcceptedBeatCount < RMSSD_ARTIFACT_SEED_BEATS ||
        g_lastAcceptedRmssdIbiMs == 0)
        return true;

    float pctChange =
        fabsf((float)ibiMs - (float)g_lastAcceptedRmssdIbiMs) /
        (float)g_lastAcceptedRmssdIbiMs;

    return pctChange <= RMSSD_ARTIFACT_MAX_PCT;
}

// Call once per accepted beat (from detectBeatOnCleanedSignal).
// Only pushes into the RMSSD buffer if the IBI passes the artifact
// check above; a rejected beat leaves the buffer untouched so the
// next candidate is still compared against the last GOOD IBI, not
// the rejected one.
bool addIbiToRmssdBuffer(uint32_t ibiMs, uint32_t nowMs)
{
    if (!isIbiPlausibleForRmssd(ibiMs))
        return false;

    rmssdBuf[rmssdHead].ibiMs = ibiMs;
    rmssdBuf[rmssdHead].tMs = nowMs;

    rmssdHead = (rmssdHead + 1) % RMSSD_BUFFER_SIZE;

    if (rmssdCount < RMSSD_BUFFER_SIZE)
        rmssdCount++;

    g_lastAcceptedRmssdIbiMs = ibiMs;
    g_rmssdAcceptedBeatCount++;
    return true;
}

// Start a fresh RMSSD session when finger contact changes.
// This prevents old RMSSD values from remaining visible after
// the finger is removed or a new contact session begins.
void resetRmssdSessionState()
{
    for (int i = 0; i < RMSSD_BUFFER_SIZE; i++)
    {
        rmssdBuf[i].ibiMs = 0;
        rmssdBuf[i].tMs = 0;
    }

    rmssdHead = 0;
    rmssdCount = 0;
    currentRMSSD = 0.0f;
    rmssdValid = false;

    g_lastAcceptedRmssdIbiMs = 0;
    g_rmssdAcceptedBeatCount = 0;
}

// Recompute RMSSD from whatever buffered IBIs fall inside the
// last RMSSD_WINDOW_MS. Call periodically (e.g. once per report cycle).
void updateRMSSD(uint32_t nowMs)
{
    static uint32_t ibiWindow[RMSSD_BUFFER_SIZE];
    int n = 0;

    // Walk buffer oldest -> newest
    for (int i = 0; i < rmssdCount; i++)
    {
        int idx = (rmssdHead - rmssdCount + i + RMSSD_BUFFER_SIZE) % RMSSD_BUFFER_SIZE;

        if (nowMs - rmssdBuf[idx].tMs <= RMSSD_WINDOW_MS)
        {
            ibiWindow[n++] = rmssdBuf[idx].ibiMs;
        }
    }

    if (n < RMSSD_MIN_DIFFS + 1)
    {
        rmssdValid = false;
        return;
    }

    float sumSqDiff = 0.0f;
    int diffCount = 0;

    for (int i = 1; i < n; i++)
    {
        float d = (float)ibiWindow[i] - (float)ibiWindow[i - 1];
        sumSqDiff += d * d;
        diffCount++;
    }

    if (diffCount < RMSSD_MIN_DIFFS)
    {
        rmssdValid = false;
        return;
    }

    currentRMSSD = sqrtf(sumSqDiff / (float)diffCount);
    rmssdValid = true;
}

// ============================================================
// BMI270
// ============================================================

float accelDC = 1.0f;

const float ACCEL_DC_ALPHA = 0.01f;

static float latestAX = 0;
static float latestAY = 0;
static float latestAZ = 0;

static float latestGX = 0;
static float latestGY = 0;
static float latestGZ = 0;

static float latestTemp = 0;

static bool latestAccelOK = false;
static bool latestGyroOK = false;
static bool latestTempOK = false;

static float latestMotionStd = 999.0f;

float getMotionReference()
{
    float magnitude =
        sqrtf(
            latestAX * latestAX +
            latestAY * latestAY +
            latestAZ * latestAZ);

    accelDC +=
        ACCEL_DC_ALPHA *
        (magnitude - accelDC);

    return magnitude - accelDC;
}

// ============================================================
// MOTION CLASSIFIER (Low / Moderate / High, with hysteresis)
//
// Combines accel-magnitude variability AND gyro RMS. Gyro is
// included specifically because loose/breadboard-mounted IMUs
// pick up cable/connector vibration that can spike gyro
// readings without genuine body movement - accel alone can
// look deceptively "calm" while gyro is noisy, or vice versa.
//
// Hysteresis (debounce) matters more than the exact thresholds
// right now: a spurious 1-2 sample spike from a loose wire
// should NOT flip the state. Only a change that is SUSTAINED
// for MOTION_STATE_HYSTERESIS_MS is accepted. This is exactly
// the mechanism to test against your breadboard-noise scenario -
// tune the thresholds/hysteresis until brief wiring artifacts
// stop causing state flips, while genuine sustained motion
// still correctly classifies as Moderate/High.
// ============================================================

#define MOTION_BUF_SIZE 50 // ~0.5s @ 100 Hz BMI270 rate

// --- accel magnitude std buffer ---
float motionMagBuf[MOTION_BUF_SIZE] = {0};
int motionMagIdx = 0;
bool motionBufFull = false;

// --- gyro magnitude RMS buffer ---
float gyroMagBuf[MOTION_BUF_SIZE] = {0};
int gyroMagIdx = 0;
bool gyroBufFull = false;

// PLACEHOLDERS - all of these need tuning against real sessions
// (still sitting vs deliberate movement vs exercise), ideally
// cross-checked against Polar H10 so you know which physiological
// windows must NOT get misclassified as High.
// Tuned from the supplied sitting log: rest was approximately
// 0.001-0.003 g accel std and ~0.6 deg/s gyro RMS.
const float ACC_STD_LOW_THRESHOLD = 0.006f;  // g
const float ACC_STD_HIGH_THRESHOLD = 0.050f; // g
const float GYRO_RMS_LOW_THRESHOLD = 2.0f;   // deg/s
const float GYRO_RMS_HIGH_THRESHOLD = 12.0f; // deg/s

// Enter movement quickly; require longer calm before declaring LOW again.
#define MOTION_ENTER_HYSTERESIS_MS 350UL
#define MOTION_EXIT_HYSTERESIS_MS 1500UL

float latestGyroRms = 999.0f;

// Call once per BMI270 sample (100 Hz block in loop())
float pushMotionSampleAndGetStd(float ax, float ay, float az)
{
    float mag = sqrtf(ax * ax + ay * ay + az * az);

    motionMagBuf[motionMagIdx] = mag;
    motionMagIdx++;

    if (motionMagIdx >= MOTION_BUF_SIZE)
    {
        motionMagIdx = 0;
        motionBufFull = true;
    }

    int count = motionBufFull ? MOTION_BUF_SIZE : motionMagIdx;

    if (count < 2)
        return 999.0f; // not enough data yet -> treat as "not low motion"

    float mean = 0.0f;
    for (int i = 0; i < count; i++)
        mean += motionMagBuf[i];
    mean /= (float)count;

    float var = 0.0f;
    for (int i = 0; i < count; i++)
    {
        float d = motionMagBuf[i] - mean;
        var += d * d;
    }
    var /= (float)count;

    return sqrtf(var);
}

// Call once per BMI270 sample (100 Hz block in loop())
float pushGyroSampleAndGetRms(float gx, float gy, float gz)
{
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);

    gyroMagBuf[gyroMagIdx] = mag;
    gyroMagIdx++;

    if (gyroMagIdx >= MOTION_BUF_SIZE)
    {
        gyroMagIdx = 0;
        gyroBufFull = true;
    }

    int count = gyroBufFull ? MOTION_BUF_SIZE : gyroMagIdx;

    if (count < 1)
        return 999.0f;

    float sumSq = 0.0f;
    for (int i = 0; i < count; i++)
        sumSq += gyroMagBuf[i] * gyroMagBuf[i];

    return sqrtf(sumSq / (float)count);
}

// Raw instantaneous classification, before hysteresis is applied.
MotionState classifyMotionInstant(float accStd, float gyroRms)
{
    bool accHigh = accStd > ACC_STD_HIGH_THRESHOLD;
    bool gyroHigh = gyroRms > GYRO_RMS_HIGH_THRESHOLD;

    bool accLow = accStd < ACC_STD_LOW_THRESHOLD;
    bool gyroLow = gyroRms < GYRO_RMS_LOW_THRESHOLD;

    if (accHigh || gyroHigh)
        return MOTION_HIGH;

    if (accLow && gyroLow)
        return MOTION_LOW;

    return MOTION_MODERATE;
}

// Call once per BMI270 sample. Applies hysteresis so brief
// spikes (wiring noise, single gestures) don't flip the
// confirmed motionState - only sustained change does.
void updateMotionState(uint32_t nowMs)
{
    MotionState instant = classifyMotionInstant(latestMotionStd, latestGyroRms);

    if (instant != motionCandidateState)
    {
        motionCandidateState = instant;
        motionCandidateSinceMs = nowMs;
    }

    if (motionCandidateState != motionState)
    {
        // Moving away from LOW should happen quickly. Returning toward LOW
        // requires a longer stable period so cable/hand jitter does not
        // repeatedly toggle the state.
        bool enteringMoreMotion = ((int)motionCandidateState > (int)motionState);
        uint32_t requiredHoldMs =
            enteringMoreMotion ? MOTION_ENTER_HYSTERESIS_MS
                               : MOTION_EXIT_HYSTERESIS_MS;

        if ((nowMs - motionCandidateSinceMs) >= requiredHoldMs)
        {
            motionState = motionCandidateState;
            motionStateEnteredMs = nowMs;

            Serial.print("[MOTION] state -> ");
            Serial.println(
                motionState == MOTION_LOW ? "LOW" : motionState == MOTION_MODERATE ? "MODERATE"
                                                                                   : "HIGH");
        }
    }
}

// Fast gate for adaptive filtering. This is deliberately independent of the
// slower displayed MotionState so LMS can start cancelling motion artifact
// within a few hundred milliseconds of movement onset.
void updateLmsFastMotionGate(uint32_t nowMs)
{
    if (!fingerPresent ||
        !latestAccelOK || !latestGyroOK ||
        latestMotionStd > 100.0f || latestGyroRms > 1000.0f)
    {
        // Invalid/uninitialised IMU data must never drive adaptation.
        g_lmsFastMotion = false;
        g_lmsFastMotionCandidate = false;
        g_lmsFastMotionCandidateSinceMs = nowMs;
        return;
    }

    bool wantsOn =
        (latestMotionStd >= LMS_FAST_ACC_STD_ON) ||
        (latestGyroRms >= LMS_FAST_GYRO_ON);

    bool wantsOff =
        (latestMotionStd <= LMS_FAST_ACC_STD_OFF) &&
        (latestGyroRms <= LMS_FAST_GYRO_OFF);

    bool requestedState = g_lmsFastMotion;

    if (!g_lmsFastMotion && wantsOn)
        requestedState = true;
    else if (g_lmsFastMotion && wantsOff)
        requestedState = false;

    if (requestedState != g_lmsFastMotionCandidate)
    {
        g_lmsFastMotionCandidate = requestedState;
        g_lmsFastMotionCandidateSinceMs = nowMs;
    }

    if (g_lmsFastMotionCandidate != g_lmsFastMotion)
    {
        uint32_t holdMs =
            g_lmsFastMotionCandidate ? LMS_FAST_MOTION_ON_MS
                                     : LMS_FAST_MOTION_OFF_MS;

        if ((nowMs - g_lmsFastMotionCandidateSinceMs) >= holdMs)
            g_lmsFastMotion = g_lmsFastMotionCandidate;
    }
}

const char *getMotionStateName()
{
    switch (motionState)
    {
    case MOTION_LOW:
        return "LOW";
    case MOTION_MODERATE:
        return "MODERATE";
    case MOTION_HIGH:
        return "HIGH";
    }
    return "?";
}

bool isLowMotionPlaceholder()
{
    return motionState == MOTION_LOW;
}

bool isSignalQualityGoodPlaceholder()
{
    // Simplified stand-in: LMS detector locked on + physiologically plausible.
    return isLmsHRValid();
}

// ============================================================
// LMS CONVERGENCE GATE
//
// Confirms the adaptive filter has actually settled before
// letting the baseline module trust RMSSD samples. Tracks
// g_lmsWeight once per report cycle (1s) and requires the
// range (max-min) over a trailing window to stay below a
// tight tolerance for CONVERGENCE_HOLD_MS before declaring
// "converged". This directly targets the failure seen in
// testing: baseline locking in during a window where
// LMSweight was still climbing, producing a baseline that
// didn't match the filter's actual settled RMSSD.
// ============================================================

#define LMS_CONVERGENCE_BUF_SIZE 20           // 20 samples @ 1/s = 20s trailing window
#define LMS_CONVERGENCE_RANGE_THRESHOLD 20.0f // relaxed from 5.0 - see note below
#define LMS_CONVERGENCE_HOLD_MS 10000UL       // must stay within range for this long to GAIN convergence
#define LMS_CONVERGENCE_LOSS_HOLD_MS 5000UL   // must stay OUT of range for this long to LOSE convergence
                                              // (debounce - see note below). PLACEHOLDER - tune.
#define LMS_WEIGHT_SMOOTH_ALPHA 0.2f          // EMA smoothing applied before the range check

// Asymmetric hysteresis note: gaining convergence requires
// LMS_CONVERGENCE_HOLD_MS of sustained in-range behavior, but losing it
// used to happen on a single out-of-range window. Testing showed this
// flapped convergence 4x in ~3.5 minutes even while g_lmsWeight was
// sitting in a fairly tame range overall - a brief one-window wobble was
// enough to trip an all-or-nothing check. LMS_CONVERGENCE_LOSS_HOLD_MS
// requires the SAME sustained-out-of-range proof to lose convergence that
// LMS_CONVERGENCE_HOLD_MS requires to gain it (deliberately set shorter,
// so genuine re-drift is still caught reasonably quickly) - a single brief
// wobble that recovers within the grace period no longer flips the gate.

float lmsWeightHistory[LMS_CONVERGENCE_BUF_SIZE] = {0};
int lmsWeightHistIdx = 0;
bool lmsWeightHistFull = false;

uint32_t lastLmsSampleMs = 0;
uint32_t lmsConvergedSinceMs = 0;
uint32_t lmsOutOfRangeSinceMs = 0; // when a currently-converged gate first saw an out-of-range window
bool lmsConverged = false;

float lmsWeightSmoothed = 0.0f;
bool lmsWeightSmoothedInit = false;

// Call once per report cycle (1s) - NOT per raw sample, so the
// window represents ~20s of real time, not 20 rapid samples.
void updateLmsConvergence(uint32_t nowMs)
{
    if (nowMs - lastLmsSampleMs < 1000UL)
        return;

    lastLmsSampleMs = nowMs;

    // Smooth the weight before feeding the convergence check.
    // 15-min bench testing showed the raw per-second weight can
    // jitter by more than the original 5.0 threshold even once
    // the underlying trend has genuinely leveled off - which
    // meant convergence could never latch. Smoothing (EMA) plus
    // a more realistic threshold targets "has the trend settled"
    // rather than "is this exact instant noise-free".
    if (!lmsWeightSmoothedInit)
    {
        lmsWeightSmoothed = g_lmsWeight;
        lmsWeightSmoothedInit = true;
    }
    else
    {
        lmsWeightSmoothed =
            (LMS_WEIGHT_SMOOTH_ALPHA * g_lmsWeight) +
            ((1.0f - LMS_WEIGHT_SMOOTH_ALPHA) * lmsWeightSmoothed);
    }

    lmsWeightHistory[lmsWeightHistIdx] = lmsWeightSmoothed;
    lmsWeightHistIdx++;

    if (lmsWeightHistIdx >= LMS_CONVERGENCE_BUF_SIZE)
    {
        lmsWeightHistIdx = 0;
        lmsWeightHistFull = true;
    }

    int count = lmsWeightHistFull ? LMS_CONVERGENCE_BUF_SIZE : lmsWeightHistIdx;

    if (count < LMS_CONVERGENCE_BUF_SIZE)
    {
        // Not enough history yet - definitely not converged.
        lmsConverged = false;
        lmsConvergedSinceMs = 0;
        lmsOutOfRangeSinceMs = 0;
        return;
    }

    float minVal = lmsWeightHistory[0];
    float maxVal = lmsWeightHistory[0];

    for (int i = 1; i < count; i++)
    {
        if (lmsWeightHistory[i] < minVal)
            minVal = lmsWeightHistory[i];
        if (lmsWeightHistory[i] > maxVal)
            maxVal = lmsWeightHistory[i];
    }

    bool withinRange = (maxVal - minVal) <= LMS_CONVERGENCE_RANGE_THRESHOLD;

    if (!withinRange)
    {
        if (!lmsConverged)
        {
            // Never converged yet - out-of-range just delays the initial
            // gain, same behavior as before. No debounce needed here.
            lmsConvergedSinceMs = 0;
            lmsOutOfRangeSinceMs = 0;
            return;
        }

        // Currently converged, this window is out-of-range: don't drop
        // convergence immediately. Require sustained out-of-range for
        // LMS_CONVERGENCE_LOSS_HOLD_MS first (see hysteresis note above).
        if (lmsOutOfRangeSinceMs == 0)
            lmsOutOfRangeSinceMs = nowMs;

        if ((nowMs - lmsOutOfRangeSinceMs) >= LMS_CONVERGENCE_LOSS_HOLD_MS)
        {
            Serial.println("[LMS] convergence LOST - weight drifting again");
            lmsConverged = false;
            lmsConvergedSinceMs = 0;
            lmsOutOfRangeSinceMs = 0;
        }
        // else: still inside the grace window, stay converged and wait
        return;
    }

    // withinRange == true this cycle: cancel any pending loss-of-convergence
    lmsOutOfRangeSinceMs = 0;

    if (lmsConvergedSinceMs == 0)
        lmsConvergedSinceMs = nowMs;

    if (!lmsConverged &&
        (nowMs - lmsConvergedSinceMs) >= LMS_CONVERGENCE_HOLD_MS)
    {
        lmsConverged = true;
        Serial.print("[LMS] CONVERGED, weight range=");
        Serial.println(maxVal - minVal, 2);
    }
}

bool isLmsConvergedGate()
{
    return lmsConverged;
}

// Called when the finger comes off/on (resetLMSState) so a new
// session starts convergence detection cleanly rather than
// carrying over the previous session's smoothed weight/history.
void resetLmsConvergenceState()
{
    for (int i = 0; i < LMS_CONVERGENCE_BUF_SIZE; i++)
        lmsWeightHistory[i] = 0.0f;

    lmsWeightHistIdx = 0;
    lmsWeightHistFull = false;

    lastLmsSampleMs = 0;
    lmsConvergedSinceMs = 0;
    lmsOutOfRangeSinceMs = 0;
    lmsConverged = false;

    lmsWeightSmoothed = 0.0f;
    lmsWeightSmoothedInit = false;
}

// ============================================================
// RMSSD STABILITY GATE
//
// LMSconverged only proves the ADAPTIVE FILTER'S WEIGHT has settled -
// it says nothing about whether RMSSD itself has leveled off. Testing
// showed a real failure from trusting LMSconverged alone: baseline
// calibration folded in qualifying samples of 57 -> 76 -> 92 -> 106 ->
// 119 -> 131 -> 146 ms, climbing steadily for the entire 120s
// calibration window, and still called the resulting ~106ms average
// an "established" baseline even though RMSSD had never actually
// leveled off - the baseline landed on roughly the midpoint of a
// still-rising trend, not a true steady-state value.
//
// This mirrors the LMS convergence gate's structure exactly (trailing
// window, range check, hold-before-latch), but tracks currentRMSSD
// instead of g_lmsWeight. The baseline module below now requires BOTH
// gates before it will accept a sample.
// ============================================================

#define RMSSD_STABILITY_BUF_SIZE 20         // 20 samples @ 1/s = 20s trailing window
#define RMSSD_STABILITY_RANGE_FLOOR_MS 8.0f // range check never tighter than this
#define RMSSD_STABILITY_RANGE_PCT 0.12f     // ...or 12% of the window mean, whichever is larger.
                                            // BUG FIX: a flat 8ms band is only realistic for a
                                            // small RMSSD (~20-40ms). Real session data showed
                                            // RMSSD legitimately sitting at 150-300+ms while the
                                            // artifact filter above was still bedding in, and an
                                            // 8ms ABSOLUTE band at that magnitude is far tighter
                                            // than the beat-to-beat measurement noise floor, so
                                            // stability could only ever latch by chance for a few
                                            // seconds and then was effectively unrecoverable. A
                                            // percentage-of-magnitude band scales with the signal
                                            // instead of assuming one fixed noise floor for every
                                            // possible RMSSD value. PLACEHOLDER - tune against
                                            // real sessions once RMSSD is in a plausible range.
#define RMSSD_STABILITY_HOLD_MS 10000UL     // must stay within range this long to GAIN stability
#define RMSSD_STABILITY_LOSS_HOLD_MS 5000UL // must stay OUT of range this long to LOSE stability
                                            // (same debounce reasoning as the LMS gate above)

float rmssdHistory[RMSSD_STABILITY_BUF_SIZE] = {0};
int rmssdHistIdx = 0;
bool rmssdHistFull = false;

uint32_t lastRmssdStabilitySampleMs = 0;
uint32_t rmssdStableSinceMs = 0;
uint32_t rmssdOutOfRangeSinceMs = 0;
bool rmssdStable = false;

// Light 3-sample median pre-filter feeding the stability window only
// (NOT the reported currentRMSSD, which stays raw). This absorbs the
// occasional single-sample spike that survives artifact rejection
// (e.g. one beat right at the edge of the 20% Malik threshold)
// without hiding genuine multi-second trends from the range check.
float g_rmssdMedianBuf[3] = {0.0f, 0.0f, 0.0f};
int g_rmssdMedianIdx = 0;
bool g_rmssdMedianFull = false;

float medianOf3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a))
        return b;
    if ((b <= a && a <= c) || (c <= a && a <= b))
        return a;
    return c;
}

float pushRmssdMedian(float v)
{
    g_rmssdMedianBuf[g_rmssdMedianIdx] = v;
    g_rmssdMedianIdx = (g_rmssdMedianIdx + 1) % 3;

    if (g_rmssdMedianIdx == 0)
        g_rmssdMedianFull = true;

    if (!g_rmssdMedianFull)
        return v; // not enough history yet - pass through

    return medianOf3(g_rmssdMedianBuf[0], g_rmssdMedianBuf[1], g_rmssdMedianBuf[2]);
}

void resetRmssdMedian()
{
    g_rmssdMedianBuf[0] = g_rmssdMedianBuf[1] = g_rmssdMedianBuf[2] = 0.0f;
    g_rmssdMedianIdx = 0;
    g_rmssdMedianFull = false;
}

// Call once per report cycle (1s), same cadence as updateLmsConvergence.
// Only samples currentRMSSD when rmssdValid, so an invalid/missing RMSSD
// reading pauses the check rather than corrupting the trailing window.
void updateRmssdStability(uint32_t nowMs)
{
    if (nowMs - lastRmssdStabilitySampleMs < 1000UL)
        return;

    lastRmssdStabilitySampleMs = nowMs;

    if (!rmssdValid)
        return; // pause - don't fold a garbage/missing sample into the window

    float smoothedRmssd = pushRmssdMedian(currentRMSSD);

    rmssdHistory[rmssdHistIdx] = smoothedRmssd;
    rmssdHistIdx++;

    if (rmssdHistIdx >= RMSSD_STABILITY_BUF_SIZE)
    {
        rmssdHistIdx = 0;
        rmssdHistFull = true;
    }

    int count = rmssdHistFull ? RMSSD_STABILITY_BUF_SIZE : rmssdHistIdx;

    if (count < RMSSD_STABILITY_BUF_SIZE)
    {
        rmssdStable = false;
        rmssdStableSinceMs = 0;
        rmssdOutOfRangeSinceMs = 0;
        return;
    }

    float minVal = rmssdHistory[0];
    float maxVal = rmssdHistory[0];
    float sumVal = 0.0f;

    for (int i = 0; i < count; i++)
    {
        if (rmssdHistory[i] < minVal)
            minVal = rmssdHistory[i];
        if (rmssdHistory[i] > maxVal)
            maxVal = rmssdHistory[i];
        sumVal += rmssdHistory[i];
    }

    float meanVal = sumVal / (float)count;

    float rangeThreshold =
        max(RMSSD_STABILITY_RANGE_FLOOR_MS, RMSSD_STABILITY_RANGE_PCT * meanVal);

    bool withinRange = (maxVal - minVal) <= rangeThreshold;

    if (!withinRange)
    {
        if (!rmssdStable)
        {
            rmssdStableSinceMs = 0;
            rmssdOutOfRangeSinceMs = 0;
            return;
        }

        if (rmssdOutOfRangeSinceMs == 0)
            rmssdOutOfRangeSinceMs = nowMs;

        if ((nowMs - rmssdOutOfRangeSinceMs) >= RMSSD_STABILITY_LOSS_HOLD_MS)
        {
            Serial.println("[RMSSD] stability LOST - trend moving again");
            rmssdStable = false;
            rmssdStableSinceMs = 0;
            rmssdOutOfRangeSinceMs = 0;
        }
        return;
    }

    rmssdOutOfRangeSinceMs = 0;

    if (rmssdStableSinceMs == 0)
        rmssdStableSinceMs = nowMs;

    if (!rmssdStable &&
        (nowMs - rmssdStableSinceMs) >= RMSSD_STABILITY_HOLD_MS)
    {
        rmssdStable = true;
        Serial.print("[RMSSD] STABLE, range=");
        Serial.println(maxVal - minVal, 2);
    }
}

bool isRmssdStableGate()
{
    return rmssdStable;
}

void resetRmssdStabilityState()
{
    for (int i = 0; i < RMSSD_STABILITY_BUF_SIZE; i++)
        rmssdHistory[i] = 0.0f;

    rmssdHistIdx = 0;
    rmssdHistFull = false;

    lastRmssdStabilitySampleMs = 0;
    rmssdStableSinceMs = 0;
    rmssdOutOfRangeSinceMs = 0;
    rmssdStable = false;

    resetRmssdMedian();
}

// ============================================================
// SEVEN-SESSION PERSONAL BASELINE (PROTOTYPE, NO CALENDAR)
// Each boot collects one median from 20 qualifying five-second resting blocks.
// Only the persisted median of seven sessions enables the LONG_TERM path.
// ============================================================

bool baselineSaveError = false;
bool baselineSaveAttempted = false;
uint32_t lastBaselineSaveAttemptMs = 0;
const uint32_t BASELINE_SAVE_RETRY_MS = 10000UL;

void updateBaseline(uint32_t nowMs)
{
    // Collect this boot's resting measurement independently of the short
    // reference. Collector stops accepting data at 100 s.
    bool qualifies = referenceMonitor.restingGate() == rmssd::CollectionGate::OPEN &&
                     isLmsConvergedGate();
    sessionCollector.update(nowMs, currentRMSSD, qualifies);
    if (!sessionCollector.complete() || personalBaseline.savedThisBoot() ||
        !personalBaseline.storageReady() || !qualifies)
        return;
    if (baselineSaveAttempted && nowMs - lastBaselineSaveAttemptMs < BASELINE_SAVE_RETRY_MS)
        return;
    baselineSaveAttempted = true;
    lastBaselineSaveAttemptMs = nowMs;
    if (!personalBaseline.saveSession(sessionCollector.median()))
    {
        if (!baselineSaveError)
            Serial.println("[Personal baseline] SAVE FAILED; retrying, previous history retained");
        baselineSaveError = true;
        return;
    }
    baselineSaveError = false;
    Serial.print("[Personal baseline] session SAVED | Median:");
    Serial.print(sessionCollector.median(), 1);
    Serial.print("ms | Saved sessions:");
    Serial.print(personalBaseline.count());
    Serial.print("/7 | Long-term:");
    if (personalBaseline.ready())
    {
        Serial.print(personalBaseline.baseline(), 1);
        Serial.println("ms (READY, SEVEN_SESSIONS)");
    }
    else
        Serial.println("UNAVAILABLE (session + short-term triggers enabled)");
}

// ============================================================
// PHASE-1 PoC: LOCAL P1 / P2 TRIGGERS + HAPTIC ACTUATION
//
// Session, live short-term, and seven-session personal references have independent timers.
// LONG_TERM is enabled only after seven completed sessions are saved in NVS.
// See lib/RmssdReferences for rolling median and recovery rules.
//
// Prototype wiring: one SmartElex DA7280 module with its onboard LRA on
// TCA9548A channel 0. The driver still supports three modules by increasing
// motorCount and wiring channels 1/2 when the hardware is added.
// ============================================================

const uint32_t HAPTIC_FRAME_INTERVAL_MS = 10UL;
const uint32_t HAPTIC_FAULT_POLL_MS = 100UL;
const uint32_t HAPTIC_STOP_RETRY_MS = 250UL;

HapticWireBus hapticBus(Wire);
haptics::ArrayConfig singleMotorHapticConfig()
{
    haptics::ArrayConfig config;
    config.motorCount = 1;
    config.channels[0] = 0;
    config.outputScalePercent = 40; // Map the original 0..100% envelope to 0..40%.
    return config;
}
haptics::ArrayConfig threeMotorBootCheckConfig()
{
    haptics::ArrayConfig config;
    config.motorCount = haptics::MOTOR_COUNT;
    config.channels[0] = 0;
    config.channels[1] = 1;
    config.channels[2] = 2;
    config.outputScalePercent = 40;
    return config;
}
haptics::ArrayConfig hapticArrayConfig = singleMotorHapticConfig();
haptics::Array hapticArray(hapticBus, hapticArrayConfig);
haptics::ArrayConfig bootCheckArrayConfig = threeMotorBootCheckConfig();
haptics::Array bootCheckArray(hapticBus, bootCheckArrayConfig);
haptics::Patterns hapticPatterns;
bool hapticPatternActive = false;
bool hapticFaultReported = false;
uint8_t reportedHapticWarnings = 0;
uint32_t lastHapticFrameMs = 0;
uint32_t lastHapticFaultPollMs = 0;
uint32_t lastHapticStopRetryMs = 0;

const char *getHapticStateName()
{
    if (!hapticArray.ready())
        return hapticArray.outputStopPending() ? "FAULT_STOP_PENDING" : "FAULT";
    if (hapticPatterns.protocol() == haptics::Protocol::P1_FLUTTER)
        return "P1_FLUTTER";
    if (hapticPatterns.protocol() == haptics::Protocol::P2_SWEEP)
        return "P2_SWEEP";
    return "OFF";
}

void reportHapticFault()
{
    if (hapticArray.warningBits() != reportedHapticWarnings)
    {
        reportedHapticWarnings = hapticArray.warningBits();
        Serial.print("[HAPTIC] driver warning | Bits:0x");
        Serial.println(reportedHapticWarnings, HEX);
    }
    if (hapticArray.ready() || hapticFaultReported)
        return;
    hapticFaultReported = true;
    Serial.print("[HAPTIC] disabled | Error:");
    switch (hapticArray.error())
    {
    case haptics::ArrayError::CONFIG:
        Serial.print("CONFIG");
        break;
    case haptics::ArrayError::I2C:
        Serial.print("I2C");
        break;
    case haptics::ArrayError::CHIP_ID:
        Serial.print("CHIP_ID");
        break;
    case haptics::ArrayError::DRIVER_FAULT:
        Serial.print("DRIVER_FAULT");
        break;
    default:
        Serial.print("NONE");
        break;
    }
    Serial.print(" | Motor:");
    Serial.print(hapticArray.errorMotor());
    Serial.print(" | FaultBits:0x");
    Serial.println(hapticArray.faultBits(), HEX);
}

void updateHapticActuator(uint32_t nowMs)
{
    const bool wasP2 = hapticPatterns.protocol() == haptics::Protocol::P2_SWEEP;
    if (!hapticArray.ready())
    {
        if (wasP2)
            pdrService::endP2(nowMs, false);
        hapticPatterns.stop();
        if (hapticArray.shutdownPending() &&
            nowMs - lastHapticStopRetryMs >= HAPTIC_STOP_RETRY_MS)
        {
            lastHapticStopRetryMs = nowMs;
            hapticArray.stopAll();
        }
        hapticPatternActive = hapticArray.outputStopPending();
        reportHapticFault();
        return;
    }
    if (!hapticPatterns.active() ||
        nowMs - lastHapticFrameMs < HAPTIC_FRAME_INTERVAL_MS)
        return;
    lastHapticFrameMs = nowMs;

    const haptics::Frame &frame = hapticPatterns.update(nowMs);
    const bool naturallyCompleted = !hapticPatterns.active();
    if (!hapticPatterns.active())
    {
        if (hapticArray.stopAll())
            Serial.println("[HAPTIC] protocol complete");
    }
    else if (!hapticArray.apply(frame))
        hapticPatterns.stop();

    if (hapticPatterns.active() && nowMs - lastHapticFaultPollMs >= HAPTIC_FAULT_POLL_MS)
    {
        lastHapticFaultPollMs = nowMs;
        if (!hapticArray.pollFaults())
            hapticPatterns.stop();
    }
    hapticPatternActive = hapticPatterns.active() || hapticArray.outputStopPending();
    if (wasP2 && !hapticPatterns.active())
        pdrService::endP2(nowMs, naturallyCompleted && hapticArray.ready() && !hapticArray.outputStopPending());
    reportHapticFault();
}

void startHapticProtocol(haptics::Protocol protocol, uint32_t nowMs)
{
    if (!hapticArray.ready())
    {
        Serial.println("[HAPTIC] trigger received; driver array unavailable");
        return;
    }
    if (protocol == haptics::Protocol::P1_FLUTTER &&
        hapticPatterns.protocol() == haptics::Protocol::P2_SWEEP)
        return; // P2 retains priority

    if (!hapticArray.pollFaults() || !hapticArray.stopAll())
    {
        if (hapticPatterns.protocol() == haptics::Protocol::P2_SWEEP)
            pdrService::endP2(nowMs, false);
        hapticPatterns.stop();
        hapticPatternActive = hapticArray.outputStopPending();
        reportHapticFault();
        return;
    }
    // Fresh hardware seed for each flutter. P2 replaces P1 immediately.
    nowMs = millis(); // Start the envelope after the preparation transfers.
    bool started = protocol == haptics::Protocol::P2_SWEEP
                       ? hapticPatterns.startP2(nowMs)
                       : hapticPatterns.startP1(nowMs, esp_random());
    if (started && !hapticArray.apply(hapticPatterns.frame()))
        hapticPatterns.stop();
    lastHapticFrameMs = lastHapticFaultPollMs = nowMs;
    hapticPatternActive = hapticPatterns.active() || hapticArray.outputStopPending();
    if (hapticPatterns.active())
    {
        if (protocol == haptics::Protocol::P2_SWEEP)
            pdrService::beginP2(nowMs);
        Serial.print("[HAPTIC] started ");
        Serial.print(getHapticStateName());
        Serial.print(" | Duration:");
        Serial.print(hapticPatterns.durationMs() / 1000UL);
        Serial.println("s");
    }
    reportHapticFault();
}

// Startup-only hardware check. It deliberately uses a separate three-motor
// array so normal P1/P2 playback remains on the configured single motor.
void runStartupMotorCheck()
{
    Serial.println("[HAPTIC] BOOT_TEST: all three motors for 10s at 40%; confirm by touch");
    if (!bootCheckArray.begin())
    {
        Serial.print("[HAPTIC] BOOT_TEST failed to initialize | Motor:");
        Serial.print(bootCheckArray.errorMotor());
        Serial.print(" | FaultBits:0x");
        Serial.println(bootCheckArray.faultBits(), HEX);
        return;
    }

    haptics::Patterns bootPattern;
    const uint32_t startedMs = millis();
    bool ok = bootPattern.startBootTest(startedMs) &&
              bootCheckArray.apply(bootPattern.frame());
    uint32_t lastFaultPollMs = startedMs;

    while (ok && bootPattern.active())
    {
        const uint32_t nowMs = millis();
        bootPattern.update(nowMs);
        if (!bootPattern.active())
            break;
        if (nowMs - lastFaultPollMs >= HAPTIC_FAULT_POLL_MS)
        {
            lastFaultPollMs = nowMs;
            ok = bootCheckArray.pollFaults();
        }
        delay(1);
    }

    const bool stopped = bootCheckArray.stopAll();
    if (ok && stopped && bootCheckArray.ready())
        Serial.println("[HAPTIC] BOOT_TEST passed; all motors stopped");
    else
    {
        Serial.print("[HAPTIC] BOOT_TEST failed | Motor:");
        Serial.print(bootCheckArray.errorMotor());
        Serial.print(" | FaultBits:0x");
        Serial.println(bootCheckArray.faultBits(), HEX);
    }
}

// RMSSD can remain mathematically valid while its last accepted beat is old.
// Require fresh RMSSD input as well as the existing LMS-HR validity gate.
bool isReferenceInputFresh(uint32_t nowMs)
{
    if (!rmssdValid || !isLmsHRValid() || rmssdCount == 0)
        return false;
    int newest = (rmssdHead + RMSSD_BUFFER_SIZE - 1) % RMSSD_BUFFER_SIZE;
    return nowMs - rmssdBuf[newest].tMs < STALE_RESEED_MS;
}

void updateStressTriggers(uint32_t nowMs)
{
    rmssd::Input input = {
        nowMs,
        currentRMSSD,
        isReferenceInputFresh(nowMs) && latestAccelOK && latestGyroOK,
        getReferenceMotion(),
        isRmssdStableGate(),
        hapticPatternActive,
        personalBaseline.ready(),
        personalBaseline.baseline(),
        sessionCollector.complete(),
        sessionCollector.median()};
    rmssd::Events events = referenceMonitor.update(input);
    const pdr::Result pdrContext = pdrService::read(nowMs);

    if (events.resumed)
        Serial.println("[P1/P2] recovery complete - triggers rearmed");

    if (events.p1 != rmssd::Source::NONE)
    {
        Serial.print("[P1] TRIGGERED | TriggerSource:");
        Serial.print(rmssd::sourceName(events.p1));
        Serial.print(" | LongTermBasis:SEVEN_SESSIONS | PDR_SUPPORT:");
        Serial.println(pdrContext.support ? "Y" : "N");
    }
    if (events.p2 != rmssd::Source::NONE)
    {
        Serial.print("[P2] TRIGGERED | TriggerSource:");
        Serial.print(rmssd::sourceName(events.p2));
        Serial.print(" | LongTermBasis:SEVEN_SESSIONS | PDR_SUPPORT:");
        Serial.println(pdrContext.support ? "Y" : "N");
        startHapticProtocol(haptics::Protocol::P2_SWEEP, nowMs);
    }
    else if (events.p1 != rmssd::Source::NONE)
    {
        startHapticProtocol(haptics::Protocol::P1_FLUTTER, nowMs);
    }
}

void processSample(uint32_t green, uint32_t nowMs);

// ============================================================
// SETUP
// ============================================================

// Keep a failed startup visible even when the monitor connects after boot.
// Do not enter measurement/actuation with a required sensor unavailable.
[[noreturn]] void reportStartupFailure(const char *message)
{
    while (true)
    {
        Serial.print("[STARTUP ERROR] ");
        Serial.print(message);
        Serial.println(" | Check power/SDA/SCL and press RESET.");
        delay(2000);
    }
}

void setup()
{
    Serial.begin(115200);

    uint32_t serialWaitStart = millis();

    while (!Serial &&
           (millis() - serialWaitStart < 3000))
    {
        delay(10);
    }

    Serial.println();
    Serial.println("==========================================");
    Serial.println("ESP32-S3 + MAX30101 + BMI270");
    Serial.println("Reference + LMS/Elgendi");
    Serial.println("LONG_TERM: median of 7 saved startup sessions (prototype, not calendar days)");
    if (!baselineStore.begin() || !personalBaseline.load(baselineStore))
        Serial.println("[Personal baseline] STORAGE ERROR; long-term unavailable, session/short paths still enabled");
    else
    {
        Serial.print("[Personal baseline] restored ");
        Serial.print(personalBaseline.count());
        Serial.print("/7 sessions | ");
        Serial.println(personalBaseline.ready() ? "READY" : "long-term path disabled");
    }
    Serial.print("Post-exercise recovery: ");
    Serial.print(rmssd::POST_EXERCISE_RECOVERY_MS / 1000UL);
    Serial.println(" seconds of LOW motion after HIGH; MODERATE pauses without resetting");
    Serial.println("==========================================");

#if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(I2C_SDA, I2C_SCL);
#else
    Wire.begin();
#endif

    Wire.setClock(I2C_SPEED);
    Wire.setTimeOut(10); // Bound failed I2C transfers, including mux branches.
    delay(2);            // DA7280 cold-boot allowance; no delays in the playback sequencer.
    runStartupMotorCheck();
    // Sensor initialization continues if the haptic array is absent/faulty.
    Serial.println("[Startup] Checking haptic mux/drivers...");
    if (hapticArray.begin())
        Serial.println("[HAPTIC] 1 x DA7280 ready | M1:CH0 | P1:5s P2:10s | Intensity range:0-40%");
    else
        reportHapticFault();
    hapticPatternActive = hapticArray.outputStopPending();

    // ========================================================
    // MAX30101
    // ========================================================

    Serial.println("[Startup] Checking MAX3010x...");
    if (!particleSensor.begin(Wire, I2C_SPEED))
        reportStartupFailure("MAX3010x not found at 0x57");

    byte powerLevel = 0xFF;

    // ========================================================
    // IMPORTANT:
    // MAX30101 FIFO averaging = 2 (measured LMS stream ~100 Hz)
    // ========================================================

    byte sampleAverage = 2;

    byte ledMode = 3;

    int sampleRate = SAMPLE_RATE_HZ;

    int pulseWidth = 411;

    int adcRange = 4096;

    particleSensor.setup(
        powerLevel,
        sampleAverage,
        ledMode,
        sampleRate,
        pulseWidth,
        adcRange);

    particleSensor.setPulseAmplitudeRed(0);
    particleSensor.setPulseAmplitudeIR(0);

    // ========================================================
    // BMI270
    // ========================================================

    Serial.println("BMI270 [0x69]: initializing...");

    BMI270_7Semi::Config cfg;

    cfg.bus = BMI270_7Semi::Bus::I2C;
    cfg.addr = 0x69;
    cfg.i2c = &Wire;
    cfg.sda = -1;
    cfg.scl = -1;
    cfg.i2cHz = I2C_SPEED;

    if (!imu.begin(cfg))
        reportStartupFailure("BMI270 initialization failed at 0x69");

    Serial.println("BMI270 SUCCESS");

    imu.setAccelConfig(
        BMI2_ACC_ODR_100HZ,
        BMI2_ACC_RANGE_2G,
        BMI2_ACC_NORMAL_AVG4,
        BMI2_PERF_OPT_MODE);

    imu.setGyroConfig(
        BMI2_GYR_ODR_100HZ,
        BMI2_GYR_RANGE_2000,
        BMI2_GYR_NORMAL_MODE,
        BMI2_PERF_OPT_MODE);

    // ========================================================
    // Initial Elgendi configuration
    //
    // sampleAverage = 2 has been measured on this hardware at
    // approximately 100 LMS samples/sec.
    //
    // For this test W1/W2 are deliberately LOCKED:
    //   W1 = 11  (~111 ms)
    //   W2 = 67  (~667 ms)
    //
    // The measured LMS rate will continue to be printed, but
    // it will NOT change/reset the Elgendi detector.
    // ========================================================

    // ========================================================
    // LOCKED 100 Hz ELGENDI CONFIGURATION
    //
    // Actual measured LMS stream with:
    //   MAX sampleRate   = 400 Hz
    //   sampleAverage    = 2
    // is approximately 100 Hz.
    //
    // Keep the Elgendi time windows at approximately:
    //   W1 = 111 ms -> 11 samples @ 100 Hz
    //   W2 = 667 ms -> 67 samples @ 100 Hz
    //
    // These values are NOT recalculated every second.
    // measuredLmsRateHz is still updated below for diagnostics only.
    // ========================================================

    measuredLmsRateHz = 100.0f;
    W1_SAMPLES = 11;
    W2_SAMPLES = 67;

    // Detailed startup parameters (uncomment when tuning).
    // Serial.println();
    // Serial.println("==========================================");
    // Serial.println("INITIAL PARAMETERS");
    // Serial.print("Sensor rate: ");
    // Serial.print(SAMPLE_RATE_HZ);
    // Serial.println(" Hz");
    //
    // Serial.print("MAX301 sampleAverage: ");
    // Serial.println(sampleAverage);
    //
    // Serial.print("LMS mu: ");
    // Serial.println(LMS_MU, 4);
    //
    // Serial.print("LMS motion scale: ");
    // Serial.println(LMS_MOTION_SCALE, 1);
    //
    // Serial.print("LMS max adapt error: ");
    // Serial.println(LMS_MAX_ADAPT_ERROR, 1);
    //
    // Serial.print("DC alpha: ");
    // Serial.println(DC_REMOVER_ALPHA, 3);
    //
    // Serial.print("Elgendi beta: ");
    // Serial.println(ELGENDI_BETA, 3);
    //
    // Serial.print("Initial W1: ");
    // Serial.println(W1_SAMPLES);
    //
    // Serial.print("Initial W2: ");
    // Serial.println(W2_SAMPLES);
    //
    // Serial.println("==========================================");
    // Serial.println();

    Serial.println(pdrService::begin() ? "[PDR] Observer started (context only)" : "[PDR] Task allocation failed");
    Serial.println(telemetry::begin() ? "[BLE] Starting Sutra telemetry" : "[BLE] Task allocation failed");
    tsLastReport = millis();
    rateWindowStartMs = millis();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    static uint32_t previousLoopUs = 0;
    static uint32_t maxLoopGapUs = 0;
    const uint32_t loopUs = micros();
    if (previousLoopUs != 0)
    {
        const uint32_t gap = loopUs - previousLoopUs;
        if (gap > maxLoopGapUs)
            maxLoopGapUs = gap;
    }
    previousLoopUs = loopUs;
    // Run the actuator state machine on every pass without blocking.
    updateHapticActuator(millis());

    particleSensor.check();

    while (particleSensor.available())
    {
        uint32_t green =
            particleSensor.getFIFOGreen();

        particleSensor.nextSample();

        uint32_t nowMs = millis();

        processSample(green, nowMs);

        // --------------------------------------------
        // Count actual LMS samples
        // --------------------------------------------

        if (fingerPresent &&
            nowMs >= contactSettleUntilMs)
        {
            lmsSamplesThisSecond++;
            lmsTotalSamples++;
        }
    }

    // ========================================================
    // BMI270 @ 100 Hz
    // ========================================================

    static uint32_t lastBMI270 = 0;

    uint32_t nowMicros = micros();

    if ((uint32_t)(nowMicros - lastBMI270) >= 10000UL)
    {
        lastBMI270 += 10000UL;

        latestAccelOK =
            imu.readAccel(
                latestAX,
                latestAY,
                latestAZ);

        latestGyroOK =
            imu.readGyro(
                latestGX,
                latestGY,
                latestGZ);

        latestTempOK =
            imu.readTemperatureC(latestTemp);

        // Only update motion features from valid sensor reads.
        // Stale/failed IMU samples must not drive LMS adaptation.
        if (latestAccelOK)
        {
            g_latestMotionRef = getMotionReference();

            latestMotionStd =
                pushMotionSampleAndGetStd(
                    latestAX, latestAY, latestAZ);
        }

        if (latestGyroOK)
        {
            latestGyroRms =
                pushGyroSampleAndGetRms(
                    latestGX, latestGY, latestGZ);
        }

        uint32_t motionNowMs = millis();
        updateMotionState(motionNowMs);
        updateLmsFastMotionGate(motionNowMs);
        referenceMonitor.observeMotion(motionNowMs, getReferenceMotion());
        if (motionState != MOTION_LOW)
            sessionCollector.interrupt();
    }

    // ========================================================
    // Measure effective LMS rate every second
    // ========================================================

    uint32_t nowMs = millis();

    // A brief invalid period between reports must break continuous holds.
    if (!isReferenceInputFresh(nowMs) || !latestAccelOK || !latestGyroOK)
    {
        referenceMonitor.interruptContinuity();
        sessionCollector.interrupt();
    }

    // Independent observer. Its output is never passed to the RMSSD monitor.
    pdrService::context(nowMs, {isReferenceInputFresh(nowMs) && latestAccelOK && latestGyroOK,
                                motionState == MOTION_LOW,
                                !referenceMonitor.postExerciseRecovery(),
                                isLmsConvergedGate() && !referenceMonitor.p1Triggered() && !referenceMonitor.p2Triggered(),
                                hapticPatternActive});

    if (nowMs - rateWindowStartMs >= 1000)
    {
        float elapsed =
            (nowMs - rateWindowStartMs) / 1000.0f;

        if (elapsed > 0)
        {
            measuredLmsRateHz =
                (float)lmsSamplesThisSecond /
                elapsed;

            // IMPORTANT:
            // Keep W1/W2 locked at 11/67 for this 100 Hz test.
            // measuredLmsRateHz is DISPLAY/DIAGNOSTIC ONLY.
            // updateElgendiWindows();
        }

        lmsSamplesThisSecond = 0;
        rateWindowStartMs = nowMs;
    }

    // ========================================================
    // REPORT
    // ========================================================

    if (nowMs - tsLastReport >
        REPORTING_PERIOD_MS)
    {
        // Recompute RMSSD + fold into baseline once per report cycle.
        updateRMSSD(nowMs);
        updateLmsConvergence(nowMs);
        updateRmssdStability(nowMs);
        updateStressTriggers(nowMs);
        updateBaseline(nowMs);

        const bool lmsValid = isLmsHRValid();
        const bool normalValid = isReferenceHRValid();
        const bool fresh = isReferenceInputFresh(nowMs);
        telemetry::Snapshot snapshot{};
        snapshot.hr = lmsValid ? lmsHR_smoothed : NAN;
        snapshot.ibi = lmsValid && latestCleanedIBI > 0 ? medianRecentLmsIbi() : NAN;
        snapshot.rmssd = fresh ? currentRMSSD : NAN;
        snapshot.session = sessionCollector.complete() ? sessionCollector.median() : NAN;
        snapshot.shortReference = referenceMonitor.shortAvailable() ? referenceMonitor.shortReference() : NAN;
        snapshot.personal = personalBaseline.ready() ? personalBaseline.baseline() : NAN;
        snapshot.uptimeSeconds = nowMs / 1000UL;
        snapshot.maxLoopGapUs = maxLoopGapUs;
        snapshot.finger = fingerPresent;
        snapshot.motion = static_cast<uint8_t>(motionState);
        snapshot.converged = isLmsConvergedGate();
        snapshot.stable = isRmssdStableGate();
        snapshot.fresh = fresh;
        snapshot.savedSessions = personalBaseline.count();
        snapshot.p1Triggered = referenceMonitor.p1Triggered();
        snapshot.p2Triggered = referenceMonitor.p2Triggered();
        snapshot.episode = snapshot.p2Triggered ? 2 : snapshot.p1Triggered ? 1 : 0;
        snapshot.respiration = pdrService::read(nowMs);
        if (hapticPatternActive)
        {
            snapshot.respiration.support = false;
            snapshot.respiration.supportMs = 0;
        }
        snapshot.normalHr = normalValid ? currentBPM : NAN;
        snapshot.normalIbi = normalValid && currentIBI > 0 ? currentIBI : NAN;
        snapshot.imuOK = latestAccelOK && latestGyroOK;
        snapshot.p1Ms[0] = referenceMonitor.p1SessionHoldMs(nowMs);
        snapshot.p1Ms[1] = referenceMonitor.p1ShortHoldMs(nowMs);
        snapshot.p1Ms[2] = referenceMonitor.p1LongHoldMs(nowMs);
        snapshot.p2Ms[0] = referenceMonitor.p2SessionHoldMs(nowMs);
        snapshot.p2Ms[1] = referenceMonitor.p2ShortHoldMs(nowMs);
        snapshot.p2Ms[2] = referenceMonitor.p2LongHoldMs(nowMs);
        snapshot.p1Source = referenceMonitor.p1Source();
        snapshot.p2Source = referenceMonitor.p2Source();
        snapshot.sessionValidMs = sessionCollector.validDurationMs();
        snapshot.sessionTargetMs = personal::SESSION_DURATION_MS;
        snapshot.sessionSamples = sessionCollector.sampleCount();
        snapshot.sessionTargetSamples = personal::SESSION_SAMPLES;
        if (sessionCollector.complete())
            snprintf(snapshot.sessionState, sizeof(snapshot.sessionState), "FROZEN");
        else if (referenceMonitor.restingGate() != rmssd::CollectionGate::OPEN)
            snprintf(snapshot.sessionState, sizeof(snapshot.sessionState), "PAUSED:%s",
                     rmssd::collectionGateName(referenceMonitor.restingGate()));
        else
            snprintf(snapshot.sessionState, sizeof(snapshot.sessionState), "%s",
                     snapshot.converged ? "COLLECTING" : "PAUSED:LMS_CONVERGENCE");
        snprintf(snapshot.storageState, sizeof(snapshot.storageState), "%s",
                 !personalBaseline.storageReady() ? "ERROR" : baselineSaveError ? "SAVE_RETRY" :
                 personalBaseline.savedThisBoot() ? "SAVED" : sessionCollector.complete() ? "PENDING" : "COLLECTING");
        snprintf(snapshot.shortState, sizeof(snapshot.shortState), "%s",
                 referenceMonitor.collectionGate() != rmssd::CollectionGate::OPEN ? "PAUSED" :
                 referenceMonitor.shortAvailable() ? "ADAPTING" : "COLLECTING");
        snprintf(snapshot.shortGate, sizeof(snapshot.shortGate), "%s",
                 rmssd::collectionGateName(referenceMonitor.collectionGate()));
        snapshot.shortValidMs = referenceMonitor.validDurationMs();
        snapshot.shortBlockMs = referenceMonitor.sampleBlockMs(nowMs);
        snapshot.shortBreaks = referenceMonitor.collectionBreaks();
        snapshot.postExercise = referenceMonitor.postExerciseRecovery();
        snapshot.postExerciseRemainingMs = referenceMonitor.motionRecoveryRemainingMs(nowMs);
        snapshot.rearmMs = referenceMonitor.recoveryHoldMs(nowMs);
        snprintf(snapshot.hapticState, sizeof(snapshot.hapticState), "%s", getHapticStateName());
        if (hapticPatterns.active())
        {
            snapshot.hapticElapsedMs = hapticPatterns.elapsedMs(millis());
            snapshot.hapticDurationMs = hapticPatterns.durationMs();
        }
        snprintf(snapshot.bleState, sizeof(snapshot.bleState), "%s", telemetry::state());
        newCleanedIBIAvailable = false;
        maxLoopGapUs = 0;
        telemetry::publish(snapshot);

        // Both transports use identical, labeled, newline-terminated groups.
        char line[telemetry::LINE_CAPACITY];
        for (unsigned i = 0; i < static_cast<unsigned>(telemetry::Group::COUNT); ++i)
        {
            if (telemetry::formatLine(static_cast<telemetry::Group>(i), snapshot, line, sizeof(line)))
                Serial.print(line);
            else
                Serial.println("Telemetry formatting error");
            updateHapticActuator(millis());
        }
        Serial.println();

        // Detailed diagnostics (uncomment only when troubleshooting).
        // bool refValid = isReferenceHRValid();
        // Serial.print(" | RefHR:");
        //
        // if (refValid)
        // {
        // Serial.print(currentBPM, 1);
        // Serial.print("bpm");
        // }
        // else
        // {
        // Serial.print("--");
        // }
        //
        // Serial.print(" | RefIBI:");
        //
        // if (refValid)
        // {
        // Serial.print(currentIBI);
        // Serial.print("ms");
        // }
        // else
        // {
        // Serial.print("--");
        // }
        //
        // Serial.print(" | LMS_N:");
        // Serial.print(lmsTotalSamples);
        //
        // Serial.print(" | LMSrate:");
        // Serial.print(measuredLmsRateHz, 1);
        // Serial.print("Hz");
        //
        // Serial.print(" | W1:");
        // Serial.print(W1_SAMPLES);
        //
        // Serial.print(" | W2:");
        // Serial.print(W2_SAMPLES);
        //
        // Serial.print(" | MApeak:");
        // Serial.print(dbgMAPeak, 1);
        //
        // Serial.print(" | MAbeat:");
        // Serial.print(dbgMABeat, 1);
        //
        // Serial.print(" | THR1:");
        // Serial.print(dbgTHR1, 1);
        //
        // Serial.print(" | Block:");
        // Serial.print(dbgLastBlockLen);
        //
        // Serial.print(" | MotionRef:");
        // Serial.print(g_latestMotionRef, 4);
        //
        // Serial.print(" | LMSweight:");
        // Serial.print(g_lmsWeight, 4);
        //
        // Serial.print(" | LMSadapt:");
        // Serial.print(g_lmsFastMotion ? "Y" : "N");
        //
        // Serial.print(" | LMSweightSm:");
        // Serial.print(lmsWeightSmoothed, 4);
        //
        // Serial.print(" | MotionStd:");
        // Serial.print(latestMotionStd, 4);
        //
        // Serial.print(" | GyroRms:");
        // Serial.print(latestGyroRms, 2);
        //
        //
        // // ====================================================
        // // BMI270
        // // ====================================================
        //
        // Serial.print(" | ACC:");
        //
        // if (latestAccelOK)
        // {
        // Serial.print(latestAX, 3);
        // Serial.print(",");
        // Serial.print(latestAY, 3);
        // Serial.print(",");
        // Serial.print(latestAZ, 3);
        // }
        // else
        // {
        // Serial.print("ERROR");
        // }
        //
        // Serial.print(" | GYR:");
        //
        // if (latestGyroOK)
        // {
        // Serial.print(latestGX, 3);
        // Serial.print(",");
        // Serial.print(latestGY, 3);
        // Serial.print(",");
        // Serial.print(latestGZ, 3);
        // }
        // else
        // {
        // Serial.print("ERROR");
        // }
        //
        // Serial.print(" | TEMP:");
        //
        // if (latestTempOK)
        // {
        // Serial.print(latestTemp, 2);
        // Serial.print("C");
        // }
        // else
        // {
        // Serial.print("ERROR");
        // }
        //

        tsLastReport = nowMs;
    }
}

// ============================================================
// PROCESS ONE MAX30101 SAMPLE
// ============================================================

void processSample(
    uint32_t green,
    uint32_t nowMs)
{
    // ========================================================
    // REFERENCE DETECTOR
    // ========================================================

    float fast =
        smoothFilter.filter(
            (float)green);

    float slow =
        baselineFilter.filter(
            (float)green);

    float ac =
        fast - slow;

    bool nowPresent =
        (slow > FINGER_PRESENT_THRESHOLD);

    // ========================================================
    // FINGER STATE CHANGED
    // ========================================================

    if (nowPresent != fingerPresent)
    {
        referenceMonitor.contactChanged();
        sessionCollector.interrupt();
        resetReferenceBeatState();

        resetElgendiState();

        // NEW:
        // Reset LMS adaptive filter too.
        resetLMSState();

        fingerPresent = nowPresent;

        if (fingerPresent)
        {
            contactSettleUntilMs =
                nowMs + CONTACT_SETTLE_MS;
        }
    }

    // ========================================================
    // REFERENCE DETECTOR
    // ========================================================

    if (!fingerPresent)
    {
        haveExtreme = false;
    }
    else if (nowMs < contactSettleUntilMs)
    {
        haveExtreme = false;
    }
    else
    {
        if (!haveExtreme)
        {
            extremeVal = ac;
            haveExtreme = true;
        }
        else if (seekingPeak)
        {
            if (ac > extremeVal)
            {
                extremeVal = ac;
            }
            else if (
                extremeVal - ac >
                    PEAK_HYSTERESIS &&
                (!haveLastExtremeConfirm ||
                 nowMs -
                         lastExtremeConfirmMs >=
                     MIN_EXTREME_INTERVAL_MS))
            {
                float peakVal = extremeVal;

                lastExtremeConfirmMs =
                    nowMs;

                haveLastExtremeConfirm =
                    true;

                bool refractoryOk =
                    (!haveLastBeat) ||
                    (nowMs - lastBeatMs >=
                     BEAT_REFRACTORY_MS);

                if (haveLastBeat &&
                    (nowMs - lastBeatMs >
                     STALE_RESEED_MS))
                {
                    haveLastBeat = false;
                    acceptedBeatCount = 0;
                    haveEmaIBI = false;
                    haveRunningAmplitude = false;
                }

                if (haveTrough &&
                    refractoryOk)
                {
                    float amplitude =
                        peakVal -
                        lastTroughAc;

                    float effectiveThreshold =
                        (acceptedBeatCount <
                             PLAUSIBILITY_SEED_BEATS ||
                         !haveRunningAmplitude)
                            ? MIN_ABS_AMPLITUDE_FLOOR
                            : max(
                                  MIN_ABS_AMPLITUDE_FLOOR,
                                  ADAPTIVE_AMPLITUDE_FRACTION *
                                      runningAmplitude);

                    if (amplitude >=
                        effectiveThreshold)
                    {
                        if (haveLastBeat)
                        {
                            uint32_t ibi =
                                nowMs -
                                lastBeatMs;

                            bool plausible =
                                (acceptedBeatCount <
                                 PLAUSIBILITY_SEED_BEATS) ||
                                !haveEmaIBI ||
                                (fabsf(
                                     (float)ibi -
                                     emaIBI) <=
                                 PLAUSIBILITY_TOLERANCE *
                                     emaIBI);

                            if (
                                ibi >= MIN_IBI_MS &&
                                ibi <= MAX_IBI_MS &&
                                plausible)
                            {
                                currentIBI = ibi;

                                currentBPM =
                                    60000.0f /
                                    (float)ibi;

                                if (!haveEmaIBI)
                                {
                                    emaIBI =
                                        (float)ibi;

                                    haveEmaIBI =
                                        true;
                                }
                                else
                                {
                                    emaIBI =
                                        BPM_EMA_ALPHA *
                                            (float)ibi +
                                        (1.0f -
                                         BPM_EMA_ALPHA) *
                                            emaIBI;
                                }

                                if (!haveRunningAmplitude)
                                {
                                    runningAmplitude =
                                        amplitude;

                                    haveRunningAmplitude =
                                        true;
                                }
                                else
                                {
                                    runningAmplitude =
                                        ADAPTIVE_AMPLITUDE_ALPHA *
                                            amplitude +
                                        (1.0f -
                                         ADAPTIVE_AMPLITUDE_ALPHA) *
                                            runningAmplitude;
                                }

                                acceptedBeatCount++;

                                if (!haveSmoothedBPM)
                                {
                                    smoothedBPM =
                                        currentBPM;

                                    haveSmoothedBPM =
                                        true;
                                }
                                else
                                {
                                    smoothedBPM =
                                        BPM_EMA_ALPHA *
                                            currentBPM +
                                        (1.0f -
                                         BPM_EMA_ALPHA) *
                                            smoothedBPM;
                                }

                                lastBeatMs =
                                    nowMs;

                                haveLastBeat =
                                    true;
                            }
                        }
                        else
                        {
                            lastBeatMs =
                                nowMs;

                            haveLastBeat =
                                true;
                        }
                    }
                }

                seekingPeak = false;
                extremeVal = ac;
            }
        }
        else
        {
            if (ac < extremeVal)
            {
                extremeVal = ac;
            }
            else if (
                ac - extremeVal >
                    PEAK_HYSTERESIS &&
                (!haveLastExtremeConfirm ||
                 nowMs -
                         lastExtremeConfirmMs >=
                     MIN_EXTREME_INTERVAL_MS))
            {
                lastTroughAc =
                    extremeVal;

                haveTrough = true;

                lastExtremeConfirmMs =
                    nowMs;

                haveLastExtremeConfirm =
                    true;

                seekingPeak = true;

                extremeVal = ac;
            }
        }
    }

    // ========================================================
    // LMS PATH
    // ========================================================

    if (fingerPresent &&
        nowMs >= contactSettleUntilMs)
    {
        // --------------------------------------------
        // DC removal
        // SAME alpha as working code
        // --------------------------------------------

        if (!g_rawDcInitialized)
        {
            g_rawDC =
                (float)green;

            g_rawDcInitialized =
                true;
        }

        g_rawDC =
            g_rawDC *
                DC_REMOVER_ALPHA +
            (float)green *
                (1.0f -
                 DC_REMOVER_ALPHA);

        float rawAC =
            (float)green -
            g_rawDC;

        // --------------------------------------------
        // NLMS
        // SAME mu as working code
        // --------------------------------------------

        // Scale the very small accelerometer residual from g into milli-g.
        // This keeps the NLMS state numerically well-conditioned.
        float motionRef =
            g_latestMotionRef * LMS_MOTION_SCALE;

        float predictedNoise =
            g_lmsWeight *
            motionRef;

        float cleaned =
            rawAC -
            predictedNoise;

        // Never allow one PPG spike to create a huge LMS gradient.
        float adaptError =
            constrain(
                cleaned,
                -LMS_MAX_ADAPT_ERROR,
                LMS_MAX_ADAPT_ERROR);

        // Adapt only after the BMI270 has confirmed real movement quickly.
        // MotionRef magnitude remains a second safety check so gyro-only
        // disturbances cannot make a nearly-zero accel reference update the
        // adaptive coefficient with meaningless correlation.
        bool motionAboveFloor =
            fabsf(g_latestMotionRef) >= MOTION_REF_NOISE_FLOOR;

        bool allowLmsAdaptation =
            g_lmsFastMotion &&
            latestAccelOK &&
            latestGyroOK &&
            motionAboveFloor;

        if (allowLmsAdaptation)
        {
            float update =
                LMS_MU *
                adaptError *
                motionRef /
                (LMS_DENOM_EPS +
                 motionRef * motionRef);

            update =
                constrain(
                    update,
                    -LMS_MAX_UPDATE,
                    LMS_MAX_UPDATE);

            g_lmsWeight += update;

            // Very light leakage while active: preserve the learned
            // cancellation during sustained movement.
            g_lmsWeight *= (1.0f - LMS_LEAK_ACTIVE);
        }
        else
        {
            // At rest, adaptation is completely frozen and the old movement
            // coefficient decays rapidly toward zero. This is what prevents
            // the stationary random-walk seen in the previous logs.
            g_lmsWeight *= (1.0f - LMS_LEAK_REST);

            if (fabsf(g_lmsWeight) < 0.0001f)
                g_lmsWeight = 0.0f;
        }

        g_lmsWeight =
            constrain(
                g_lmsWeight,
                LMS_WEIGHT_MIN,
                LMS_WEIGHT_MAX);

        // --------------------------------------------
        // Elgendi
        // --------------------------------------------

        detectBeatOnCleanedSignal(
            cleaned);
    }
}
