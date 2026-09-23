#include "RmssdReferences.h"
#include <math.h>

namespace rmssd
{
const char *collectionGateName(CollectionGate gate)
{
    switch (gate)
    {
    case CollectionGate::OPEN: return "OPEN";
    case CollectionGate::SIGNAL_OR_IMU: return "SIGNAL_OR_IMU";
    case CollectionGate::MOTION: return "MOTION";
    case CollectionGate::POST_EXERCISE: return "POST_EXERCISE";
    case CollectionGate::UNSTABLE: return "RMSSD_UNSTABLE";
    case CollectionGate::INTERVENTION: return "HAPTIC_ACTIVE_OR_STOP_PENDING";
    }
    return "UNKNOWN";
}

const char *sourceName(Source source)
{
    switch (source)
    {
    case Source::LONG_TERM: return "LONG_TERM";
    case Source::SHORT_TERM: return "SHORT_TERM";
    case Source::BOTH: return "LONG_TERM+SHORT_TERM";
    case Source::SESSION: return "SESSION";
    case Source::LONG_AND_SESSION: return "LONG_TERM+SESSION";
    case Source::SHORT_AND_SESSION: return "SESSION+SHORT_TERM";
    case Source::ALL: return "LONG_TERM+SESSION+SHORT_TERM";
    default: return "NONE";
    }
}

void Hold::update(bool condition, uint32_t now)
{
    if (!condition) active = false;
    else if (!active) { active = true; since = now; }
}

uint32_t Hold::elapsed(uint32_t now) const { return active ? now - since : 0; }
bool Hold::reached(uint32_t now, uint32_t duration) const
{
    return active && elapsed(now) >= duration;
}

float Monitor::deviation(float current, float reference)
{
    return reference > 0.0f ? 100.0f * (reference - current) / reference : 0.0f;
}

void Monitor::interruptContinuity()
{
    if (sampleBlock_.active) ++collectionBreaks_;
    sampleBlock_.active = false;
    recovery_.active = false;
    p1Long_.active = p1Short_.active = false;
    p2Long_.active = p2Short_.active = false;
    p1Session_.active = p2Session_.active = false;
}

uint32_t Monitor::motionRecoveryRemainingMs(uint32_t now) const
{
    (void)now; // only observed LOW intervals count, not time since a report
    if (!motionRecovery_) return 0;
    return POST_EXERCISE_RECOVERY_MS - recoveryLowMs_;
}

void Monitor::contactChanged()
{
    interruptContinuity();
    head_ = count_ = 0;
    shortReady_ = false;
    shortValue_ = 0.0f;
}

void Monitor::observeMotion(uint32_t now, Motion motion)
{
    if (motionRecovery_ && haveMotion_ &&
        previousMotion_ == Motion::LOW_MOTION && motion == Motion::LOW_MOTION)
    {
        uint32_t elapsed = now - lastMotionMs_;
        // Do not credit a gap with no motion observations.
        if (elapsed <= MAX_REPORT_GAP_MS)
        {
            uint32_t remaining = POST_EXERCISE_RECOVERY_MS - recoveryLowMs_;
            recoveryLowMs_ += elapsed < remaining ? elapsed : remaining;
        }
    }
    haveMotion_ = true;
    previousMotion_ = motion;
    lastMotionMs_ = now;

    if (motion == Motion::HIGH_MOTION)
    {
        if (motionRecoveryArmed_) motionRecovery_ = true;
        recoveryLowMs_ = 0;
    }
    else if (motion == Motion::LOW_MOTION && recoveryLowMs_ >= POST_EXERCISE_RECOVERY_MS)
        motionRecovery_ = false;

    // MODERATE pauses recovery without starting it or discarding LOW time.
    // Both movement levels still interrupt collection and P1/P2 holds.
    if (motion != Motion::LOW_MOTION) interruptContinuity();
}

void Monitor::expire(uint32_t now)
{
    while (count_ > 0)
    {
        unsigned oldest = (head_ + CAPACITY - count_) % CAPACITY;
        if (now - samples_[oldest].time < WINDOW_MS) break;
        --count_;
    }
}

void Monitor::calculateMedian()
{
    shortReady_ = count_ >= MIN_SAMPLES;
    if (!shortReady_) { shortValue_ = 0.0f; return; }
    float sorted[CAPACITY];
    for (unsigned i = 0; i < count_; ++i)
    {
        float value = samples_[(head_ + CAPACITY - count_ + i) % CAPACITY].value;
        unsigned j = i;
        while (j > 0 && sorted[j - 1] > value)
        {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
    shortValue_ = count_ % 2 ? sorted[count_ / 2]
                           : 0.5f * (sorted[count_ / 2 - 1] + sorted[count_ / 2]);
    shortReady_ = shortValue_ > 0.0f;
}

void Monitor::append(float value, uint32_t now)
{
    samples_[head_] = {value, now};
    head_ = (head_ + 1) % CAPACITY;
    if (count_ < CAPACITY) ++count_;
    calculateMedian();
}

static Source completed(const Hold &longHold, const Hold &shortHold, const Hold &sessionHold,
                        uint32_t now, uint32_t duration)
{
    unsigned mask = longHold.reached(now, duration) ? 1 : 0;
    if (shortHold.reached(now, duration)) mask |= 2;
    if (sessionHold.reached(now, duration)) mask |= 4;
    return static_cast<Source>(mask);
}

Events Monitor::update(const Input &in)
{
    Events events;
    observeMotion(in.now, in.motion);
    if (haveReport_ && in.now - lastReport_ > MAX_REPORT_GAP_MS) interruptContinuity();
    haveReport_ = true;
    lastReport_ = in.now;
    expire(in.now);
    calculateMedian();

    bool valid = in.signalGood && isfinite(in.current) && in.current >= 0.0f;
    bool qualifies = valid && in.motion == Motion::LOW_MOTION && !motionRecovery_;
    if (!valid) restingGate_ = CollectionGate::SIGNAL_OR_IMU;
    else if (in.motion != Motion::LOW_MOTION) restingGate_ = CollectionGate::MOTION;
    else if (motionRecovery_) restingGate_ = CollectionGate::POST_EXERCISE;
    else if (in.interventionActive) restingGate_ = CollectionGate::INTERVENTION;
    else if (!in.stable) restingGate_ = CollectionGate::UNSTABLE;
    else restingGate_ = CollectionGate::OPEN;

    // The live short reference continues collecting through pending triggers
    // and latched episodes. A changing RMSSD is not invalid data: only the
    // fixed resting baseline/rearm still require the stability gate.
    collectionGate_ = restingGate_ == CollectionGate::UNSTABLE
                          ? CollectionGate::OPEN : restingGate_;
    bool canCollect = collectionGate_ == CollectionGate::OPEN;
    if (!canCollect && sampleBlock_.active) ++collectionBreaks_;
    sampleBlock_.update(canCollect, in.now);
    if (sampleBlock_.reached(in.now, SAMPLE_MS))
    {
        append(in.current, in.now);
        motionRecoveryArmed_ = true;
        sampleBlock_.since = in.now;
    }

    bool longReady = in.longTermAvailable && isfinite(in.longTermBaseline) && in.longTermBaseline > 0;
    bool sessionReady = in.sessionAvailable && isfinite(in.sessionBaseline) && in.sessionBaseline > 0;
    // A newly saved personal baseline must not inherit another value's hold.
    float longValue = longReady ? in.longTermBaseline : 0;
    float sessionValue = sessionReady ? in.sessionBaseline : 0;
    if (longValue != lastLongBaseline_) p1Long_.active = p2Long_.active = false;
    if (sessionValue != lastSessionBaseline_) p1Session_.active = p2Session_.active = false;
    lastLongBaseline_ = longValue;
    lastSessionBaseline_ = sessionValue;
    float longDrop = deviation(in.current, longValue);
    float sessionDrop = deviation(in.current, sessionValue);
    float shortDrop = deviation(in.current, shortValue_);

    // These are overlapping conditions, not exclusive P1/P2 zones. Entering
    // or leaving P2 preserves P1's original start while its drop still qualifies.
    // With valid input, P1 resets only above its RMSSD boundary (drop < P1).
    p1Long_.update(qualifies && longReady && longDrop >= P1_DROP_PCT, in.now);
    p2Long_.update(qualifies && longReady && longDrop >= P2_DROP_PCT, in.now);
    p1Session_.update(qualifies && sessionReady && sessionDrop >= P1_DROP_PCT, in.now);
    p2Session_.update(qualifies && sessionReady && sessionDrop >= P2_DROP_PCT, in.now);
    p1Short_.update(qualifies && shortReady_ && shortDrop >= P1_DROP_PCT, in.now);
    p2Short_.update(qualifies && shortReady_ && shortDrop >= P2_DROP_PCT, in.now);
    if (!p1Fired_)
    {
        events.p1 = completed(p1Long_, p1Short_, p1Session_, in.now, P1_HOLD_MS);
        if (events.p1 != Source::NONE) { p1Fired_ = true; p1Source_ = events.p1; }
    }
    if (!p2Fired_)
    {
        events.p2 = completed(p2Long_, p2Short_, p2Session_, in.now, P2_HOLD_MS);
        if (events.p2 != Source::NONE) { p2Fired_ = true; p2Source_ = events.p2; }
    }
    bool recovered = (shortReady_ || sessionReady || longReady) &&
                     (!shortReady_ || shortDrop < 10) &&
                     (!sessionReady || sessionDrop < 10) &&
                     (!longReady || longDrop < 10);
    recovery_.update((p1Fired_ || p2Fired_) && qualifies && in.stable &&
                     recovered && !in.interventionActive, in.now);
    if (recovery_.reached(in.now, RECOVERY_HOLD_MS))
    {
        p1Fired_ = p2Fired_ = false;
        p1Source_ = p2Source_ = Source::NONE;
        recovery_.active = false;
        events.resumed = true;
    }
    return events;
}
} // namespace rmssd
