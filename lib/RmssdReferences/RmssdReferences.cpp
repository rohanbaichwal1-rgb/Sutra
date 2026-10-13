#include "RmssdReferences.h"
#include <math.h>

namespace rmssd
{
const char *followUpName(FollowUp state)
{
    switch (state)
    {
    case FollowUp::PENDING: return "PENDING";
    case FollowUp::ISSUED: return "ISSUED";
    case FollowUp::CANCELLED: return "CANCELLED";
    default: return "IDLE";
    }
}

void Monitor::cancelFollowUp()
{
    if (followUp_ != FollowUp::PENDING) return;
    followUp_ = FollowUp::CANCELLED;
    followUpCancellation_ = true;
}

void Monitor::armFollowUpCountdown(uint32_t now)
{
    // A follow-up belongs to one already-latched P1 episode and can be armed
    // once only, after its initial vibration has completed.
    if (!p1Fired_ || followUp_ != FollowUp::IDLE) return;
    followUp_ = FollowUp::PENDING;
    followUpCancellation_ = false;
    followUpSince_ = followUpLastCheck_ = now;
}

void Monitor::completeP1Intervention()
{
    if (p1Fired_)
        interventionComplete_ = true;
}

void Monitor::completeP1FollowUp()
{
    if (p1Fired_)
        resetP1Episode();
}

void Monitor::checkFollowUp(const Input &in)
{
    if (followUp_ != FollowUp::PENDING) return;
    // Once a P1 intervention owns the sequence, the breathing delay is not
    // contingent on later DEV/RMSSD, motion, or report continuity.  Only the
    // haptic layer may cancel it for a genuine execution/hardware failure.
    followUpLastCheck_ = in.now;
}

FollowUpEvent Monitor::pollFollowUp(const Input &in)
{
    checkFollowUp(in);
    if (followUpCancellation_)
    {
        followUpCancellation_ = false;
        return FollowUpEvent::CANCELLED;
    }
    if (followUp_ == FollowUp::PENDING && in.now - followUpSince_ >= P1_FOLLOW_UP_DELAY_MS)
    {
        followUp_ = FollowUp::ISSUED;
        return FollowUpEvent::START;
    }
    return FollowUpEvent::NONE;
}

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
    if (!condition) reset();
    else if (!active) { active = true; since = now; }
}

void Hold::updateP1(bool condition, bool blankDev, uint32_t now)
{
    static constexpr uint8_t maxBlankDev = P1_MAX_CONSECUTIVE_BLANK_DEV;
    if (condition)
    {
        blankCount = 0;
        if (!active) { active = true; since = now; }
        return;
    }
    if (blankDev && active && blankCount < maxBlankDev)
    {
        ++blankCount;
        return;
    }
    reset();
}

void Hold::reset()
{
    active = false;
    blankCount = 0;
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

void Monitor::interruptContinuity(bool resetP1)
{
    if (sampleBlock_.active) ++collectionBreaks_;
    sampleBlock_.active = false;
    recovery_.active = false;
    if (resetP1)
    {
        p1Long_.reset();
        p1Short_.reset();
        p1Session_.reset();
    }
}

void Monitor::interruptSignalContinuity()
{
    if (sampleBlock_.active) ++collectionBreaks_;
    sampleBlock_.active = false;
    recovery_.active = false;
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

    // MODERATE pauses recovery without starting it or discarding LOW time,
    // but P1 itself continues its normal persistence count. HIGH motion and
    // post-exercise recovery retain the existing reset behavior.
    if (motion == Motion::MODERATE_MOTION && !motionRecovery_)
        interruptContinuity(false);
    else if (motion == Motion::HIGH_MOTION ||
             (motion == Motion::MODERATE_MOTION && motionRecovery_))
        interruptContinuity();
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

bool Monitor::p1CountersReset() const
{
    return !p1Long_.active && !p1Short_.active && !p1Session_.active;
}

void Monitor::resetP1Episode()
{
    p1Fired_ = false;
    p1Source_ = Source::NONE;
    interventionComplete_ = false;
    followUp_ = FollowUp::IDLE;
    followUpCancellation_ = false;
    p1Long_.reset();
    p1Short_.reset();
    p1Session_.reset();
    recovery_.reset();
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
    if (longValue != lastLongBaseline_) p1Long_.reset();
    if (sessionValue != lastSessionBaseline_) p1Session_.reset();
    lastLongBaseline_ = longValue;
    lastSessionBaseline_ = sessionValue;
    float longDrop = deviation(in.current, longValue);
    float sessionDrop = deviation(in.current, sessionValue);
    float shortDrop = deviation(in.current, shortValue_);

    checkFollowUp(in);
    // A running P1 hold may bridge at most five consecutive blank DEV reports.
    // Blank DEV cannot start a hold; post-exercise state, report gaps, and
    // unavailable reference paths still reset immediately. LOW and MODERATE
    // motion both preserve normal P1 counting; HIGH remains disqualifying.
    bool p1GateOpen = in.motion != Motion::HIGH_MOTION && !motionRecovery_;
    bool blankDev = !valid && p1GateOpen;
    bool p1Qualifies = valid && p1GateOpen;
    p1Long_.updateP1(p1Qualifies && longReady && longDrop >= P1_DROP_PCT,
                      blankDev && longReady, in.now);
    p1Session_.updateP1(p1Qualifies && sessionReady && sessionDrop >= P1_DROP_PCT,
                         blankDev && sessionReady, in.now);
    p1Short_.updateP1(p1Qualifies && shortReady_ && shortDrop >= P1_DROP_PCT,
                       blankDev && shortReady_, in.now);

    // A completed intervention never remains latched merely because the old
    // recovery timer has not elapsed.  Once every P1 persistence counter has
    // reset, this is a new independent physiological episode.
    if (p1Fired_ && interventionComplete_ && p1CountersReset())
    {
        resetP1Episode();
        events.resumed = true;
    }

    if (!p1Fired_)
    {
        events.p1 = completed(p1Long_, p1Short_, p1Session_, in.now, P1_HOLD_MS);
        if (events.p1 != Source::NONE)
        {
            p1Fired_ = true;
            p1Source_ = events.p1;
            interventionComplete_ = false;
            // The runtime haptic layer starts the follow-up countdown only
            // after the five-second P1 pattern completes successfully.
            followUp_ = FollowUp::IDLE;
            followUpCancellation_ = false;
        }
    }
    // Retain the legacy recovery fallback only for callers that do not report
    // haptic completion.  The runtime haptic path uses the completed-
    // intervention + zero-counter transition above instead of WAIT_RECOVERY.
    bool recovered = (shortReady_ || sessionReady || longReady) &&
                     (!shortReady_ || shortDrop < 10) &&
                     (!sessionReady || sessionDrop < 10) &&
                     (!longReady || longDrop < 10);
    recovery_.update(p1Fired_ && !interventionComplete_ && qualifies && in.stable &&
                     recovered && !in.interventionActive, in.now);
    if (recovery_.reached(in.now, RECOVERY_HOLD_MS))
    {
        resetP1Episode();
        events.resumed = true;
    }
    return events;
}
} // namespace rmssd
