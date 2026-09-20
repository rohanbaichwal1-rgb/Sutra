#include "RmssdReferences.h"
#include <math.h>

namespace rmssd
{
const char *sourceName(Source source)
{
    switch (source)
    {
    case Source::LONG_TERM: return "LONG_TERM";
    case Source::SHORT_TERM: return "SHORT_TERM";
    case Source::BOTH: return "BOTH";
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
    sampleBlock_.active = false;
    preliminary_.active = false;
    recovery_.active = false;
    p1Long_.active = p1Short_.active = false;
    p2Long_.active = p2Short_.active = false;
}

void Monitor::contactChanged()
{
    interruptContinuity();
    head_ = count_ = 0;
    if (!frozen_) { shortReady_ = false; shortValue_ = 0.0f; }
}

void Monitor::observeMotion(uint32_t now, bool lowMotion)
{
    if (!lowMotion)
    {
        motionRecovery_ = true;
        lowMotion_.active = false;
        interruptContinuity();
    }
    else if (motionRecovery_)
    {
        lowMotion_.update(true, now);
        if (lowMotion_.reached(now, POST_EXERCISE_RECOVERY_MS))
            motionRecovery_ = false;
    }
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

void Monitor::freeze(Events &events)
{
    if (!frozen_ && shortReady_)
    {
        frozen_ = true;
        events.froze = true;
    }
}

static Source completed(const Hold &longHold, const Hold &shortHold,
                        bool frozen, uint32_t now, uint32_t duration)
{
    unsigned mask = longHold.reached(now, duration) ? 1 : 0;
    if (frozen && shortHold.reached(now, duration)) mask |= 2;
    return static_cast<Source>(mask);
}

Events Monitor::update(const Input &in)
{
    Events events;
    observeMotion(in.now, in.lowMotion);
    if (haveReport_ && in.now - lastReport_ > MAX_REPORT_GAP_MS)
        interruptContinuity();
    haveReport_ = true;
    lastReport_ = in.now;

    // The ring always ages in wall time. A frozen (or pending transition)
    // snapshot is separate and can survive a long intervention.
    expire(in.now);
    if (!frozen_ && !preliminary_.active) calculateMedian();

    bool valid = in.signalGood && isfinite(in.current) && in.current >= 0.0f;
    bool qualifies = valid && in.lowMotion && !motionRecovery_;
    bool sessionReady = in.sessionAvailable && isfinite(in.sessionBaseline) &&
                        in.sessionBaseline > 0.0f;
    float sessionDrop = deviation(in.current, in.sessionBaseline);
    float shortDrop = deviation(in.current, shortValue_);

    // Stop collecting immediately during the preliminary hold so the median
    // cannot follow the drop during the ten seconds before confirmed freezing.
    preliminary_.update(qualifies && shortReady_ && !frozen_ && shortDrop >= 10.0f, in.now);
    if (preliminary_.reached(in.now, FREEZE_HOLD_MS)) freeze(events);

    p1Long_.update(qualifies && sessionReady && sessionDrop >= 20.0f, in.now);
    p2Long_.update(qualifies && sessionReady && sessionDrop >= 30.0f, in.now);
    // Count the initial ten seconds against the same held reference, but only
    // allow a short-path trigger once the freeze has been confirmed.
    p1Short_.update(qualifies && shortReady_ && shortDrop >= 20.0f, in.now);
    p2Short_.update(qualifies && shortReady_ && shortDrop >= 30.0f, in.now);

    if (!p1Fired_)
    {
        events.p1 = completed(p1Long_, p1Short_, frozen_, in.now, P1_HOLD_MS);
        if (events.p1 != Source::NONE)
        {
            p1Fired_ = true;
            p1Source_ = events.p1;
            freeze(events);
        }
    }
    if (!p2Fired_)
    {
        events.p2 = completed(p2Long_, p2Short_, frozen_, in.now, P2_HOLD_MS);
        if (events.p2 != Source::NONE)
        {
            p2Fired_ = true;
            p2Source_ = events.p2;
            freeze(events);
        }
    }

    bool triggered = p1Fired_ || p2Fired_;
    bool recovering = frozen_ || triggered;
    // Before P1, recover against the short reference. After an intervention,
    // both available references must recover to avoid immediately re-arming
    // the still-depressed alternate path.
    bool recovered = (!shortReady_ || shortDrop < 10.0f) &&
                     (!triggered || !sessionReady || sessionDrop < 10.0f);
    bool pendingTrigger = p1Long_.active || p1Short_.active ||
                          p2Long_.active || p2Short_.active;
    recovery_.update(recovering && qualifies && in.stable && recovered &&
                     !pendingTrigger && !in.interventionActive, in.now);
    if (recovery_.reached(in.now, RECOVERY_HOLD_MS))
    {
        frozen_ = false;
        p1Fired_ = p2Fired_ = false;
        p1Source_ = p2Source_ = Source::NONE;
        interruptContinuity();
        calculateMedian(); // expired samples cannot initialize a new reference
        events.resumed = true;
    }

    bool canCollect = qualifies && in.stable && !frozen_ &&
                      !preliminary_.active && !p1Fired_ && !p2Fired_ &&
                      !pendingTrigger && !in.interventionActive;
    sampleBlock_.update(canCollect, in.now);
    if (sampleBlock_.reached(in.now, SAMPLE_MS))
    {
        append(in.current, in.now);
        sampleBlock_.since = in.now;
    }
    return events;
}
} // namespace rmssd
