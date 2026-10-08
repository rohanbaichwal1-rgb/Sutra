#include "HapticPatterns.h"

namespace haptics
{
namespace
{
// patternTest.ino passes values directly to DA7280 setVibrate(), whose DRO
// range is 0..127. Frames use thousandths of that full driver range.
uint16_t driverValueToLevel(uint8_t value)
{
    return static_cast<uint16_t>((static_cast<uint32_t>(value) * 1000UL + 63UL) / 127UL);
}
} // namespace

uint32_t Patterns::randomBetween(uint32_t minimum, uint32_t maximum)
{
    // Seeded by the caller; ESP32 firmware can supply hardware randomness.
    randomState_ ^= randomState_ << 13;
    randomState_ ^= randomState_ >> 17;
    randomState_ ^= randomState_ << 5;
    return minimum + randomState_ % (maximum - minimum + 1);
}

void Patterns::generateP1Sequence()
{
    for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor)
        p1Sequence_[motor] = motor;

    // Fisher-Yates shuffle: every three 200 ms slots use each motor exactly
    // once, in a new random order, matching the supplied p1Pattern.ino.
    for (unsigned motor = MOTOR_COUNT - 1; motor > 0; --motor)
    {
        const unsigned chosen = randomBetween(0, motor);
        const uint8_t saved = p1Sequence_[motor];
        p1Sequence_[motor] = p1Sequence_[chosen];
        p1Sequence_[chosen] = saved;
    }
}

void Patterns::stop()
{
    protocol_ = Protocol::IDLE;
    frame_ = Frame{};
}

uint32_t Patterns::elapsedMs(uint32_t now) const
{
    return active() ? now - startedMs_ : 0;
}

bool Patterns::startBootTest(uint32_t now)
{
    if (active()) return false;
    startedMs_ = now;
    durationMs_ = 3000UL;
    protocol_ = Protocol::BOOT_TEST;
    frame_ = Frame{};
    for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor)
        frame_.level[motor] = 1000; // Array's existing 40% scaling applies.
    return true;
}

bool Patterns::startP1(uint32_t now, uint32_t seed, uint32_t durationMs)
{
    // A lower-priority P1 request cannot interrupt paced breathing.
    if (protocol_ == Protocol::P1_FOLLOW_UP || durationMs < MIN_BURST_MS ||
        durationMs > 0x7fffffffUL)
        return false;
    stop();
    startedMs_ = now;
    durationMs_ = durationMs;
    randomState_ = seed ? seed : 1;
    protocol_ = Protocol::P1_FLUTTER;
    p1Slot_ = UINT32_MAX;
    generateP1Sequence();
    update(now);
    return true;
}

bool Patterns::startFollowUp(uint32_t now, uint16_t cycles)
{
    if (cycles == 0) return false;
    stop();
    startedMs_ = now;
    durationMs_ = static_cast<uint32_t>(cycles) * BREATH_CYCLE_MS;
    protocol_ = Protocol::P1_FOLLOW_UP;
    update(now);
    return true;
}

const Frame &Patterns::update(uint32_t now)
{
    if (!active()) return frame_;
    uint32_t elapsed = now - startedMs_;
    if (elapsed >= durationMs_)
    {
        stop();
        return frame_;
    }
    if (protocol_ == Protocol::P1_FLUTTER) updateFlutter(now, elapsed);
    else if (protocol_ == Protocol::P1_FOLLOW_UP) updateSweep(elapsed);
    return frame_;
}

void Patterns::updateFlutter(uint32_t now, uint32_t elapsed)
{
    (void)now;
    const uint32_t slot = elapsed / P1_MOTOR_ON_MS;
    if (slot == p1Slot_) return;

    const unsigned sequenceIndex = slot % MOTOR_COUNT;
    if (sequenceIndex == 0 && slot != 0)
        generateP1Sequence();

    // p1Pattern.ino: one motor is on for each 200 ms slot at direct DRO
    // value 60, with no intentional gap between randomized three-motor sets.
    frame_ = Frame{};
    frame_.level[p1Sequence_[sequenceIndex]] = driverValueToLevel(60);
    p1Slot_ = slot;
}

void Patterns::updateSweep(uint32_t elapsed)
{
    const uint32_t phase = elapsed % BREATH_CYCLE_MS;
    frame_ = Frame{};
    // Nonblocking translation of patternTest.ino's 12-second sequence.
    // Each value is the same intended DA7280 DRO value as the reference.
    if (phase < 3000UL)
        frame_.level[0] = frame_.level[2] =
            driverValueToLevel(5U + 40U * phase / 3000UL); // M1 + M3: 5 -> 45
    else if (phase < 4000UL)
        frame_.level[1] = driverValueToLevel(50); // M2: 50 hold
    else if (phase < 5000UL)
        ;                                   // All motors off
    else if (phase < 7000UL)
        frame_.level[1] =
            driverValueToLevel(50U - 25U * (phase - 5000UL) / 2000UL); // M2: 50 -> 25
    else
        frame_.level[0] = frame_.level[2] =
            driverValueToLevel(25U - 25U * (phase - 7000UL) / 5000UL); // M1 + M3: 25 -> 0
}
} // namespace haptics
