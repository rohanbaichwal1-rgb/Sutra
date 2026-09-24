#include "HapticPatterns.h"

namespace haptics
{
uint32_t Patterns::randomBetween(uint32_t minimum, uint32_t maximum)
{
    // Seeded by the caller; ESP32 firmware can supply hardware randomness.
    randomState_ ^= randomState_ << 13;
    randomState_ ^= randomState_ >> 17;
    randomState_ ^= randomState_ << 5;
    return minimum + randomState_ % (maximum - minimum + 1);
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
    durationMs_ = 10000UL;
    protocol_ = Protocol::BOOT_TEST;
    frame_ = Frame{};
    for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor)
        frame_.level[motor] = 1000; // Array's existing 40% scaling applies.
    return true;
}

bool Patterns::startP1(uint32_t now, uint32_t seed, uint32_t durationMs)
{
    // A lower-priority P1 request cannot interrupt paced breathing.
    if (protocol_ == Protocol::P2_SWEEP || durationMs < MIN_BURST_MS ||
        durationMs > 0x7fffffffUL)
        return false;
    stop();
    startedMs_ = now;
    durationMs_ = durationMs;
    randomState_ = seed ? seed : 1;
    protocol_ = Protocol::P1_FLUTTER;
    for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor)
    {
        taps_[motor].on = false;
        taps_[motor].since = now;
        taps_[motor].duration = motor == 0 ? 0 :
            randomBetween(motor * 80UL, motor * 80UL + 70UL);
    }
    update(now);
    return true;
}

bool Patterns::startP2(uint32_t now, uint16_t cycles)
{
    if (cycles == 0) return false;
    stop();
    startedMs_ = now;
    durationMs_ = static_cast<uint32_t>(cycles) * BREATH_CYCLE_MS;
    protocol_ = Protocol::P2_SWEEP;
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
    else if (protocol_ == Protocol::P2_SWEEP) updateSweep(elapsed);
    return frame_;
}

void Patterns::updateFlutter(uint32_t now, uint32_t elapsed)
{
    for (unsigned motor = 0; motor < MOTOR_COUNT; ++motor)
    {
        Tap &tap = taps_[motor];
        if (now - tap.since < tap.duration) continue;
        tap.since = now;
        if (tap.on)
        {
            tap.on = false;
            frame_.level[motor] = 0;
            tap.duration = randomBetween(MIN_GAP_MS, MAX_GAP_MS);
        }
        else
        {
            uint32_t remaining = durationMs_ - elapsed;
            if (remaining < MIN_BURST_MS) continue;
            tap.on = true;
            frame_.level[motor] = 900;
            tap.duration = randomBetween(MIN_BURST_MS, MAX_BURST_MS);
            if (tap.duration > remaining) tap.duration = remaining;
        }
    }
}

void Patterns::updateSweep(uint32_t elapsed)
{
    const uint32_t phase = elapsed % BREATH_CYCLE_MS;
    frame_ = Frame{};
    if (phase < 1300UL)
        frame_.level[0] = 100 + 400UL * phase / 1300UL;
    else if (phase < 2600UL)
        frame_.level[2] = 100 + 400UL * (phase - 1300UL) / 1300UL;
    else if (phase < 4000UL)
        frame_.level[1] = 600;
    else if (phase < 6000UL)
        frame_.level[1] = 600 - 300UL * (phase - 4000UL) / 2000UL;
    else
        frame_.level[0] = frame_.level[2] =
            300 - 300UL * (phase - 6000UL) / 4000UL;
}
} // namespace haptics
