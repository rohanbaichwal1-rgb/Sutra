#include "PersonalBaseline.h"
#include <math.h>
#include <string.h>

namespace personal
{
static float medianOf(const float *values, unsigned count)
{
    if (count == 0 || count > SESSION_SAMPLES) return 0;
    float sorted[SESSION_SAMPLES];
    for (unsigned i = 0; i < count; ++i)
    {
        unsigned j = i;
        while (j > 0 && sorted[j - 1] > values[i])
        {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = values[i];
    }
    return count % 2 ? sorted[count / 2]
        : sorted[count / 2 - 1] * 0.5f + sorted[count / 2] * 0.5f;
}

void Collector::update(uint32_t now, float value, bool qualifies)
{
    if (complete()) return;
    if (haveReport_ && now - lastReport_ > rmssd::MAX_REPORT_GAP_MS) interrupt();
    haveReport_ = true;
    lastReport_ = now;
    block_.update(qualifies && isfinite(value) && value > 0.0f, now);
    if (block_.reached(now, rmssd::SAMPLE_MS))
    {
        values_[count_++] = value;
        block_.since = now;
    }
}

float Collector::median() const
{
    return complete() ? medianOf(values_, count_) : 0.0f;
}

uint32_t History::checksum(const Record &record)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
    uint32_t crc = 0xffffffffUL;
    for (size_t i = 0; i < offsetof(Record, checksum); ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320UL : 0);
    }
    return ~crc;
}

bool History::valid(const Record &r)
{
    if (r.magic != 0x53555452UL || r.version != 1 ||
        r.count == 0 || r.count > HISTORY_SIZE || r.next >= HISTORY_SIZE ||
        (r.count < HISTORY_SIZE && r.next != r.count) ||
        r.checksum != checksum(r)) return false;
    for (unsigned i = 0; i < r.count; ++i)
        if (!isfinite(r.values[i]) || r.values[i] <= 0.0f) return false;
    return true;
}

bool History::load(Store &store)
{
    store_ = &store;
    record_ = Record{};
    activeSlot_ = -1;
    savedThisBoot_ = storageReady_ = false;
    bool bothMissing = true;
    for (unsigned slot = 0; slot < 2; ++slot)
    {
        Record candidate = {};
        ReadResult result = store.read(slot, candidate);
        if (result != ReadResult::MISSING) bothMissing = false;
        if (result != ReadResult::FOUND || !valid(candidate)) continue;
        // Generation ordering also handles the uint32_t wrap boundary.
        if (activeSlot_ < 0 ||
            (candidate.generation != record_.generation &&
             candidate.generation - record_.generation < 0x80000000UL))
        {
            record_ = candidate;
            activeSlot_ = static_cast<int>(slot);
        }
    }
    storageReady_ = activeSlot_ >= 0 || bothMissing;
    return storageReady_;
}

bool History::saveSession(float value)
{
    if (!storageReady_ || savedThisBoot_ || !isfinite(value) || value <= 0.0f)
        return false;
    Record candidate = record_;
    candidate.magic = 0x53555452UL;
    candidate.version = 1;
    ++candidate.generation;
    candidate.values[candidate.next] = value;
    candidate.next = (candidate.next + 1) % HISTORY_SIZE;
    if (candidate.count < HISTORY_SIZE) ++candidate.count;
    candidate.checksum = checksum(candidate);
    unsigned slot = activeSlot_ == 0 ? 1 : 0;
    if (!store_->write(slot, candidate)) return false;
    Record verified = {};
    if (store_->read(slot, verified) != ReadResult::FOUND || !valid(verified) ||
        memcmp(&verified, &candidate, sizeof(Record)) != 0) return false;
    record_ = verified;
    activeSlot_ = static_cast<int>(slot);
    savedThisBoot_ = true;
    return true;
}

float History::baseline() const
{
    return ready() ? medianOf(record_.values, HISTORY_SIZE) : 0.0f;
}
} // namespace personal
