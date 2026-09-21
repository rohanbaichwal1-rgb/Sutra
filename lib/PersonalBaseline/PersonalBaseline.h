#pragma once

#include <RmssdReferences.h>
#include <stddef.h>
#include <stdint.h>

namespace personal
{
constexpr unsigned SESSION_SAMPLES = 20; // 100 qualifying seconds for prototype testing
constexpr unsigned HISTORY_SIZE = 7;
constexpr uint32_t SESSION_DURATION_MS = SESSION_SAMPLES * rmssd::SAMPLE_MS;

// One boot's resting measurement. Completed blocks accumulate across pauses;
// partial blocks and unobserved time never count. No rolling expiry here.
class Collector
{
public:
    void update(uint32_t now, float value, bool qualifies);
    void interrupt() { block_.active = false; }
    bool complete() const { return count_ == SESSION_SAMPLES; }
    unsigned sampleCount() const { return count_; }
    uint32_t validDurationMs() const { return count_ * rmssd::SAMPLE_MS; }
    float median() const;
private:
    float values_[SESSION_SAMPLES] = {};
    unsigned count_ = 0;
    rmssd::Hold block_;
    bool haveReport_ = false;
    uint32_t lastReport_ = 0;
};

// Fixed-size, versioned flash format. CRC covers everything before checksum.
struct Record
{
    uint32_t magic;
    uint32_t version;
    uint32_t generation;
    uint32_t count;
    uint32_t next;
    float values[HISTORY_SIZE];
    uint32_t checksum;
};
static_assert(sizeof(Record) == 52, "Persistent record layout changed");
enum class ReadResult { MISSING, FOUND, ERROR };

class Store
{
public:
    virtual ~Store() = default;
    virtual ReadResult read(unsigned slot, Record &record) = 0;
    virtual bool write(unsigned slot, const Record &record) = 0;
};

class History
{
public:
    bool load(Store &store); // once at boot; never clears damaged storage
    bool saveSession(float median); // at most one successful save per boot
    bool storageReady() const { return storageReady_; }
    bool savedThisBoot() const { return savedThisBoot_; }
    unsigned count() const { return record_.count; }
    bool ready() const { return storageReady_ && count() == HISTORY_SIZE; }
    float baseline() const;
private:
    Record record_ = {};
    Store *store_ = nullptr;
    int activeSlot_ = -1;
    bool storageReady_ = false;
    bool savedThisBoot_ = false;
    static uint32_t checksum(const Record &record);
    static bool valid(const Record &record);
};
} // namespace personal
