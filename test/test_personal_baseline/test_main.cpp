#include <unity.h>
#include <PersonalBaseline.h>
#include <math.h>

using namespace personal;
void setUp() {}
void tearDown() {}

class MemoryStore : public Store
{
public:
    Record records[2] = {};
    bool present[2] = {false, false};
    bool failWrite = false;
    bool corruptWrite = false;
    bool failRead = false;
    unsigned writes = 0;
    ReadResult read(unsigned slot, Record &r) override
    {
        if (failRead) return ReadResult::ERROR;
        if (!present[slot]) return ReadResult::MISSING;
        r = records[slot];
        return ReadResult::FOUND;
    }
    bool write(unsigned slot, const Record &r) override
    {
        ++writes;
        if (failWrite) return false;
        records[slot] = r;
        present[slot] = true;
        if (corruptWrite) records[slot].checksum ^= 1;
        return true;
    }
};

void addBoot(MemoryStore &store, float value)
{
    History boot;
    TEST_ASSERT_TRUE(boot.load(store));
    TEST_ASSERT_TRUE(boot.saveSession(value));
}

void test_session_requires_100_seconds_and_uses_median()
{
    Collector c;
    for (uint32_t t = 0; t < 100000; t += 1000)
        c.update(t, t <= 50000 ? 50.0f : 70.0f, true);
    TEST_ASSERT_FALSE(c.complete());
    TEST_ASSERT_EQUAL_UINT32(95000, c.validDurationMs());
    c.update(100000, 70, true);
    TEST_ASSERT_TRUE(c.complete());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 60, c.median());
    c.update(400000, 999, true);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 60, c.median());
}

void test_session_pauses_without_expiring_completed_blocks()
{
    Collector c;
    for (uint32_t t = 0; t <= 6000; t += 1000) c.update(t, 50, true);
    c.interrupt(); // a failure between serial reports discards only the partial block
    for (uint32_t t = 7000; t <= 10000; t += 1000) c.update(t, 50, true);
    TEST_ASSERT_EQUAL_UINT32(5000, c.validDurationMs());
    c.update(400000, 50, true); // a report gap must not credit the unobserved time
    TEST_ASSERT_EQUAL_UINT32(5000, c.validDurationMs());
    for (uint32_t t = 401000; t <= 405000; t += 1000) c.update(t, 50, true);
    TEST_ASSERT_EQUAL_UINT32(10000, c.validDurationMs());
    c.update(406000, NAN, true);
    c.update(407000, 0, true);
    c.update(408000, -1, true);
    TEST_ASSERT_EQUAL_UINT32(10000, c.validDurationMs());
}

void test_session_timer_survives_rollover_and_power_loss_discards_incomplete_session()
{
    Collector c;
    for (uint32_t dt = 0; dt <= 5000; dt += 1000)
        c.update(UINT32_MAX - 2000 + dt, 80, true);
    TEST_ASSERT_EQUAL_UINT32(5000, c.validDurationMs());
    Collector rebooted;
    TEST_ASSERT_EQUAL_UINT32(0, rebooted.validDurationMs());
    TEST_ASSERT_FALSE(rebooted.complete());
}

void test_seven_boots_required_and_one_save_per_boot()
{
    MemoryStore store;
    const float sessions[] = {80, 60, 70, 90, 1000, 50, 75};
    for (unsigned i = 0; i < 7; ++i)
    {
        History boot;
        TEST_ASSERT_TRUE(boot.load(store));
        TEST_ASSERT_EQUAL_UINT(i, boot.count());
        TEST_ASSERT_FALSE(boot.ready());
        TEST_ASSERT_TRUE(boot.saveSession(sessions[i]));
        TEST_ASSERT_EQUAL_UINT(i + 1, boot.count());
        TEST_ASSERT_EQUAL(i == 6, boot.ready());
        TEST_ASSERT_FALSE(boot.saveSession(10));
        TEST_ASSERT_EQUAL_UINT(i + 1, store.writes);
    }
    History rebooted;
    TEST_ASSERT_TRUE(rebooted.load(store));
    TEST_ASSERT_TRUE(rebooted.ready());
    TEST_ASSERT_FALSE(rebooted.savedThisBoot());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 75, rebooted.baseline());
}

void test_eighth_session_replaces_oldest()
{
    MemoryStore store;
    for (unsigned i = 1; i <= 7; ++i) addBoot(store, i * 10.0f);
    addBoot(store, 80);
    History restored;
    TEST_ASSERT_TRUE(restored.load(store));
    TEST_ASSERT_EQUAL_UINT(7, restored.count());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50, restored.baseline());
}

void test_save_failure_does_not_enable_long_term_and_retry_counts_once()
{
    MemoryStore store;
    for (unsigned i = 0; i < 6; ++i) addBoot(store, 80);
    History boot;
    boot.load(store);
    store.failWrite = true;
    TEST_ASSERT_FALSE(boot.saveSession(90));
    TEST_ASSERT_FALSE(boot.ready());
    TEST_ASSERT_EQUAL_UINT(6, boot.count());
    store.failWrite = false;
    TEST_ASSERT_TRUE(boot.saveSession(90));
    TEST_ASSERT_TRUE(boot.ready());
    TEST_ASSERT_FALSE(boot.saveSession(90));
    History restored;
    restored.load(store);
    TEST_ASSERT_EQUAL_UINT(7, restored.count());
}

void test_torn_write_falls_back_to_previous_record()
{
    MemoryStore store;
    addBoot(store, 70);
    History boot;
    boot.load(store);
    store.corruptWrite = true;
    TEST_ASSERT_FALSE(boot.saveSession(80));
    TEST_ASSERT_EQUAL_UINT(1, boot.count());
    History rebooted;
    TEST_ASSERT_TRUE(rebooted.load(store));
    TEST_ASSERT_EQUAL_UINT(1, rebooted.count());
    store.corruptWrite = false;
    TEST_ASSERT_TRUE(rebooted.saveSession(90));
    TEST_ASSERT_EQUAL_UINT(2, rebooted.count());
}

void test_corrupt_or_unreadable_history_is_not_silently_overwritten()
{
    MemoryStore store;
    addBoot(store, 70);
    store.records[0].values[0] = NAN;
    History damaged;
    TEST_ASSERT_FALSE(damaged.load(store));
    TEST_ASSERT_FALSE(damaged.ready());
    TEST_ASSERT_FALSE(damaged.saveSession(80));
    TEST_ASSERT_EQUAL_UINT(1, store.writes);
    MemoryStore unreadable;
    unreadable.failRead = true;
    History error;
    TEST_ASSERT_FALSE(error.load(unreadable));
    TEST_ASSERT_FALSE(error.saveSession(80));
    TEST_ASSERT_EQUAL_UINT(0, unreadable.writes);
}

void test_personal_path_waits_for_seven_while_short_works_without_session()
{
    MemoryStore store;
    for (unsigned i = 0; i < 6; ++i) addBoot(store, 100);
    History h;
    h.load(store);
    rmssd::Monitor monitor;
    rmssd::Input in = {0, 100, true, rmssd::Motion::LOW_MOTION, true, false, h.ready(), h.baseline()};
    for (unsigned t = 0; t <= 300000; t += 1000)
    {
        in.now = t;
        monitor.update(in);
    }
    TEST_ASSERT_TRUE(monitor.shortAvailable());
    in.current = 70;
    rmssd::Events events;
    for (unsigned t = 301000; t <= 421000; t += 1000)
    {
        in.now = t;
        events = monitor.update(in);
        if (t == 331000)
            TEST_ASSERT_EQUAL_INT(static_cast<int>(rmssd::Source::SHORT_TERM), static_cast<int>(events.p1));
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(rmssd::Source::SHORT_TERM), static_cast<int>(events.p2));
    TEST_ASSERT_EQUAL_UINT32(0, monitor.p1LongHoldMs(in.now));
    TEST_ASSERT_TRUE(h.saveSession(100));
    rmssd::Monitor nextBoot;
    in.current = 100;
    in.longTermAvailable = h.ready();
    in.longTermBaseline = h.baseline();
    for (unsigned t = 0; t <= 300000; t += 1000)
    {
        in.now = t;
        nextBoot.update(in);
    }
    in.current = 70;
    for (unsigned t = 301000; t <= 421000; t += 1000)
    {
        in.now = t;
        events = nextBoot.update(in);
        if (t == 331000)
            TEST_ASSERT_EQUAL_INT(static_cast<int>(rmssd::Source::BOTH), static_cast<int>(events.p1));
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(rmssd::Source::BOTH), static_cast<int>(events.p2));
}

void test_session_freezes_and_saves_its_own_median_while_short_adapts()
{
    MemoryStore store;
    History history;
    TEST_ASSERT_TRUE(history.load(store));
    Collector session;
    rmssd::Monitor monitor;
    rmssd::Input in = {0, 100, true, rmssd::Motion::LOW_MOTION, true, false, false, 0};
    for (uint32_t t = 0; t <= 300000; t += 1000)
    {
        in.now = t;
        in.current = t <= 30000 ? 100 : 70;
        monitor.update(in);
        session.update(t, in.current, monitor.restingGate() == rmssd::CollectionGate::OPEN);
    }
    TEST_ASSERT_TRUE(session.complete());
    TEST_ASSERT_EQUAL_UINT(20, session.sampleCount());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, session.median());
    TEST_ASSERT_TRUE(history.saveSession(session.median()));
    for (uint32_t t = 301000; t <= 700000; t += 1000)
    {
        in.now = t;
        in.current = 130;
        monitor.update(in);
        session.update(t, in.current, monitor.restingGate() == rmssd::CollectionGate::OPEN);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 130, monitor.shortReference());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, session.median());
    TEST_ASSERT_FALSE(history.saveSession(session.median()));
    TEST_ASSERT_EQUAL_UINT(1, store.writes);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, store.records[0].values[0]);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_session_requires_100_seconds_and_uses_median);
    RUN_TEST(test_session_pauses_without_expiring_completed_blocks);
    RUN_TEST(test_session_timer_survives_rollover_and_power_loss_discards_incomplete_session);
    RUN_TEST(test_seven_boots_required_and_one_save_per_boot);
    RUN_TEST(test_eighth_session_replaces_oldest);
    RUN_TEST(test_save_failure_does_not_enable_long_term_and_retry_counts_once);
    RUN_TEST(test_torn_write_falls_back_to_previous_record);
    RUN_TEST(test_corrupt_or_unreadable_history_is_not_silently_overwritten);
    RUN_TEST(test_personal_path_waits_for_seven_while_short_works_without_session);
    RUN_TEST(test_session_freezes_and_saves_its_own_median_while_short_adapts);
    return UNITY_END();
}
