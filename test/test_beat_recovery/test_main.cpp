#include <unity.h>
#include <BeatRecovery.h>

void setUp() {}
void tearDown() {}

void test_rejected_candidates_do_not_refresh_hr()
{
    beats::Recovery r;
    r.observe(100);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, r.acceptedAge(100));
    r.accept(100);
    for (uint32_t t = 850; t < 3100; t += 750) {
        TEST_ASSERT_FALSE(r.observe(t));
        TEST_ASSERT_TRUE(r.fresh(t));
    }
    TEST_ASSERT_TRUE(r.observe(3100));
    TEST_ASSERT_FALSE(r.fresh(3100));
    TEST_ASSERT_EQUAL_UINT32(3000, r.acceptedAge(3100));
    for (unsigned i = 0; i < beats::Recovery::REQUIRED; ++i)
        TEST_ASSERT_EQUAL_UINT32(750, r.interval(i));
    r.clearCandidates();
    TEST_ASSERT_FALSE(r.fresh(3101));
    TEST_ASSERT_FALSE(r.observe(3850));
    r.accept(3850);
    TEST_ASSERT_TRUE(r.fresh(3850));
    TEST_ASSERT_EQUAL_UINT32(0, r.acceptedAge(3850));
}

void test_isolated_and_inconsistent_candidates_cannot_reacquire()
{
    beats::Recovery r;
    r.observe(0); r.accept(0);
    TEST_ASSERT_FALSE(r.observe(5000)); // long dropout is not an interval
    uint32_t now = 5000;
    for (unsigned i = 0; i < 12; ++i) {
        now += i % 2 ? 750 : 350;
        TEST_ASSERT_FALSE(r.observe(now));
    }
    TEST_ASSERT_FALSE(r.fresh(now));
}

void test_regular_acceptance_prevents_notch_reacquisition()
{
    beats::Recovery r;
    r.observe(0); r.accept(0);
    for (uint32_t t = 750; t <= 15000; t += 750) {
        TEST_ASSERT_FALSE(r.observe(t - 400)); // rejected secondary crest
        TEST_ASSERT_FALSE(r.observe(t));
        r.accept(t);
        TEST_ASSERT_TRUE(r.fresh(t));
    }
}

void test_faster_rhythm_requires_staleness_and_consistency()
{
    beats::Recovery r;
    r.observe(0); r.accept(0);
    for (uint32_t t = 400; t < 3200; t += 400)
        TEST_ASSERT_FALSE(r.observe(t));
    TEST_ASSERT_TRUE(r.observe(3200));
    TEST_ASSERT_EQUAL_UINT32(400, r.interval(0));
}

void test_startup_reset_and_rollover()
{
    beats::Recovery r;
    for (uint32_t t = 0; t < 6000; t += 750)
        TEST_ASSERT_FALSE(r.observe(t)); // no previously acquired rhythm
    TEST_ASSERT_FALSE(r.fresh(6000));
    r.reset();
    const uint32_t start = UINT32_MAX - 1000;
    r.observe(start); r.accept(start);
    for (unsigned i = 1; i < 4; ++i)
        TEST_ASSERT_FALSE(r.observe(start + i * 750UL));
    TEST_ASSERT_TRUE(r.observe(start + 3000UL));
    TEST_ASSERT_FALSE(r.fresh(start + 3000UL));
    r.reset();
    TEST_ASSERT_FALSE(r.fresh(start + 3001UL));
    TEST_ASSERT_FALSE(r.observe(start + 3750UL));
}

void test_reconstructed_acquisition_is_not_fresh_but_can_recover()
{
    beats::Recovery r;
    r.observe(1000);
    r.seedRhythm(1000);
    TEST_ASSERT_FALSE(r.fresh(1000));
    for (uint32_t t = 1750; t < 4000; t += 750)
        TEST_ASSERT_FALSE(r.observe(t));
    TEST_ASSERT_TRUE(r.observe(4000));
    TEST_ASSERT_FALSE(r.fresh(4000));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_reconstructed_acquisition_is_not_fresh_but_can_recover);
    RUN_TEST(test_rejected_candidates_do_not_refresh_hr);
    RUN_TEST(test_isolated_and_inconsistent_candidates_cannot_reacquire);
    RUN_TEST(test_regular_acceptance_prevents_notch_reacquisition);
    RUN_TEST(test_faster_rhythm_requires_staleness_and_consistency);
    RUN_TEST(test_startup_reset_and_rollover);
    return UNITY_END();
}
