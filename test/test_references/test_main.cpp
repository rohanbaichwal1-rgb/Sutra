#include <unity.h>
#include <RmssdReferences.h>

using namespace rmssd;

struct Fixture
{
    Monitor monitor;
    Input input = {0, 100.0f, true, true, true, false, false, 100.0f};
    Events last;
    Fixture() { last = monitor.update(input); }
    Events tick()
    {
        input.now += 1000;
        last = monitor.update(input);
        return last;
    }
    void seconds(unsigned n) { while (n--) tick(); }
    void initialize() { seconds(300); }
};

void setUp() {}
void tearDown() {}

void test_requires_three_minutes_and_prefers_five()
{
    Fixture f;
    f.seconds(179);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    f.tick();
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT32(180000, f.monitor.validDurationMs());
    f.seconds(120);
    TEST_ASSERT_EQUAL_UINT(60, f.monitor.sampleCount());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, f.monitor.shortReference());
    f.seconds(100);
    TEST_ASSERT_EQUAL_UINT(60, f.monitor.sampleCount());
}

void test_median_uses_middle_pair_and_resists_outlier()
{
    Fixture f;
    f.seconds(90); // 18 readings at 100
    f.input.current = 120;
    f.seconds(90); // 18 readings at 120
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 110.0f, f.monitor.shortReference());
    f.input.current = 1000;
    f.seconds(5);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 120.0f, f.monitor.shortReference());
}

void test_partial_or_bad_quality_blocks_do_not_count()
{
    Fixture f;
    for (unsigned i = 0; i < 60; ++i)
    {
        f.input.signalGood = true;
        f.seconds(4);
        f.input.signalGood = false;
        f.tick();
    }
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    f.input.signalGood = true;
    f.input.stable = false;
    f.seconds(180);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
}

void test_old_readings_expire_in_wall_time()
{
    Fixture f;
    f.initialize();
    f.input.signalGood = false;
    f.seconds(125);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    f.seconds(175);
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
}

void test_preliminary_freeze_requires_ten_seconds_and_is_not_p1()
{
    Fixture f;
    f.initialize();
    f.input.current = 90;
    f.tick(); // start at 301 s
    f.seconds(9);
    TEST_ASSERT_FALSE(f.monitor.frozen());
    TEST_ASSERT_TRUE(f.monitor.preliminary());
    TEST_ASSERT_TRUE(f.tick().froze);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, f.monitor.shortReference());
    f.input.current = 100;
    f.input.stable = false;
    f.seconds(80);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    f.input.stable = true;
    f.tick();
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    TEST_ASSERT_TRUE(f.tick().resumed);
    TEST_ASSERT_FALSE(f.monitor.frozen());
}

void test_both_paths_trigger_at_30_and_120_seconds()
{
    Fixture f;
    f.initialize();
    f.input.sessionAvailable = true;
    f.input.current = 70;
    f.input.stable = false; // stability is required for adaptation, not triggers
    f.tick();
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::BOTH), static_cast<int>(f.tick().p1));
    f.seconds(89);
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::BOTH), static_cast<int>(f.tick().p2));
    f.seconds(200);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::NONE), static_cast<int>(f.last.p1));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::NONE), static_cast<int>(f.last.p2));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, f.monitor.shortReference());
}

void test_short_path_does_not_require_session_baseline()
{
    Fixture f;
    f.initialize();
    f.input.current = 80;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(f.last.p1));
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
}

void test_session_path_does_not_require_short_reference()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::LONG_TERM), static_cast<int>(f.last.p1));
    f.seconds(90);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::LONG_TERM), static_cast<int>(f.last.p2));
}

void test_source_timers_cannot_borrow_elapsed_time()
{
    Fixture f;
    f.initialize();
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 125;
    f.input.current = 100; // only the session path qualifies
    f.tick();
    f.seconds(20);
    f.input.sessionAvailable = false;
    f.input.current = 80; // only the short path now qualifies
    f.tick();
    f.seconds(10);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.seconds(20);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(f.last.p1));
}

void test_invalid_input_and_report_gaps_break_continuous_timers()
{
    Fixture f;
    f.initialize();
    f.input.current = 80;
    f.tick();
    f.seconds(20);
    f.input.signalGood = false;
    f.tick();
    f.input.signalGood = true;
    f.tick();
    f.seconds(20);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.input.now += 5000;
    f.monitor.update(f.input);
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(f.tick().p1));
}

void test_intervention_and_contact_loss_preserve_frozen_episode()
{
    Fixture f;
    f.initialize();
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    f.monitor.contactChanged();
    TEST_ASSERT_TRUE(f.monitor.frozen());
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    f.input.current = 100;
    f.input.interventionActive = true;
    f.seconds(90);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    f.input.interventionActive = false;
    f.tick();
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    TEST_ASSERT_TRUE(f.tick().resumed);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
}

void test_recovery_must_be_below_ten_percent_continuously()
{
    Fixture f;
    f.initialize();
    f.input.sessionAvailable = true;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    f.input.current = 90; // exactly 10% is not recovered
    f.seconds(70);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    f.input.current = 100;
    f.seconds(40);
    f.input.signalGood = false;
    f.tick();
    f.input.signalGood = true;
    f.tick();
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.frozen());
    TEST_ASSERT_TRUE(f.tick().resumed);
}

void test_motion_requires_two_minutes_of_continuous_low()
{
    Fixture f;
    f.monitor.observeMotion(f.input.now, false);
    f.tick(); // LOW recovery starts
    f.seconds(119);
    TEST_ASSERT_TRUE(f.monitor.postExerciseRecovery());
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    f.tick();
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
    f.seconds(5);
    TEST_ASSERT_EQUAL_UINT(1, f.monitor.sampleCount());
    f.input.lowMotion = false;
    f.tick();
    f.input.lowMotion = true;
    f.seconds(60);
    f.monitor.observeMotion(f.input.now, false);
    f.seconds(61);
    TEST_ASSERT_TRUE(f.monitor.postExerciseRecovery());
}

void test_timers_and_sample_expiry_survive_millis_rollover()
{
    Fixture f;
    f.input.now = UINT32_MAX - 100000;
    f.monitor.update(f.input);
    f.initialize();
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    f.input.now = UINT32_MAX - 15000;
    f.monitor.contactChanged();
    f.input.sessionAvailable = true;
    f.input.current = 80;
    f.monitor.update(f.input);
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::LONG_TERM), static_cast<int>(f.tick().p1));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_requires_three_minutes_and_prefers_five);
    RUN_TEST(test_median_uses_middle_pair_and_resists_outlier);
    RUN_TEST(test_partial_or_bad_quality_blocks_do_not_count);
    RUN_TEST(test_old_readings_expire_in_wall_time);
    RUN_TEST(test_preliminary_freeze_requires_ten_seconds_and_is_not_p1);
    RUN_TEST(test_both_paths_trigger_at_30_and_120_seconds);
    RUN_TEST(test_short_path_does_not_require_session_baseline);
    RUN_TEST(test_session_path_does_not_require_short_reference);
    RUN_TEST(test_source_timers_cannot_borrow_elapsed_time);
    RUN_TEST(test_invalid_input_and_report_gaps_break_continuous_timers);
    RUN_TEST(test_intervention_and_contact_loss_preserve_frozen_episode);
    RUN_TEST(test_recovery_must_be_below_ten_percent_continuously);
    RUN_TEST(test_motion_requires_two_minutes_of_continuous_low);
    RUN_TEST(test_timers_and_sample_expiry_survive_millis_rollover);
    return UNITY_END();
}
