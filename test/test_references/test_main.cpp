#include <unity.h>
#include <RmssdReferences.h>

using namespace rmssd;

struct Fixture
{
    Monitor monitor;
    Input input = {0, 100.0f, true, Motion::LOW_MOTION, true, false, false, 100.0f};
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

void test_requires_one_minute_and_prefers_five()
{
    Fixture f;
    f.seconds(59);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    f.tick();
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT32(60000, f.monitor.validDurationMs());
    f.seconds(240);
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
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
    TEST_ASSERT_EQUAL_STRING("RMSSD_UNSTABLE", collectionGateName(f.monitor.restingGate()));
}

void test_old_readings_expire_in_wall_time()
{
    Fixture f;
    f.initialize();
    f.input.signalGood = false;
    f.seconds(240);
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT(12, f.monitor.sampleCount());
    f.seconds(5);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    f.seconds(55);
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
}

void test_ten_percent_drop_triggers_p1_at_the_configured_five_percent_threshold()
{
    Fixture f;
    f.initialize();
    f.input.current = 90;
    f.tick();
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.tick();
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
    TEST_ASSERT_EQUAL_UINT32(300000, f.monitor.validDurationMs());
    // The 5-minute median remains at its former value during this initial
    // 30-second drop, so the trigger cannot be attributed to a changed reference.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100, f.monitor.shortReference());
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
}

void test_both_paths_trigger_at_30_and_120_seconds()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = true;
    f.input.current = 70;
    f.input.stable = false; // changing RMSSD still permits short adaptation
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
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, f.monitor.shortReference());
}

void test_short_path_does_not_require_long_term_baseline()
{
    Fixture f;
    f.initialize();
    f.input.current = 80;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(f.last.p1));
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
}

void test_long_term_path_does_not_require_short_reference()
{
    Fixture f;
    f.input.longTermAvailable = true;
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
    f.input.longTermAvailable = true;
    f.input.longTermBaseline = 125;
    f.input.current = 100; // only the session path qualifies
    f.tick();
    f.seconds(20);
    f.input.longTermAvailable = false;
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

void test_intervention_and_contact_loss_preserve_episode_latches()
{
    Fixture f;
    f.initialize();
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    f.monitor.contactChanged();
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    f.input.current = 100;
    f.input.interventionActive = true;
    f.seconds(90);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    f.input.interventionActive = false;
    f.tick();
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.tick().resumed);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
}

void test_recovery_must_be_below_ten_percent_continuously()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = true;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    f.input.current = 90; // exactly 10% is not recovered
    f.seconds(70);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    f.input.current = 100;
    f.seconds(40);
    f.input.signalGood = false;
    f.tick();
    f.input.signalGood = true;
    f.tick();
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.tick().resumed);
}

void test_motion_requires_one_minute_of_continuous_low()
{
    Fixture f;
    f.seconds(5); // first qualifying resting block arms exercise recovery
    f.monitor.observeMotion(f.input.now, Motion::HIGH_MOTION);
    f.tick(); // LOW recovery starts
    f.seconds(59);
    TEST_ASSERT_TRUE(f.monitor.postExerciseRecovery());
    TEST_ASSERT_EQUAL_UINT(1, f.monitor.sampleCount());
    f.tick();
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
    f.seconds(5);
    TEST_ASSERT_EQUAL_UINT(2, f.monitor.sampleCount());
    f.input.motion = Motion::HIGH_MOTION;
    f.tick();
    f.input.motion = Motion::LOW_MOTION;
    f.seconds(30);
    f.monitor.observeMotion(f.input.now, Motion::HIGH_MOTION);
    f.seconds(31);
    TEST_ASSERT_TRUE(f.monitor.postExerciseRecovery());
    f.seconds(30);
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
}

void test_startup_motion_does_not_start_recovery_before_resting_data()
{
    Monitor monitor;
    Input input = {0, 100.0f, false, Motion::LOW_MOTION, false, false, false, 0.0f};
    monitor.update(input); // boot-time LOW default, with no valid signal
    for (uint32_t t = 1000; t <= 20000; t += 1000)
    {
        monitor.observeMotion(t, Motion::HIGH_MOTION); // initial sensor placement
        input.now = t;
        input.motion = Motion::HIGH_MOTION;
        monitor.update(input);
        TEST_ASSERT_FALSE(monitor.postExerciseRecovery());
        TEST_ASSERT_EQUAL_UINT(0, monitor.sampleCount());
    }
    input.signalGood = input.stable = true;
    input.motion = Motion::LOW_MOTION;
    input.now = 21000;
    monitor.update(input);
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(monitor.collectionGate()));
    for (unsigned i = 0; i < 60; ++i)
    {
        input.now += 1000;
        monitor.update(input);
    }
    TEST_ASSERT_TRUE(monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT32(60000, monitor.validDurationMs());
    // Once armed, losing contact must not bypass a real recovery wait.
    monitor.observeMotion(++input.now, Motion::HIGH_MOTION);
    TEST_ASSERT_TRUE(monitor.postExerciseRecovery());
    monitor.contactChanged();
    TEST_ASSERT_TRUE(monitor.postExerciseRecovery());
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
    f.input.longTermAvailable = true;
    f.input.current = 80;
    f.monitor.update(f.input);
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::LONG_TERM), static_cast<int>(f.tick().p1));
}

void test_long_wait_reports_intervention_block_and_collection_resumes()
{
    Fixture f;
    f.input.interventionActive = true;
    f.seconds(2000);
    TEST_ASSERT_FALSE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    TEST_ASSERT_EQUAL_STRING("HAPTIC_ACTIVE_OR_STOP_PENDING",
                            collectionGateName(f.monitor.collectionGate()));
    f.input.interventionActive = false;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
    f.seconds(60);
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT32(60000, f.monitor.validDurationMs());
}

void test_between_report_interruptions_are_visible_even_when_gate_is_open()
{
    Fixture f;
    for (unsigned i = 0; i < 400; ++i)
    {
        f.seconds(4);
        // Model a brief signal/IMU failure that recovers before the report.
        f.monitor.interruptContinuity();
        f.tick();
    }
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
    TEST_ASSERT_EQUAL_UINT32(400, f.monitor.collectionBreaks());
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.sampleBlockMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT(0, f.monitor.sampleCount());
    f.seconds(180);
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
}

void test_collection_diagnostics_show_quality_and_motion_recovery()
{
    Fixture f;
    f.seconds(3);
    TEST_ASSERT_EQUAL_UINT32(3000, f.monitor.sampleBlockMs(f.input.now));
    f.input.stable = false;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
    TEST_ASSERT_EQUAL_STRING("RMSSD_UNSTABLE", collectionGateName(f.monitor.restingGate()));
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.collectionBreaks());
    f.input.signalGood = false;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("SIGNAL_OR_IMU", collectionGateName(f.monitor.collectionGate()));
    f.input.signalGood = f.input.stable = true;
    f.seconds(6); // establish a resting block before later motion
    f.input.motion = Motion::HIGH_MOTION;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("MOTION", collectionGateName(f.monitor.collectionGate()));
    f.input.motion = Motion::LOW_MOTION;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("POST_EXERCISE", collectionGateName(f.monitor.collectionGate()));
    TEST_ASSERT_EQUAL_UINT32(60000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.seconds(59);
    TEST_ASSERT_EQUAL_UINT32(1000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.tick();
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.motionRecoveryRemainingMs(f.input.now));
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
}

void test_moderate_motion_does_not_start_post_exercise()
{
    Fixture f;
    f.seconds(5); // arm recovery with valid resting data
    f.input.motion = Motion::MODERATE_MOTION;
    f.seconds(70);
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
    TEST_ASSERT_EQUAL_UINT(1, f.monitor.sampleCount());
    TEST_ASSERT_EQUAL_STRING("MOTION", collectionGateName(f.monitor.collectionGate()));
    f.input.motion = Motion::LOW_MOTION;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
    f.seconds(5);
    TEST_ASSERT_EQUAL_UINT(2, f.monitor.sampleCount());
}

void test_moderate_pauses_recovery_without_resetting_low_time()
{
    Fixture f;
    f.seconds(5);
    f.input.motion = Motion::HIGH_MOTION;
    f.tick();
    f.input.motion = Motion::LOW_MOTION;
    f.tick();
    f.seconds(20);
    TEST_ASSERT_EQUAL_UINT32(40000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.input.motion = Motion::MODERATE_MOTION;
    f.seconds(90);
    TEST_ASSERT_TRUE(f.monitor.postExerciseRecovery());
    TEST_ASSERT_EQUAL_UINT32(40000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.input.motion = Motion::LOW_MOTION;
    f.tick();
    f.seconds(39);
    TEST_ASSERT_EQUAL_UINT32(1000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.tick();
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
}

void test_recovery_low_time_handles_rollover_and_ignores_report_gaps()
{
    Fixture f;
    f.seconds(5);
    f.input.now = UINT32_MAX - 2000;
    f.input.motion = Motion::HIGH_MOTION;
    f.monitor.update(f.input);
    f.input.motion = Motion::LOW_MOTION;
    f.tick();
    f.seconds(3); // cross millis rollover
    TEST_ASSERT_EQUAL_UINT32(57000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.input.now += 100000;
    f.monitor.update(f.input);
    TEST_ASSERT_EQUAL_UINT32(57000, f.monitor.motionRecoveryRemainingMs(f.input.now));
    f.seconds(57);
    TEST_ASSERT_FALSE(f.monitor.postExerciseRecovery());
}

void test_fixed_session_triggers_before_seven_saved_sessions()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SESSION), static_cast<int>(f.last.p1));
    f.seconds(90);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SESSION), static_cast<int>(f.last.p2));
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1LongHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p2LongHoldMs(f.input.now));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, f.monitor.shortReference());
}

void test_session_and_adapting_short_can_trigger_together()
{
    Fixture f;
    f.initialize();
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_AND_SESSION), static_cast<int>(f.last.p1));
    f.seconds(90);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_AND_SESSION), static_cast<int>(f.last.p2));
    f.seconds(180);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, f.monitor.shortReference());
    TEST_ASSERT_TRUE(f.monitor.p2Triggered()); // fixed session still shows a drop
    TEST_ASSERT_EQUAL_UINT32(300000, f.monitor.validDurationMs());
}

void test_live_short_adaptation_can_cancel_a_pending_p2()
{
    Fixture f;
    f.seconds(60); // small initial window adapts faster than a full five minutes
    f.input.current = 70;
    f.tick();
    f.seconds(120);
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p2ShortHoldMs(f.input.now));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, f.monitor.shortReference());
}

void test_new_personal_baseline_does_not_inherit_old_trigger_time()
{
    Fixture f;
    f.input.longTermAvailable = true;
    f.input.longTermBaseline = 100;
    f.input.current = 70;
    f.tick();
    f.seconds(20);
    f.input.longTermBaseline = 110;
    f.tick();
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::LONG_TERM), static_cast<int>(f.tick().p1));
}

void assertAllHolds(const Fixture &f, uint32_t p1, uint32_t p2)
{
    TEST_ASSERT_EQUAL_UINT32(p1, f.monitor.p1LongHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(p1, f.monitor.p1SessionHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(p1, f.monitor.p1ShortHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(p2, f.monitor.p2LongHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(p2, f.monitor.p2SessionHoldMs(f.input.now));
    TEST_ASSERT_EQUAL_UINT32(p2, f.monitor.p2ShortHoldMs(f.input.now));
}

void test_p1_continues_through_p2_and_resets_only_above_p1_boundary()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = f.input.sessionAvailable = true;
    f.input.longTermBaseline = f.input.sessionBaseline = 100;
    // Keep the short reference fixed to isolate threshold/timer transitions.
    f.input.interventionActive = true;
    f.input.current = 95; // exactly 5%: P1 only
    f.monitor.update(f.input);
    f.seconds(20);
    assertAllHolds(f, 20000, 0);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());

    f.input.current = 90; // exactly 10%: start P2 without restarting P1
    f.monitor.update(f.input);
    assertAllHolds(f, 20000, 0);
    f.seconds(9);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::ALL, (int)f.tick().p1);
    assertAllHolds(f, 30000, 10000);

    f.seconds(109);
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::ALL, (int)f.tick().p2);
    assertAllHolds(f, 140000, 120000);

    f.input.current = 92; // leaving P2 keeps the original P1 timer running
    f.tick();
    assertAllHolds(f, 141000, 0);
    f.input.current = 95;
    f.tick();
    assertAllHolds(f, 142000, 0);
    f.input.current = 95.1f;
    f.tick();
    assertAllHolds(f, 0, 0);
    // Resetting a hold does not bypass the existing episode-recovery latch.
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.monitor.p2Triggered());
}

void test_p2_restart_does_not_borrow_time_or_reset_p1()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = f.input.sessionAvailable = true;
    f.input.longTermBaseline = f.input.sessionBaseline = 100;
    f.input.interventionActive = true; // isolate timers from short adaptation
    f.input.current = 90;
    f.monitor.update(f.input);
    f.seconds(20);
    assertAllHolds(f, 20000, 20000);
    f.input.current = 90.1f; // just above P2, still below P1
    f.tick();
    assertAllHolds(f, 21000, 0);
    f.seconds(9);
    TEST_ASSERT_EQUAL_INT((int)Source::ALL, (int)f.last.p1);
    f.input.current = 90;
    f.tick();
    assertAllHolds(f, 31000, 0);
    f.seconds(119);
    TEST_ASSERT_FALSE(f.monitor.p2Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::ALL, (int)f.tick().p2);
    assertAllHolds(f, 151000, 120000);
}

void test_short_adapts_to_trends_but_retains_quality_gates()
{
    Fixture f;
    f.input.stable = false;
    for (unsigned i = 0; i < 360; ++i) {
        f.input.current = 100 + i * 0.1f;
        f.tick();
        TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
        TEST_ASSERT_EQUAL_STRING("RMSSD_UNSTABLE", collectionGateName(f.monitor.restingGate()));
    }
    TEST_ASSERT_TRUE(f.monitor.shortAvailable());
    TEST_ASSERT_EQUAL_UINT(60, f.monitor.sampleCount());
    TEST_ASSERT_GREATER_THAN_FLOAT(110, f.monitor.shortReference());
    f.input.interventionActive = true;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("HAPTIC_ACTIVE_OR_STOP_PENDING", collectionGateName(f.monitor.collectionGate()));
    f.input.interventionActive = false;
    f.input.signalGood = false;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("SIGNAL_OR_IMU", collectionGateName(f.monitor.collectionGate()));
    f.input.signalGood = true;
    f.input.motion = Motion::MODERATE_MOTION;
    f.tick();
    TEST_ASSERT_EQUAL_STRING("MOTION", collectionGateName(f.monitor.collectionGate()));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_short_adapts_to_trends_but_retains_quality_gates);
    RUN_TEST(test_p1_continues_through_p2_and_resets_only_above_p1_boundary);
    RUN_TEST(test_p2_restart_does_not_borrow_time_or_reset_p1);
    RUN_TEST(test_requires_one_minute_and_prefers_five);
    RUN_TEST(test_median_uses_middle_pair_and_resists_outlier);
    RUN_TEST(test_partial_or_bad_quality_blocks_do_not_count);
    RUN_TEST(test_old_readings_expire_in_wall_time);
    RUN_TEST(test_ten_percent_drop_triggers_p1_at_the_configured_five_percent_threshold);
    RUN_TEST(test_both_paths_trigger_at_30_and_120_seconds);
    RUN_TEST(test_short_path_does_not_require_long_term_baseline);
    RUN_TEST(test_long_term_path_does_not_require_short_reference);
    RUN_TEST(test_source_timers_cannot_borrow_elapsed_time);
    RUN_TEST(test_invalid_input_and_report_gaps_break_continuous_timers);
    RUN_TEST(test_intervention_and_contact_loss_preserve_episode_latches);
    RUN_TEST(test_recovery_must_be_below_ten_percent_continuously);
    RUN_TEST(test_motion_requires_one_minute_of_continuous_low);
    RUN_TEST(test_startup_motion_does_not_start_recovery_before_resting_data);
    RUN_TEST(test_timers_and_sample_expiry_survive_millis_rollover);
    RUN_TEST(test_long_wait_reports_intervention_block_and_collection_resumes);
    RUN_TEST(test_between_report_interruptions_are_visible_even_when_gate_is_open);
    RUN_TEST(test_collection_diagnostics_show_quality_and_motion_recovery);
    RUN_TEST(test_moderate_motion_does_not_start_post_exercise);
    RUN_TEST(test_moderate_pauses_recovery_without_resetting_low_time);
    RUN_TEST(test_recovery_low_time_handles_rollover_and_ignores_report_gaps);
    RUN_TEST(test_fixed_session_triggers_before_seven_saved_sessions);
    RUN_TEST(test_session_and_adapting_short_can_trigger_together);
    RUN_TEST(test_live_short_adaptation_can_cancel_a_pending_p2);
    RUN_TEST(test_new_personal_baseline_does_not_inherit_old_trigger_time);
    return UNITY_END();
}
