#include <unity.h>
#include <RmssdReferences.h>
#include <math.h>

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

void test_configured_drop_triggers_p1_after_30_seconds()
{
    Fixture f;
    f.initialize();
    f.input.current = 100.0f - P1_DROP_PCT;
    f.tick();
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.tick();
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_UINT32(300000, f.monitor.validDurationMs());
    // The 5-minute median remains at its former value during this initial
    // 30-second drop, so the trigger cannot be attributed to a changed reference.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100, f.monitor.shortReference());
    TEST_ASSERT_EQUAL_STRING("OPEN", collectionGateName(f.monitor.collectionGate()));
}

void test_both_paths_trigger_once_after_30_seconds()
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
    f.seconds(90);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    f.seconds(200);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::NONE), static_cast<int>(f.last.p1));
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

void test_moderate_motion_continues_running_p1_hold_but_high_motion_resets_it()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 95; // Exactly the configured 5% P1 threshold.
    f.tick();
    f.seconds(14);
    TEST_ASSERT_EQUAL_UINT32(14000, f.monitor.p1SessionHoldMs(f.input.now));

    f.input.motion = Motion::MODERATE_MOTION;
    f.seconds(10);
    // MODERATE neither resets nor pauses the P1 persistence duration.
    TEST_ASSERT_EQUAL_UINT32(24000, f.monitor.p1SessionHoldMs(f.input.now));
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.seconds(5);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SESSION), static_cast<int>(f.tick().p1));

    Fixture high;
    high.input.sessionAvailable = true;
    high.input.sessionBaseline = 100;
    high.input.current = 95;
    high.tick();
    high.seconds(10);
    high.input.motion = Motion::HIGH_MOTION;
    high.tick();
    TEST_ASSERT_EQUAL_UINT32(0, high.monitor.p1SessionHoldMs(high.input.now));
}

void test_blank_dev_bridges_running_p1_hold_but_report_gaps_still_reset()
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
    f.seconds(7);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(f.tick().p1));

    Fixture gap;
    gap.initialize();
    gap.input.current = 80;
    gap.tick();
    gap.seconds(20);
    gap.input.now += 5000;
    gap.monitor.update(gap.input);
    gap.seconds(29);
    TEST_ASSERT_FALSE(gap.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SHORT_TERM), static_cast<int>(gap.tick().p1));
}

void test_running_p1_hold_bridges_up_to_ten_blank_dev_samples()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 80;
    f.tick();
    f.seconds(14);
    for (unsigned i = 0; i < 5; ++i)
    {
        f.input.signalGood = false;
        f.input.current = NAN;
        f.monitor.interruptSignalContinuity();
        f.tick();
    }
    TEST_ASSERT_EQUAL_UINT32(19000, f.monitor.p1SessionHoldMs(f.input.now));
    f.input.signalGood = true;
    f.input.current = 80;
    f.tick();
    f.seconds(9);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Source::SESSION), static_cast<int>(f.tick().p1));
}

void test_valid_below_threshold_after_blank_dev_resets_p1_hold()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 80;
    f.tick();
    f.seconds(14);
    for (unsigned i = 0; i < 5; ++i)
    {
        f.input.signalGood = false;
        f.input.current = NAN;
        f.tick();
    }
    f.input.signalGood = true;
    f.input.current = 100;
    f.tick();
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1SessionHoldMs(f.input.now));
}

void test_eleventh_consecutive_blank_dev_resets_and_blank_cannot_start_p1_hold()
{
    Fixture f;
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 80;
    f.tick();
    for (unsigned i = 0; i < P1_MAX_CONSECUTIVE_BLANK_DEV; ++i)
    {
        f.input.signalGood = false;
        f.input.current = NAN;
        f.tick();
    }
    TEST_ASSERT_EQUAL_UINT32(P1_MAX_CONSECUTIVE_BLANK_DEV * 1000UL,
                             f.monitor.p1SessionHoldMs(f.input.now));
    f.tick();
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1SessionHoldMs(f.input.now));

    Fixture blankOnly;
    blankOnly.input.sessionAvailable = true;
    blankOnly.input.sessionBaseline = 100;
    blankOnly.input.signalGood = false;
    blankOnly.input.current = NAN;
    blankOnly.seconds(P1_MAX_CONSECUTIVE_BLANK_DEV);
    TEST_ASSERT_EQUAL_UINT32(0, blankOnly.monitor.p1SessionHoldMs(blankOnly.input.now));
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

void test_recovery_must_be_below_rearm_boundary_continuously()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = true;
    f.input.current = 70;
    f.tick();
    f.seconds(30);
    f.input.current = 100.0f - P1_DROP_PCT; // At the trigger boundary: not recovered.
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
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1LongHoldMs(f.input.now));
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
    f.seconds(180);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70, f.monitor.shortReference());
    TEST_ASSERT_EQUAL_UINT32(300000, f.monitor.validDurationMs());
}

void test_live_short_adaptation_resets_p1_hold_but_keeps_episode()
{
    Fixture f;
    f.seconds(60); // small initial window adapts faster than a full five minutes
    f.input.current = 70;
    f.tick();
    f.seconds(120);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1ShortHoldMs(f.input.now));
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


void triggerSession(Fixture &f)
{
    f.input.sessionAvailable = true;
    f.input.sessionBaseline = 100;
    f.input.current = 100.0f - P1_DROP_PCT;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_EQUAL_INT((int)Source::SESSION, (int)f.last.p1);
    // P1 detection alone does not start the breathing delay.  Runtime haptic
    // completion explicitly arms it after the five-second P1 pattern.
    TEST_ASSERT_EQUAL_INT((int)FollowUp::IDLE, (int)f.monitor.followUp());
}

void completeInitialP1AndArmFollowUp(Fixture &f)
{
    f.monitor.armFollowUpCountdown(f.input.now);
    TEST_ASSERT_EQUAL_INT((int)FollowUp::PENDING, (int)f.monitor.followUp());
}

void test_threshold_boundary_for_all_three_paths()
{
    Fixture f;
    f.initialize();
    f.input.longTermAvailable = f.input.sessionAvailable = true;
    f.input.longTermBaseline = f.input.sessionBaseline = 100;
    f.input.interventionActive = true; // Freeze short adaptation, not eligibility.
    f.input.current = 100.0f - P1_DROP_PCT + 0.1f;
    f.seconds(40);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    f.input.current = 100.0f - P1_DROP_PCT;
    f.tick();
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::ALL, (int)f.tick().p1);
}

void test_follow_up_at_exact_five_seconds_once_despite_active_flutter()
{
    Fixture f;
    triggerSession(f);
    completeInitialP1AndArmFollowUp(f);
    const uint32_t triggered = f.input.now;
    f.input.interventionActive = true;
    f.input.stable = false; // Neither is a trigger eligibility check.
    for (unsigned ms = 10; ms <= 6000; ms += 10)
    {
        f.input.now = triggered + ms;
        if (ms % 1000 == 0) f.monitor.update(f.input);
        auto event = f.monitor.pollFollowUp(f.input);
        TEST_ASSERT_EQUAL_INT((int)(ms == 5000 ? FollowUpEvent::START : FollowUpEvent::NONE), (int)event);
    }
    TEST_ASSERT_EQUAL_INT((int)FollowUp::ISSUED, (int)f.monitor.followUp());
    f.seconds(180);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::NONE, (int)f.last.p1);
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::NONE, (int)f.monitor.pollFollowUp(f.input));
}

void test_signal_or_activity_does_not_cancel_follow_up_before_deadline()
{
    for (unsigned failure = 0; failure < 6; ++failure)
    {
        Fixture f;
        triggerSession(f);
        completeInitialP1AndArmFollowUp(f);
        f.input.interventionActive = true;
        f.seconds(2);
        f.input.now += 10;
        switch (failure)
        {
        case 0: f.input.signalGood = false; break;
        case 1: f.input.current = NAN; break;
        case 2: f.input.motion = Motion::MODERATE_MOTION; break;
        case 3: f.input.motion = Motion::HIGH_MOTION; break;
        case 4: f.monitor.interruptContinuity(); break;
        case 5: f.monitor.contactChanged(); break;
        }
        // A triggered intervention owns its delay.  Physiological changes,
        // report continuity, and contact state must not cancel it.
        TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::NONE, (int)f.monitor.pollFollowUp(f.input));
        f.input.current = 90; f.input.signalGood = true;
        f.input.motion = Motion::LOW_MOTION; f.input.sessionAvailable = true;
        f.seconds(3);
        TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::START, (int)f.monitor.pollFollowUp(f.input));
        TEST_ASSERT_EQUAL_INT((int)FollowUp::ISSUED, (int)f.monitor.followUp());
        TEST_ASSERT_TRUE(f.monitor.p1Triggered());
        TEST_ASSERT_EQUAL_INT((int)Source::NONE, (int)f.last.p1);
    }
}

void test_deviation_recovery_does_not_cancel_or_restart_follow_up()
{
    Fixture f;
    triggerSession(f);
    completeInitialP1AndArmFollowUp(f);
    const uint32_t triggered = f.input.now;
    f.seconds(2);
    f.input.current = 100;
    f.tick();
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::NONE, (int)f.monitor.pollFollowUp(f.input));
    TEST_ASSERT_EQUAL_INT((int)FollowUp::PENDING, (int)f.monitor.followUp());
    f.tick();
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::NONE, (int)f.monitor.pollFollowUp(f.input));
    f.tick();
    TEST_ASSERT_EQUAL_UINT32(triggered + P1_FOLLOW_UP_DELAY_MS, f.input.now);
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::START, (int)f.monitor.pollFollowUp(f.input));
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::NONE, (int)f.monitor.pollFollowUp(f.input));
}

void test_reference_changes_do_not_cancel_follow_up_after_trigger()
{
    Fixture f;
    triggerSession(f);
    completeInitialP1AndArmFollowUp(f);
    for (unsigned i = 0; i < 5; ++i)
    {
        f.input.current = 120;
        f.input.sessionAvailable = false;
        f.input.longTermAvailable = (i & 1U) != 0;
        f.input.longTermBaseline = 80 + i;
        f.tick();
        TEST_ASSERT_EQUAL_INT((int)(i == 4 ? FollowUpEvent::START : FollowUpEvent::NONE),
                              (int)f.monitor.pollFollowUp(f.input));
    }
}

void test_follow_up_starts_across_unobserved_gap_and_wraps_millis()
{
    Fixture f;
    triggerSession(f);
    completeInitialP1AndArmFollowUp(f);
    f.input.now += 5000;
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::START, (int)f.monitor.pollFollowUp(f.input));
    Fixture wrap;
    wrap.input.now = UINT32_MAX - 34000;
    triggerSession(wrap);
    completeInitialP1AndArmFollowUp(wrap);
    for (unsigned i = 0; i < 5; ++i)
    {
        wrap.tick();
        auto event = wrap.monitor.pollFollowUp(wrap.input);
        TEST_ASSERT_EQUAL_INT((int)(i == 4 ? FollowUpEvent::START : FollowUpEvent::NONE), (int)event);
    }
}

void test_completed_episode_rearms_after_counter_reset_and_retriggers()
{
    Fixture f;
    triggerSession(f); // First independent 30-second physiological event.
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());

    // P1 playback has completed, then the 5-second delay emits exactly one
    // follow-up start.  The counter may reset while that intervention runs.
    completeInitialP1AndArmFollowUp(f);
    f.seconds(5);
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::START, (int)f.monitor.pollFollowUp(f.input));
    f.input.current = 100.0f;
    f.tick();
    TEST_ASSERT_TRUE(f.monitor.p1Triggered()); // Do not clear mid-intervention.

    // Runtime reports completion only after the 12-second breathing stage
    // stops.  Since all persistence holds are now zero, the next monitor
    // update completely rearms P1 for a new episode.
    f.monitor.completeP1Intervention();
    TEST_ASSERT_TRUE(f.tick().resumed);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::NONE, (int)f.monitor.p1Source());
    TEST_ASSERT_EQUAL_INT((int)FollowUp::IDLE, (int)f.monitor.followUp());

    // A second independent 30-second event must be emitted normally.
    f.input.current = 100.0f - P1_DROP_PCT;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_TRUE(f.last.p1 != Source::NONE);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());

    // Repeat once more to prove state does not accumulate a permanent latch.
    f.input.current = 100.0f;
    f.tick();
    f.monitor.completeP1Intervention();
    TEST_ASSERT_TRUE(f.tick().resumed);
    f.input.current = 100.0f - P1_DROP_PCT;
    f.tick();
    f.seconds(30);
    TEST_ASSERT_TRUE(f.last.p1 != Source::NONE);
}

void test_completed_follow_up_immediately_starts_fresh_p1_hold()
{
    Fixture f;
    triggerSession(f);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.monitor.p1SessionHoldMs(f.input.now) >= P1_HOLD_MS);

    // The successful 12-second follow-up ends the episode even though DEV
    // remains at the P1 threshold. No previous hold time may carry over.
    f.monitor.completeP1FollowUp();
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::NONE, (int)f.monitor.p1Source());
    TEST_ASSERT_EQUAL_INT((int)FollowUp::IDLE, (int)f.monitor.followUp());
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1SessionHoldMs(f.input.now));

    f.tick();
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_UINT32(0, f.monitor.p1SessionHoldMs(f.input.now));
    f.seconds(29);
    TEST_ASSERT_FALSE(f.monitor.p1Triggered());
    TEST_ASSERT_EQUAL_INT((int)Source::SESSION, (int)f.tick().p1);
}

void test_motor_fault_blocked_playback_requires_existing_recovery_before_new_trigger()
{
    Fixture f;
    triggerSession(f);
    completeInitialP1AndArmFollowUp(f);
    // This is the same operation main.cpp uses when a motor fault blocks
    // playback.  It cancels only the haptic follow-up, not the P1 episode.
    f.monitor.cancelFollowUp();
    TEST_ASSERT_EQUAL_INT((int)FollowUpEvent::CANCELLED, (int)f.monitor.pollFollowUp(f.input));
    f.input.current = 100;
    f.seconds(60);
    TEST_ASSERT_TRUE(f.monitor.p1Triggered());
    TEST_ASSERT_TRUE(f.tick().resumed);
    TEST_ASSERT_EQUAL_INT((int)FollowUp::IDLE, (int)f.monitor.followUp());
    f.input.current = 100.0f - P1_DROP_PCT; f.tick(); f.seconds(30);
    TEST_ASSERT_TRUE(f.last.p1 != Source::NONE);
    TEST_ASSERT_EQUAL_INT((int)FollowUp::IDLE, (int)f.monitor.followUp());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_threshold_boundary_for_all_three_paths);
    RUN_TEST(test_follow_up_at_exact_five_seconds_once_despite_active_flutter);
    RUN_TEST(test_signal_or_activity_does_not_cancel_follow_up_before_deadline);
    RUN_TEST(test_deviation_recovery_does_not_cancel_or_restart_follow_up);
    RUN_TEST(test_reference_changes_do_not_cancel_follow_up_after_trigger);
    RUN_TEST(test_follow_up_starts_across_unobserved_gap_and_wraps_millis);
    RUN_TEST(test_completed_episode_rearms_after_counter_reset_and_retriggers);
    RUN_TEST(test_completed_follow_up_immediately_starts_fresh_p1_hold);
    RUN_TEST(test_motor_fault_blocked_playback_requires_existing_recovery_before_new_trigger);
    RUN_TEST(test_short_adapts_to_trends_but_retains_quality_gates);
    RUN_TEST(test_requires_one_minute_and_prefers_five);
    RUN_TEST(test_median_uses_middle_pair_and_resists_outlier);
    RUN_TEST(test_partial_or_bad_quality_blocks_do_not_count);
    RUN_TEST(test_old_readings_expire_in_wall_time);
    RUN_TEST(test_configured_drop_triggers_p1_after_30_seconds);
    RUN_TEST(test_both_paths_trigger_once_after_30_seconds);
    RUN_TEST(test_short_path_does_not_require_long_term_baseline);
    RUN_TEST(test_long_term_path_does_not_require_short_reference);
    RUN_TEST(test_source_timers_cannot_borrow_elapsed_time);
    RUN_TEST(test_moderate_motion_continues_running_p1_hold_but_high_motion_resets_it);
    RUN_TEST(test_blank_dev_bridges_running_p1_hold_but_report_gaps_still_reset);
    RUN_TEST(test_running_p1_hold_bridges_up_to_ten_blank_dev_samples);
    RUN_TEST(test_valid_below_threshold_after_blank_dev_resets_p1_hold);
    RUN_TEST(test_eleventh_consecutive_blank_dev_resets_and_blank_cannot_start_p1_hold);
    RUN_TEST(test_intervention_and_contact_loss_preserve_episode_latches);
    RUN_TEST(test_recovery_must_be_below_rearm_boundary_continuously);
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
    RUN_TEST(test_live_short_adaptation_resets_p1_hold_but_keeps_episode);
    RUN_TEST(test_new_personal_baseline_does_not_inherit_old_trigger_time);
    return UNITY_END();
}
