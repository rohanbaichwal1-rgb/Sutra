#include <unity.h>
#include <PdrObserver.h>
#include <math.h>

void setUp() {}
void tearDown() {}
const pdr::Context REST = {true, true, true, true, false};
const pdr::Context PACING = {true, true, true, false, true};

// Irregular beat timestamps, with independent controllable RIAV/RIFV periods.
struct Stream {
    pdr::Observer observer;
    uint32_t base = 0, elapsed = 0, nextBeat = 0;
    void run(uint32_t end, float avRate = 12, float fvRate = 12,
             pdr::Context context = REST, float avDepth = 0.12f, float fvDepth = 80) {
        for (; elapsed <= end; elapsed += 100) {
            while (nextBeat <= elapsed) {
                float phase = 6.283185307f*nextBeat/60000.0f;
                float ibi = 800 + fvDepth*sinf(phase*fvRate);
                observer.addBeat(base+nextBeat, 1000*(1+avDepth*sinf(phase*avRate)), ibi);
                nextBeat += (uint32_t)roundf(ibi);
            }
            observer.update(base+elapsed, context);
        }
    }
};

void test_full_window_then_dual_channel_rate() {
    Stream s;
    s.run(59000);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    s.run(65000);
    auto r = s.observer.result();
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f, 12, r.rate);
    TEST_ASSERT_EQUAL_UINT(3, r.sources);
    TEST_ASSERT_GREATER_OR_EQUAL(70, r.quality);
}
void test_flat_signals_are_unavailable_not_zero_breathing() {
    Stream s; s.run(70000,12,12,REST,0,0);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::WEAK,(int)s.observer.result().status);
    TEST_ASSERT_FALSE(s.observer.result().support);
}
void test_single_channel_allowed_with_capped_quality() {
    Stream s; s.run(70000,12,12,REST,0.12f,0);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_UINT(1,s.observer.result().sources);
    TEST_ASSERT_LESS_OR_EQUAL(75,s.observer.result().quality);
}
void test_conflicting_channels_are_rejected() {
    Stream s; s.run(70000,12,20);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::DISAGREE,(int)s.observer.result().status);
}
void test_baseline_requires_180_qualifying_seconds_and_freezes() {
    Stream s; s.run(230000);
    TEST_ASSERT_FALSE(s.observer.result().baselineReady);
    s.run(300000);
    TEST_ASSERT_TRUE(s.observer.result().baselineReady);
    TEST_ASSERT_EQUAL_UINT32(180000,s.observer.result().baselineValidMs);
    const float baseline = s.observer.result().baseline;
    s.run(420000,18,18);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,baseline,s.observer.result().baseline);
    TEST_ASSERT_TRUE(s.observer.result().support);
    TEST_ASSERT_GREATER_OR_EQUAL(20000,s.observer.result().supportMs);
}
void test_deviation_hold_and_loss_of_quality() {
    Stream s; s.run(300000);
    bool sawPending = false;
    while (s.elapsed <= 420000) {
        s.run(s.elapsed,18,18);
        const auto &r = s.observer.result();
        if (r.supportMs && r.supportMs < pdr::SUPPORT_MS) {
            sawPending = true; TEST_ASSERT_FALSE(r.support);
        }
    }
    TEST_ASSERT_TRUE(sawPending);
    TEST_ASSERT_TRUE(s.observer.result().support);
    s.observer.update(s.elapsed,{false,true,true,true,false});
    TEST_ASSERT_FALSE(s.observer.result().support);
    TEST_ASSERT_EQUAL_UINT32(0,s.observer.result().supportMs);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_TRUE(s.observer.result().baselineReady);
}
void test_slow_breathing_deviation_and_intervention_suppression() {
    Stream s; s.run(300000); s.run(430000,6,6);
    TEST_ASSERT_TRUE(s.observer.result().support);
    TEST_ASSERT_LESS_THAN(-20,s.observer.result().deviation);
    s.observer.update(s.elapsed,PACING);
    TEST_ASSERT_FALSE(s.observer.result().support);
    TEST_ASSERT_EQUAL_UINT32(0,s.observer.result().supportMs);
}
void test_motion_discards_window_and_preserves_completed_baseline_blocks() {
    Stream s; s.run(150000);
    const uint32_t valid = s.observer.result().baselineValidMs;
    TEST_ASSERT_GREATER_THAN(0,valid);
    s.run(151000,12,12,{true,false,true,true,false});
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::MOTION,(int)s.observer.result().status);
    TEST_ASSERT_EQUAL_UINT32(valid,s.observer.result().baselineValidMs);
    s.run(190000);
    TEST_ASSERT_FALSE(s.observer.result().valid);
}
void test_report_gap_and_contact_reset() {
    Stream s; s.run(70000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    s.observer.update(80000,REST);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    Stream contact; contact.run(70000);
    contact.observer.contactChanged();
    TEST_ASSERT_FALSE(contact.observer.result().valid);
}
void test_long_beat_gap_does_not_get_interpolated() {
    Stream s; s.run(70000);
    s.observer.addBeat(75000,1000,800);
    s.observer.update(75000,REST);
    TEST_ASSERT_FALSE(s.observer.result().valid);
}
void test_timer_rollover() {
    Stream s; s.base = UINT32_MAX-90000;
    s.run(300000);
    TEST_ASSERT_TRUE(s.observer.result().baselineReady);
    s.run(430000,18,18);
    TEST_ASSERT_TRUE(s.observer.result().support);
}
void test_brief_recovery_gate_breaks_baseline_block_between_estimates() {
    Stream s; s.run(151000);
    const uint32_t completed = s.observer.result().baselineValidMs;
    s.observer.update(151100,{true,true,false,true,false});
    s.run(154000);
    TEST_ASSERT_EQUAL_UINT32(completed,s.observer.result().baselineValidMs);
}
void test_unstructured_noise_does_not_become_respiration() {
    pdr::Observer observer;
    uint32_t random = 0x12345;
    for (uint32_t t = 0; t <= 70000; t += 100) {
        if (t%800 == 0) {
            random = random*1664525UL + 1013904223UL;
            float a = 1000 + (int)(random%201)-100;
            random = random*1664525UL + 1013904223UL;
            float ibi = 800 + (int)(random%101)-50;
            observer.addBeat(t,a,ibi);
        }
        observer.update(t,REST);
    }
    TEST_ASSERT_FALSE(observer.result().valid);
    TEST_ASSERT_FALSE(observer.result().support);
}
void test_p2_rate_match_uses_only_intervention_data() {
    Stream s; s.run(120000,18,18);
    s.observer.beginP2(120000);
    s.run(179900,6,6,PACING);
    TEST_ASSERT_FALSE(s.observer.result().support);
    s.observer.endP2(true);
    TEST_ASSERT_EQUAL_INT((int)pdr::Sync::RATE_MATCH,(int)s.observer.result().sync);
}
void test_p2_wrong_rate_and_aborted_intervention() {
    Stream s; s.run(120000);
    s.observer.beginP2(120000);
    s.run(179900,12,12,PACING);
    s.observer.endP2(true);
    TEST_ASSERT_EQUAL_INT((int)pdr::Sync::NO_RATE_MATCH,(int)s.observer.result().sync);
    s.observer.beginP2(180000);
    s.observer.endP2(false);
    TEST_ASSERT_EQUAL_INT((int)pdr::Sync::UNKNOWN,(int)s.observer.result().sync);
}
void test_p2_poor_quality_and_queue_loss_are_unknown() {
    Stream s; s.run(120000);
    s.observer.beginP2(120000);
    s.run(179900,6,6,PACING,0,0);
    s.observer.endP2(true);
    TEST_ASSERT_EQUAL_INT((int)pdr::Sync::UNKNOWN,(int)s.observer.result().sync);
    Stream lost; lost.run(120000);
    lost.observer.beginP2(120000);
    lost.run(175000,6,6,PACING);
    lost.observer.dataLost();
    lost.observer.endP2(true);
    TEST_ASSERT_EQUAL_INT((int)pdr::Sync::UNKNOWN,(int)lost.observer.result().sync);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_window_then_dual_channel_rate);
    RUN_TEST(test_flat_signals_are_unavailable_not_zero_breathing);
    RUN_TEST(test_single_channel_allowed_with_capped_quality);
    RUN_TEST(test_conflicting_channels_are_rejected);
    RUN_TEST(test_baseline_requires_180_qualifying_seconds_and_freezes);
    RUN_TEST(test_deviation_hold_and_loss_of_quality);
    RUN_TEST(test_slow_breathing_deviation_and_intervention_suppression);
    RUN_TEST(test_motion_discards_window_and_preserves_completed_baseline_blocks);
    RUN_TEST(test_report_gap_and_contact_reset);
    RUN_TEST(test_long_beat_gap_does_not_get_interpolated);
    RUN_TEST(test_timer_rollover);
    RUN_TEST(test_brief_recovery_gate_breaks_baseline_block_between_estimates);
    RUN_TEST(test_unstructured_noise_does_not_become_respiration);
    RUN_TEST(test_p2_rate_match_uses_only_intervention_data);
    RUN_TEST(test_p2_wrong_rate_and_aborted_intervention);
    RUN_TEST(test_p2_poor_quality_and_queue_loss_are_unknown);
    return UNITY_END();
}
