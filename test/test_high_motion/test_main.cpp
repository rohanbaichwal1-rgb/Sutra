#include <unity.h>
#include <HighMotionProcessor.h>
#include <math.h>
#include <limits>

void setUp() {}
void tearDown() {}

static void runPulse(highmotion::Processor &p, uint32_t base, unsigned milliseconds,
                     bool motionValid = true, bool artifact = false,
                     unsigned periodMs = 1000, bool missingPulse = false) {
    for (unsigned ms = 0; ms < milliseconds; ms += 10) {
        uint32_t t = base + ms * 1000;
        float position = (ms % periodMs) * 0.001f;
        float pulse = position < 0.4f ? sinf(position * 3.14159265f / 0.4f) : 0;
        if (missingPulse && ms >= 1000 && ms < 2000) pulse = 0;
        float ax = artifact ? 0.15f * pulse : 0;
        p.pushImu(t, ax, 0, 1, motionValid);
        p.beginBatch(t, 1);
        p.pushPpg(50000 - 100 * pulse);
        p.poll(t);
    }
}
void test_regular_pulses_acquire_measured_intervals_and_expire() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000);
    auto r = p.read(15100000);
    TEST_ASSERT_TRUE_MESSAGE(r.valid, highmotion::reasonName(r.reason));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60, r.hr);
    TEST_ASSERT_FLOAT_WITHIN(20, 1000, r.ibiMs);
    TEST_ASSERT_TRUE(r.timestampsEstimated);
    TEST_ASSERT_FALSE(p.read(19000000).valid);
    TEST_ASSERT_TRUE(isnan(p.read(19000000).hr));
}
void test_motion_transitions_keep_shadow_warm_but_contact_clears_history() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000);
    TEST_ASSERT_TRUE(p.read(15100000).valid);
    p.setActive(false, true, 15100000);
    TEST_ASSERT_FALSE(p.read(15100000).active);
    p.setActive(true, true, 15200000);
    TEST_ASSERT_TRUE(p.read(15200000).ready);
    p.setActive(true, false, 15300000);
    TEST_ASSERT_FALSE(p.read(15300000).valid);
#if HIGH_MOTION_OUTPUT_OVERRIDE
    TEST_ASSERT_TRUE(isnan(p.selectHr(72, true, 15300000)));
#else
    TEST_ASSERT_EQUAL_FLOAT(72, p.selectHr(72, true, 15300000));
#endif
    TEST_ASSERT_EQUAL_FLOAT(72, p.selectHr(72, false, 15300000));
}
void test_missing_imu_and_large_batches_cannot_publish() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000, false);
    auto r = p.read(15100000);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_GREATER_THAN_UINT32(0, r.alignmentDrops);
    p.beginBatch(15200000, 10);
    for (unsigned i = 0; i < 10; ++i) p.pushPpg(50000);
    p.poll(15300000);
    TEST_ASSERT_FALSE(p.read(15300000).valid);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(highmotion::Reason::TIMING), static_cast<int>(p.read(15300000).reason));
}
void test_wraparound_and_motion_locked_pulses() {
    highmotion::Processor p;
    uint32_t base = UINT32_MAX - 5000000;
    p.setActive(true, true, base);
    runPulse(p, base, 15000);
    TEST_ASSERT_TRUE(p.read(base + 15000000).valid);
    highmotion::Processor coupled;
    coupled.setActive(true, true, 100000);
    runPulse(coupled, 100000, 15000, true, true);
    TEST_ASSERT_FALSE(coupled.read(15100000).valid);
    TEST_ASSERT_TRUE(isfinite(coupled.read(15100000).coefficientNorm));
}
void test_overflow_and_nonfinite_samples_fail_closed() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    for (unsigned i = 0; i < 20; ++i) {
        p.beginBatch(100000 + i * 10000, 1);
        p.pushPpg(50000);
    }
    TEST_ASSERT_FALSE(p.read(300000).valid);
    TEST_ASSERT_GREATER_THAN_UINT32(0, p.read(300000).rejected);
    p.poll(400000);
    p.pushImu(410000, 0, 0, 1, true);
    p.pushImu(420000, 0, 0, 1, true);
    p.beginBatch(415000, 1);
    p.pushPpg(std::numeric_limits<float>::infinity());
    p.poll(420000);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(highmotion::Reason::NUMERIC), static_cast<int>(p.read(420000).reason));
}
void test_missing_pulse_requires_reacquisition_without_splitting_ibi() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000);
    TEST_ASSERT_TRUE(p.read(15100000).valid);
    runPulse(p, 15100000, 2500, true, false, 1000, true);
    auto missing = p.read(17600000);
    TEST_ASSERT_FALSE(missing.valid);
    TEST_ASSERT_FALSE(missing.ready);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(highmotion::Reason::INTERVAL), static_cast<int>(missing.lastRejection));
    runPulse(p, 18100000, 12000);
    TEST_ASSERT_TRUE(p.read(30100000).valid);
    TEST_ASSERT_FLOAT_WITHIN(20, 1000, p.read(30100000).ibiMs);
}
void test_new_faster_rhythm_can_acquire_without_using_old_intervals() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000);
    runPulse(p, 15100000, 15000, true, false, 600);
    auto r = p.read(30100000);
    TEST_ASSERT_TRUE_MESSAGE(r.valid, highmotion::reasonName(r.reason));
    TEST_ASSERT_FLOAT_WITHIN(2, 100, r.hr);
    TEST_ASSERT_FLOAT_WITHIN(20, 600, r.ibiMs);
}
void test_alignment_interpolates_brackets_but_never_extrapolates() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    p.pushImu(100000, 0, 0, 1, true);
    p.pushImu(120000, 0.02f, 0, 1, true);
    p.beginBatch(110000, 1);
    p.pushPpg(50000);
    p.poll(120000);
    TEST_ASSERT_TRUE(p.read(120000).aligned);
    p.beginBatch(125000, 1);
    p.pushPpg(50000);
    p.poll(130000);
    TEST_ASSERT_TRUE(p.read(130000).aligned);
    p.poll(170000);
    auto r = p.read(170000);
    TEST_ASSERT_FALSE(r.aligned);
    TEST_ASSERT_EQUAL_UINT32(1, r.alignmentDrops);
}
void test_classifier_flapping_and_fifo_jitter_do_not_prevent_acquisition() {
    highmotion::Processor p;
    unsigned validReports = 0, highReports = 0;
    for (unsigned ms = 0; ms < 20000; ms += 10) {
        uint32_t t = 100000 + ms * 1000;
        bool high = (ms / 1000) % 2 == 0;
        p.setActive(high, true, t);
        p.pushImu(t, 0, 0, 1, true);
        float position = (ms % 600) * 0.001f;
        float pulse = position < 0.4f ? sinf(position * 3.14159265f / 0.4f) : 0;
        uint32_t received = t + ((ms / 10) % 2 ? 6000 : 1000);
        p.beginBatch(received, 1);
        p.pushPpg(50000 - 100 * pulse);
        p.poll(received);
        if (ms >= 10000 && high) {
            ++highReports;
            auto r = p.read(received);
            if (r.valid) { ++validReports; TEST_ASSERT_FLOAT_WITHIN(3, 100, r.hr); }
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(highReports * 9 / 10, validReports);
    TEST_ASSERT_EQUAL_UINT32(0, p.read(20100000).alignmentDrops);
}
void test_small_secondary_peaks_do_not_erase_fresh_hr() {
    highmotion::Processor p;
    unsigned validReports = 0;
    for (unsigned ms = 0; ms < 20000; ms += 10) {
        uint32_t t = 100000 + ms * 1000;
        p.setActive(true, true, t);
        p.pushImu(t, 0, 0, 1, true);
        float position = (ms % 1000) * 0.001f;
        float pulse = position < 0.4f ? sinf(position * 3.14159265f / 0.4f) : 0;
        if (position >= 0.55f && position < 0.61f)
            pulse += 0.06f * sinf((position - 0.55f) * 3.14159265f / 0.06f);
        p.beginBatch(t, 1);
        p.pushPpg(50000 - 100 * pulse);
        p.poll(t);
        if (ms >= 10000 && p.read(t).valid) ++validReports;
    }
    auto r = p.read(20100000);
    TEST_ASSERT_GREATER_THAN_UINT32(0, r.rejected);
    TEST_ASSERT_GREATER_THAN_UINT32(900, validReports);
    TEST_ASSERT_FLOAT_WITHIN(2, 60, r.hr);
}
void test_stale_rhythm_does_not_restart_filter_warmup() {
    highmotion::Processor p;
    p.setActive(true, true, 100000);
    runPulse(p, 100000, 15000);
    for (unsigned ms = 0; ms < 3000; ms += 10) {
        uint32_t t = 15100000 + ms * 1000;
        p.pushImu(t, 0, 0, 1, true);
        p.beginBatch(t, 1); p.pushPpg(50000); p.poll(t);
    }
    TEST_ASSERT_FALSE(p.read(18100000).valid);
    runPulse(p, 18100000, 4500);
    auto r = p.read(22600000);
    TEST_ASSERT_TRUE_MESSAGE(r.valid, highmotion::reasonName(r.reason));
    TEST_ASSERT_FLOAT_WITHIN(2, 60, r.hr);
}
void test_pulse_ripples_do_not_split_the_systolic_rise() {
    highmotion::Processor p;
    unsigned validCount = 0;
    for (unsigned ms = 0; ms < 20000; ms += 10) {
        uint32_t t = 100000 + ms * 1000;
        p.setActive(true, true, t);
        p.pushImu(t, 0, 0, 1, true);
        float position = (ms % 600) * 0.001f;
        float pulse = position < 0.4f ? sinf(position * 3.14159265f / 0.4f) : 0;
        float ripple = 8 * sinf(ms * 0.001f * 6.2831853f * 14);
        p.beginBatch(t, 1); p.pushPpg(50000 - 100 * pulse + ripple); p.poll(t);
        if (ms > 10000 && p.read(t).valid) {
            ++validCount;
            TEST_ASSERT_FLOAT_WITHIN(5, 100, p.read(t).hr);
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(850, validCount);
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_regular_pulses_acquire_measured_intervals_and_expire);
    RUN_TEST(test_motion_transitions_keep_shadow_warm_but_contact_clears_history);
    RUN_TEST(test_missing_imu_and_large_batches_cannot_publish);
    RUN_TEST(test_wraparound_and_motion_locked_pulses);
    RUN_TEST(test_overflow_and_nonfinite_samples_fail_closed);
    RUN_TEST(test_missing_pulse_requires_reacquisition_without_splitting_ibi);
    RUN_TEST(test_new_faster_rhythm_can_acquire_without_using_old_intervals);
    RUN_TEST(test_alignment_interpolates_brackets_but_never_extrapolates);
    RUN_TEST(test_classifier_flapping_and_fifo_jitter_do_not_prevent_acquisition);
    RUN_TEST(test_small_secondary_peaks_do_not_erase_fresh_hr);
    RUN_TEST(test_stale_rhythm_does_not_restart_filter_warmup);
    RUN_TEST(test_pulse_ripples_do_not_split_the_systolic_rise);
    return UNITY_END();
}
