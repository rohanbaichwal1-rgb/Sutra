#include <unity.h>
#include <PdrObserver.h>
#include <PdrPulseAmplitude.h>
#include <math.h>
#include <initializer_list>

void setUp() {}
void tearDown() {}
const pdr::Context REST = {true, true};

// Irregular timestamps; independently controlled amplitude and IBI modulation.
struct Stream {
    pdr::Observer observer;
    uint32_t base = 0, elapsed = 0, nextBeat = 0;
    uint32_t skipFrom = 0, skipUntil = 0;
    void run(uint32_t end, float avRate = 12, float fvRate = 12,
             pdr::Context context = REST, float avDepth = 0.12f, float fvDepth = 80) {
        for (; elapsed <= end; elapsed += 100) {
            while (nextBeat <= elapsed) {
                float phase = 6.283185307f*nextBeat/60000.0f;
                float ibi = 800 + fvDepth*sinf(phase*fvRate);
                if (nextBeat < skipFrom || nextBeat >= skipUntil)
                    observer.addBeat(base+nextBeat, 1000*(1+avDepth*sinf(phase*avRate)), ibi);
                nextBeat += (uint32_t)roundf(ibi);
            }
            observer.update(base+elapsed, context);
        }
    }
};

void test_full_45_second_window_then_dual_channel_rate() {
    Stream s; s.run(44000);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_TRUE(isnan(s.observer.result().rate));
    s.run(48000);
    const auto r = s.observer.result();
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.rate);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.riav);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.rifv);
    TEST_ASSERT_TRUE(r.quality >= pdr::MIN_QUALITY && r.quality <= 1);
    TEST_ASSERT_EQUAL_INT((int)pdr::ChannelStatus::OK,(int)r.av.status);
    TEST_ASSERT_TRUE(r.coverage >= pdr::MIN_COVERAGE);
}
void test_flat_signals_unavailable() {
    Stream s; s.run(70000,12,12,REST,0,0);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_TRUE(isnan(s.observer.result().rate));
    TEST_ASSERT_TRUE(isnan(s.observer.result().riav));
    TEST_ASSERT_TRUE(isnan(s.observer.result().rifv));
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::WEAK,(int)s.observer.result().status);
    TEST_ASSERT_TRUE(isnan(s.observer.result().quality));
    TEST_ASSERT_EQUAL_INT((int)pdr::ChannelStatus::LOW_MODULATION,(int)s.observer.result().av.status);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,0,s.observer.result().av.modulation);
}
void test_existing_single_channel_fallback() {
    Stream av; av.run(70000,12,12,REST,0.12f,0);
    TEST_ASSERT_TRUE(av.observer.result().valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,av.observer.result().rate);
    TEST_ASSERT_TRUE(isfinite(av.observer.result().riav));
    TEST_ASSERT_TRUE(isnan(av.observer.result().rifv));
    Stream fv; fv.run(70000,12,12,REST,0,80);
    TEST_ASSERT_TRUE(fv.observer.result().valid);
    TEST_ASSERT_TRUE(isnan(fv.observer.result().riav));
    TEST_ASSERT_TRUE(isfinite(fv.observer.result().rifv));
}
void test_conflicting_channels_rejected_but_visible() {
    Stream s; s.run(70000,12,20);
    const auto r = s.observer.result();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(isnan(r.rate));
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.riav);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,20,r.rifv);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::DISAGREE,(int)r.status);
}
void test_nearby_independent_rates_fused() {
    Stream s; s.run(70000,14,15);
    const auto r = s.observer.result();
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,14,r.riav);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,15,r.rifv);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,14.5f,r.rate);
}
void test_band_endpoints() {
    for (float rate : {6.0f,30.0f}) {
        Stream s; s.run(70000,rate,rate);
        TEST_ASSERT_TRUE(s.observer.result().valid);
        TEST_ASSERT_FLOAT_WITHIN(0.6f,rate,s.observer.result().rate);
    }
}
void test_out_of_band_modulation_is_unavailable() {
    for (float rate : {3.0f,36.0f}) {
        Stream s; s.run(70000,rate,rate);
        TEST_ASSERT_FALSE(s.observer.result().valid);
        TEST_ASSERT_TRUE(isnan(s.observer.result().rate));
    }
}
void test_motion_preserves_window_but_excludes_bad_interval() {
    Stream s; s.run(49000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    s.run(50000,12,12,{true,false});
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::MOTION,(int)s.observer.result().status);
    TEST_ASSERT_TRUE(isnan(s.observer.result().rate));
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    TEST_ASSERT_EQUAL_UINT32(0,s.observer.result().resets);
    s.run(56000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_TRUE(s.observer.result().coverage < 0.97f);
}
void test_initial_window_clock_survives_a_signal_pause() {
    Stream s; s.run(20000);
    s.run(21000,12,12,{false,true});
    TEST_ASSERT_EQUAL_UINT32(21000,s.observer.result().windowMs);
    s.run(48000);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_TRUE(s.observer.result().coverage < 0.97f);
}
void test_long_motion_expires_history_and_cannot_fill_coverage() {
    Stream s; s.run(70000);
    s.run(130000,18,18,{true,false},5,250); // These samples must never be stored.
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    s.run(136000);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::GAP,(int)s.observer.result().status);
    TEST_ASSERT_TRUE(s.observer.result().coverage < 0.20f);
    s.run(178000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,s.observer.result().rate);
}
void test_contact_discards_samples_not_completed_window_clock() {
    Stream s; s.run(70000);
    s.observer.contactChanged(70100);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    s.run(76000);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::GAP,(int)s.observer.result().status);
    TEST_ASSERT_TRUE(s.observer.result().coverage < 0.20f);
    s.run(118000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
}
void test_completed_window_stays_full_across_wrap_and_pause() {
    Stream s; s.base=UINT32_MAX-80000; s.run(79000);
    s.run(81000,12,12,{true,false});
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    s.run(88000);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    TEST_ASSERT_TRUE(s.observer.result().valid);
}
void test_signal_and_contact_reset() {
    Stream s; s.run(70000);
    s.observer.update(70100,{false,true});
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::SIGNAL,(int)s.observer.result().status);
    Stream contact; contact.run(70000);
    contact.observer.contactChanged();
    TEST_ASSERT_FALSE(contact.observer.result().valid);
    TEST_ASSERT_TRUE(isnan(contact.observer.result().riav));
}
void test_report_and_beat_gaps_invalidate() {
    Stream s; s.run(70000);
    s.observer.update(80000,REST);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    Stream beat; beat.run(70000);
    beat.observer.addBeat(75000,1000,800);
    TEST_ASSERT_FALSE(beat.observer.result().valid);
    Stream lost; lost.run(70000);
    lost.observer.dataLost();
    TEST_ASSERT_FALSE(lost.observer.result().valid);
}
void test_stale_beats_invalidate_between_analysis_ticks() {
    Stream s; s.run(70000);
    for (uint32_t t=70100; t<=73100; t+=100) s.observer.update(t,REST);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_TRUE(isnan(s.observer.result().rate));
}
void test_analysis_is_held_between_two_second_ticks() {
    Stream s; s.run(64000);
    const auto r = s.observer.result();
    TEST_ASSERT_TRUE(r.valid);
    s.run(65900,18,18);
    TEST_ASSERT_EQUAL_FLOAT(r.rate,s.observer.result().rate);
    TEST_ASSERT_EQUAL_FLOAT(r.quality,s.observer.result().quality);
}
void test_rolling_window_forgets_old_rate() {
    Stream s; s.run(70000,12,12); s.run(120000,18,18);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,18,s.observer.result().rate);
}
void test_timer_rollover() {
    Stream s; s.base=UINT32_MAX-30000; s.run(70000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,s.observer.result().rate);
}
void test_unstructured_noise_unavailable() {
    pdr::Observer observer;
    uint32_t random=0x12345;
    for (uint32_t t=0; t<=70000; t+=100) {
        if (t%800==0) {
            random=random*1664525UL+1013904223UL;
            float a=1000+(int)(random%201)-100;
            random=random*1664525UL+1013904223UL;
            float ibi=800+(int)(random%101)-50;
            observer.addBeat(t,a,ibi);
        }
        observer.update(t,REST);
    }
    TEST_ASSERT_FALSE(observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::ChannelStatus::LOW_PERIODICITY,(int)observer.result().av.status);
    TEST_ASSERT_TRUE(isfinite(observer.result().av.candidate));
    TEST_ASSERT_TRUE(observer.result().av.quality < pdr::MIN_QUALITY);
}
void test_low_modulation_candidates_are_diagnostic_only() {
    Stream s; s.run(70000,12,12,REST,0.001f,0.5f);
    const auto r=s.observer.result();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(isnan(r.rate) && isnan(r.riav) && isnan(r.rifv));
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.av.candidate);
    TEST_ASSERT_TRUE(r.av.quality >= pdr::MIN_QUALITY);
    TEST_ASSERT_TRUE(r.av.modulation < pdr::RIAV_MIN_STD);
    TEST_ASSERT_EQUAL_INT((int)pdr::ChannelStatus::LOW_MODULATION,(int)r.av.status);
}
void test_reset_diagnostics_survive_warmup_without_poll_inflation() {
    Stream s; s.run(70000);
    s.observer.update(80000,REST);
    auto r=s.observer.result();
    TEST_ASSERT_EQUAL_INT((int)pdr::ResetReason::CONTEXT_GAP,(int)r.lastReset);
    TEST_ASSERT_EQUAL_UINT32(10000,r.resetGapMs);
    TEST_ASSERT_EQUAL_UINT32(80000,r.resetTime);
    TEST_ASSERT_EQUAL_UINT32(1,r.resets);
    s.observer.update(80100,{false,true});
    s.observer.update(80200,{false,true});
    TEST_ASSERT_EQUAL_UINT32(1,s.observer.result().resets);
    TEST_ASSERT_EQUAL_INT((int)pdr::ResetReason::CONTEXT_GAP,(int)s.observer.result().lastReset);
    s.observer.update(80300,REST);
    s.observer.addBeat(80400,1000,800); // First returning interval is excluded.
    s.observer.addBeat(81200,1000,800);
    s.observer.update(82000,REST);
    r=s.observer.result();
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::GAP,(int)r.status);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,r.windowMs);
    TEST_ASSERT_EQUAL_UINT32(800,r.beatAgeMs);
    TEST_ASSERT_EQUAL_INT((int)pdr::ChannelStatus::NOT_ANALYZED,(int)r.av.status);
    TEST_ASSERT_EQUAL_FLOAT(0,r.coverage);
    TEST_ASSERT_EQUAL_UINT32(1,r.resets);
}
void test_distinct_reset_causes() {
    Stream beat; beat.run(70000);
    beat.observer.addBeat(75000,1000,800);
    TEST_ASSERT_EQUAL_UINT32(0,beat.observer.result().resets);
    TEST_ASSERT_TRUE(beat.observer.result().lastBeatGapMs > pdr::MAX_GAP_MS);
    Stream stale; stale.run(70000);
    for (uint32_t t=70100;t<=73100;t+=100) stale.observer.update(t,REST);
    TEST_ASSERT_EQUAL_UINT32(0,stale.observer.result().resets);
    TEST_ASSERT_TRUE(stale.observer.result().lastBeatGapMs > pdr::MAX_GAP_MS);
    Stream contact; contact.run(70000); contact.observer.contactChanged(70100);
    TEST_ASSERT_EQUAL_INT((int)pdr::ResetReason::CONTACT,(int)contact.observer.result().lastReset);
    Stream lost; lost.run(70000); lost.observer.dataLost(70100,pdr::ResetReason::QUEUE_OVERFLOW);
    TEST_ASSERT_EQUAL_INT((int)pdr::ResetReason::QUEUE_OVERFLOW,(int)lost.observer.result().lastReset);
    Stream stamp; stamp.run(70000); stamp.observer.addBeat(0,1000,800);
    TEST_ASSERT_EQUAL_INT((int)pdr::ResetReason::TIMESTAMP,(int)stamp.observer.result().lastReset);
    Stream signal; signal.run(70000); signal.observer.update(70100,{false,true});
    TEST_ASSERT_EQUAL_UINT32(0,signal.observer.result().resets);
    Stream motion; motion.run(70000); motion.observer.update(70100,{true,false});
    TEST_ASSERT_EQUAL_UINT32(0,motion.observer.result().resets);
}
void test_isolated_beat_gap_masks_samples_without_restarting_window() {
    Stream s; s.run(60000);
    TEST_ASSERT_TRUE(s.observer.result().valid);
    s.skipFrom = 61000; s.skipUntil = 64000;
    s.run(63900);
    TEST_ASSERT_FALSE(s.observer.result().valid); // Stale output must never survive.
    TEST_ASSERT_EQUAL_UINT32(0,s.observer.result().resets);
    TEST_ASSERT_EQUAL_UINT32(pdr::WINDOW_MS,s.observer.result().windowMs);
    s.run(68000);
    const auto r = s.observer.result();
    TEST_ASSERT_TRUE(r.valid); // Reuse real samples, not another 45 s warmup.
    TEST_ASSERT_FLOAT_WITHIN(0.6f,12,r.rate);
    TEST_ASSERT_TRUE(r.coverage >= pdr::MIN_COVERAGE && r.coverage < 0.95f);
    TEST_ASSERT_EQUAL_UINT32(0,r.resets);
    TEST_ASSERT_TRUE(r.lastBeatGapMs > pdr::MAX_GAP_MS);
}
void test_long_hole_cannot_be_interpolated_or_published() {
    Stream s; s.run(60000);
    s.skipFrom = 61000; s.skipUntil = 80000;
    s.run(79900);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    s.run(84000);
    TEST_ASSERT_FALSE(s.observer.result().valid);
    TEST_ASSERT_EQUAL_INT((int)pdr::Status::GAP,(int)s.observer.result().status);
    TEST_ASSERT_TRUE(s.observer.result().coverage < pdr::MIN_COVERAGE);
}
// Unlike Stream's legacy fixtures, IBI is the elapsed time ENDING at a beat.
// Respiratory FM is evaluated at the interval midpoint; AV has its own phase.
pdr::Result realisticRespiration(float rate, bool varying = false, bool noisy = false,
                                 unsigned seed = 123, float fvRate = 0, bool interference = false) {
    pdr::Observer observer;
    uint32_t previous = 0, next = 800;
    for (uint32_t now = 0; now <= 100000; now += 100) {
        while (next <= now) {
            const float t = next/1000.0f;
            const float phase = 6.283185307f*(rate*t/60 + (varying ? 0.0005f*t*t : 0));
            seed = seed*1664525UL+1013904223UL;
            const float amplitude = noisy ? 1000 + (int)(seed%401)-200 :
                1000 + 100*sinf(phase+0.9f) + 25*sinf(2*phase+0.3f) + 0.01f*t*t +
                (interference ? 95*sinf(6.283185307f*0.55f*t) : 0);
            observer.addBeat(next, amplitude, (float)(next-previous));
            previous = next;
            float interval = 800;
            for (unsigned k=0;k<6;++k) {
                const float mid = (previous+interval/2)/1000.0f;
                const float p = 6.283185307f*((fvRate > 0 ? fvRate : rate)*mid/60 +
                                               (varying ? 0.0005f*mid*mid : 0));
                interval = 800 + 65*sinf(p) + 15*sinf(2*p+0.4f);
            }
            if (noisy) { seed=seed*1664525UL+1013904223UL; interval=700+seed%201; }
            next += (uint32_t)roundf(interval);
        }
        observer.update(now, REST);
    }
    return observer.result();
}
void test_realistic_interval_timing_phase_and_harmonics() {
    for (float rate : {6.5f, 14.2f, 22.5f, 28.0f}) {
        const auto r = realisticRespiration(rate);
        TEST_ASSERT_TRUE(r.valid);
        TEST_ASSERT_FLOAT_WITHIN(0.8f,rate,r.rate);
    }
}
void test_slowly_changing_breathing_is_not_required_to_be_one_fixed_sine() {
    const auto r = realisticRespiration(12, true);
    TEST_ASSERT_TRUE(r.valid);
    // Last 45 s: instantaneous rate 15.3..18; midpoint about 16.65 brpm.
    TEST_ASSERT_FLOAT_WITHIN(1.0f,16.65f,r.rate);
}
void test_multiple_independent_noise_windows_stay_invalid() {
    for (unsigned seed=1;seed<=30;++seed)
        TEST_ASSERT_FALSE(realisticRespiration(12,false,true,seed).valid);
    TEST_ASSERT_FALSE(realisticRespiration(12,false,false,123,22).valid);
}
void test_respiration_with_out_of_band_interference() {
    const auto r = realisticRespiration(14.2f,false,false,123,0,true);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.8f,14.2f,r.rate);
}
void test_pulse_height_is_baseline_independent_and_local() {
    pdr::PulseAmplitude pulse;
    for (float baseline : {-70.0f, 0.0f, 50.0f}) {
        pulse.reset();
        pulse.observe(baseline-30,100,false);
        pulse.observe(baseline-20,200,false);
        pulse.observe(baseline+10,300,true);
        pulse.observe(baseline+40,400,true);
        pulse.observe(baseline+25,410,true);
        pulse.observe(baseline+5,500,false);
        TEST_ASSERT_FLOAT_WITHIN(0.001f,70,pulse.amplitude());
        // A later smaller pulse must use its own trough, not the previous one.
        pulse.observe(baseline-10,600,false);
        pulse.observe(baseline+20,700,true);
        TEST_ASSERT_FLOAT_WITHIN(0.001f,30,pulse.amplitude());
    }
    pulse.reset();
    pulse.observe(10,0,true); // No preceding trough.
    TEST_ASSERT_TRUE(isnan(pulse.amplitude()));
    pulse.observe(NAN,10,true);
    TEST_ASSERT_TRUE(isnan(pulse.amplitude()));
    pulse.observe(-30,100,false);
    pulse.observe(40,2001,true); // Stale trough cannot be reused.
    TEST_ASSERT_TRUE(isnan(pulse.amplitude()));
}
void test_near_band_leakage_and_competing_rates_stay_invalid() {
    for (float rate : {5.5f,30.5f,33.0f}) {
        Stream s; s.run(100000,rate,rate);
        TEST_ASSERT_FALSE(s.observer.result().valid);
    }
    pdr::Observer observer;
    for (uint32_t now=0;now<=100000;now+=100) {
        if (now%800==0) {
            const float phase=6.283185307f*now/60000.0f;
            const float modulation=sinf(12*phase)+sinf(22*phase);
            observer.addBeat(now,1000+100*modulation,800+70*modulation);
        }
        observer.update(now,REST);
    }
    TEST_ASSERT_FALSE(observer.result().valid);
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_initial_window_clock_survives_a_signal_pause);
    RUN_TEST(test_long_motion_expires_history_and_cannot_fill_coverage);
    RUN_TEST(test_contact_discards_samples_not_completed_window_clock);
    RUN_TEST(test_completed_window_stays_full_across_wrap_and_pause);
    RUN_TEST(test_pulse_height_is_baseline_independent_and_local);
    RUN_TEST(test_near_band_leakage_and_competing_rates_stay_invalid);
    RUN_TEST(test_realistic_interval_timing_phase_and_harmonics);
    RUN_TEST(test_slowly_changing_breathing_is_not_required_to_be_one_fixed_sine);
    RUN_TEST(test_multiple_independent_noise_windows_stay_invalid);
    RUN_TEST(test_respiration_with_out_of_band_interference);
    RUN_TEST(test_isolated_beat_gap_masks_samples_without_restarting_window);
    RUN_TEST(test_long_hole_cannot_be_interpolated_or_published);
    RUN_TEST(test_low_modulation_candidates_are_diagnostic_only);
    RUN_TEST(test_reset_diagnostics_survive_warmup_without_poll_inflation);
    RUN_TEST(test_distinct_reset_causes);
    RUN_TEST(test_full_45_second_window_then_dual_channel_rate);
    RUN_TEST(test_flat_signals_unavailable);
    RUN_TEST(test_existing_single_channel_fallback);
    RUN_TEST(test_conflicting_channels_rejected_but_visible);
    RUN_TEST(test_nearby_independent_rates_fused);
    RUN_TEST(test_band_endpoints);
    RUN_TEST(test_out_of_band_modulation_is_unavailable);
    RUN_TEST(test_motion_preserves_window_but_excludes_bad_interval);
    RUN_TEST(test_signal_and_contact_reset);
    RUN_TEST(test_report_and_beat_gaps_invalidate);
    RUN_TEST(test_stale_beats_invalidate_between_analysis_ticks);
    RUN_TEST(test_analysis_is_held_between_two_second_ticks);
    RUN_TEST(test_rolling_window_forgets_old_rate);
    RUN_TEST(test_timer_rollover);
    RUN_TEST(test_unstructured_noise_unavailable);
    return UNITY_END();
}
