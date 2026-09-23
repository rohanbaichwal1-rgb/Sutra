#include <unity.h>
#include <HapticPatterns.h>
#include <HapticPlayback.h>
#include <HapticArray.h>
#include <RmssdReferences.h>
#include <string.h>
#include <vector>

using namespace haptics;

void setUp() {}
void tearDown() {}

void assertFrame(const Frame &frame, unsigned left, unsigned center, unsigned right)
{
    TEST_ASSERT_EQUAL_UINT(left, frame.level[0]);
    TEST_ASSERT_EQUAL_UINT(center, frame.level[1]);
    TEST_ASSERT_EQUAL_UINT(right, frame.level[2]);
}

void test_p2_inhale_boundaries_and_ramps()
{
    Patterns p;
    TEST_ASSERT_TRUE(p.startP2(100));
    assertFrame(p.frame(), 100, 0, 0);
    assertFrame(p.update(750), 300, 0, 0);
    assertFrame(p.update(1399), 499, 0, 0);
    assertFrame(p.update(1400), 0, 0, 100);
    assertFrame(p.update(2050), 0, 0, 300);
    assertFrame(p.update(2699), 0, 0, 499);
    assertFrame(p.update(2700), 0, 600, 0);
    assertFrame(p.update(4099), 0, 600, 0);
}

void test_p2_exhale_and_ten_second_completion()
{
    Patterns p;
    p.startP2(0);
    TEST_ASSERT_EQUAL_UINT32(10000, p.durationMs());
    assertFrame(p.update(4000), 0, 600, 0);
    assertFrame(p.update(5000), 0, 450, 0);
    assertFrame(p.update(6000), 300, 0, 300);
    assertFrame(p.update(8000), 150, 0, 150);
    assertFrame(p.update(9999), 1, 0, 1);
    TEST_ASSERT_TRUE(p.active());
    assertFrame(p.update(10000), 0, 0, 0);
    TEST_ASSERT_FALSE(p.active());
}

void test_p1_independent_bursts_intensity_and_duration_limits()
{
    Patterns p;
    p.startP1(0, 12345);
    TEST_ASSERT_EQUAL_UINT32(5000, p.durationMs());
    bool on[3] = {false, false, false};
    uint32_t changed[3] = {0, 0, 0};
    unsigned bursts[3] = {0, 0, 0};
    uint32_t firstDuration = 0;
    bool varied = false;
    bool overlap = false;
    for (uint32_t t = 0; t <= 5000; ++t)
    {
        const Frame &f = p.update(t);
        unsigned active = 0;
        for (unsigned m = 0; m < 3; ++m)
        {
            TEST_ASSERT_TRUE(f.level[m] == 0 || f.level[m] == 900);
            bool next = f.level[m] != 0;
            if (next) ++active;
            if (next == on[m]) continue;
            uint32_t duration = t - changed[m];
            if (on[m])
            {
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(60, duration);
                TEST_ASSERT_LESS_OR_EQUAL_UINT32(250, duration);
                if (!firstDuration) firstDuration = duration;
                else if (duration != firstDuration) varied = true;
            }
            else
            {
                if (bursts[m] > 0)
                {
                    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(50, duration);
                    TEST_ASSERT_LESS_OR_EQUAL_UINT32(450, duration);
                }
                ++bursts[m];
            }
            on[m] = next;
            changed[m] = t;
        }
        if (active > 1) overlap = true;
    }
    for (unsigned m = 0; m < 3; ++m) TEST_ASSERT_GREATER_THAN_UINT(3, bursts[m]);
    TEST_ASSERT_TRUE(varied);
    TEST_ASSERT_TRUE(overlap);
    TEST_ASSERT_FALSE(p.active());
    assertFrame(p.frame(), 0, 0, 0);
}

void test_p2_preempts_p1_and_p1_cannot_interrupt_p2()
{
    Patterns p;
    p.startP1(0, 17);
    TEST_ASSERT_TRUE(p.startP2(123));
    assertFrame(p.frame(), 100, 0, 0);
    TEST_ASSERT_FALSE(p.startP1(124, 18));
    assertFrame(p.update(8123), 150, 0, 150);
    p.stop();
    assertFrame(p.frame(), 0, 0, 0);
}

void test_patterns_handle_late_updates_and_millis_wrap()
{
    Patterns p;
    p.startP2(UINT32_MAX - 3999, 1);
    assertFrame(p.update(0), 0, 600, 0);
    assertFrame(p.update(4000), 150, 0, 150);
    assertFrame(p.update(6000), 0, 0, 0);
    p.startP1(UINT32_MAX - 99, 9876);
    assertFrame(p.update(6000), 0, 0, 0);
    TEST_ASSERT_FALSE(p.active());
    TEST_ASSERT_FALSE(p.startP2(0, 0));
    TEST_ASSERT_FALSE(p.startP1(0, 1, 59));
}

class FakeBus : public I2cBus
{
public:
    uint8_t registers[8][256] = {};
    uint8_t mux = 0;
    int nackChannel = -1;
    int failedSelect = -1;
    bool failReads = false;
    bool nackMux = false;
    bool loseStartAck = false;
    bool multiChannelWrite = false;
    struct Write { unsigned channel; uint8_t reg; uint8_t value; };
    std::vector<Write> writes;

    FakeBus()
    {
        for (auto &r : registers)
        {
            r[0] = 0xBA;
            r[0x08] = 0x40;
            r[0x0A] = 0x21;
            r[0x0B] = 0x4F;
            r[0x0C] = 0x5A;
            r[0x0D] = 0x78;
            r[0x0E] = 0x17;
            r[0x0F] = 0x01;
            r[0x10] = 0x0D;
            r[0x13] = 0x1E;
            r[0x14] = 0x01;
            r[0x24] = 0x08;
        }
    }
    int channel() const
    {
        for (int ch = 0; ch < 8; ++ch) if (mux == (1U << ch)) return ch;
        return -1;
    }
    bool write(uint8_t address, const uint8_t *data, size_t size) override
    {
        if (address == 0x70 && size == 1)
        {
            if (nackMux) return false;
            if (failedSelect >= 0 && data[0] == (1U << failedSelect)) return false;
            mux = data[0];
            return true;
        }
        if (address != 0x4A || size != 2) return false;
        int ch = channel();
        if (ch < 0) { multiChannelWrite = true; return false; }
        if (ch == nackChannel) return false;
        writes.push_back({static_cast<unsigned>(ch), data[0], data[1]});
        if (data[0] == 0x03) registers[ch][data[0]] &= ~data[1];
        else registers[ch][data[0]] = data[1];
        if (loseStartAck && data[0] == 0x22 && data[1] == 1)
        {
            nackChannel = ch; // command took effect, but bus contact was lost
            return false;
        }
        return true;
    }
    bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) override
    {
        int ch = channel();
        if (failReads || ch < 0 || ch == nackChannel || address != 0x4A) return false;
        value = registers[ch][reg];
        return true;
    }
};

void assertStopped(const FakeBus &bus, unsigned ch)
{
    TEST_ASSERT_EQUAL_UINT8(0, bus.registers[ch][0x23]);
    TEST_ASSERT_EQUAL_UINT8(0, bus.registers[ch][0x22]);
}

void test_driver_initializes_each_mux_channel_without_replacing_motor_profile()
{
    FakeBus bus;
    Array driver(bus);
    TEST_ASSERT_TRUE(driver.begin());
    TEST_ASSERT_TRUE(driver.ready());
    TEST_ASSERT_EQUAL_UINT8(0, bus.mux);
    for (unsigned ch = 0; ch < 3; ++ch)
    {
        assertStopped(bus, ch);
        TEST_ASSERT_EQUAL_UINT8(0x5A, bus.registers[ch][0x0C]);
        TEST_ASSERT_EQUAL_UINT8(0x78, bus.registers[ch][0x0D]);
        TEST_ASSERT_EQUAL_UINT8(0x17, bus.registers[ch][0x0E]);
        TEST_ASSERT_EQUAL_UINT8(0x1E, bus.registers[ch][0x13]);
        TEST_ASSERT_EQUAL_UINT8(0, bus.registers[ch][0x08] & 0x40);
    }
    for (const auto &w : bus.writes)
        TEST_ASSERT_FALSE(w.reg >= 0x0A && w.reg <= 0x10);
    TEST_ASSERT_FALSE(bus.multiChannelWrite);
}

void test_driver_outer_motors_continue_together_after_mux_disconnect()
{
    FakeBus bus;
    Array driver(bus);
    Patterns p;
    driver.begin();
    p.startP2(0);
    TEST_ASSERT_TRUE(driver.apply(p.update(6000)));
    TEST_ASSERT_EQUAL_UINT8(38, bus.registers[0][0x23]);
    TEST_ASSERT_EQUAL_UINT8(0, bus.registers[1][0x23]);
    TEST_ASSERT_EQUAL_UINT8(38, bus.registers[2][0x23]);
    TEST_ASSERT_EQUAL_UINT8(1, bus.registers[0][0x22]);
    TEST_ASSERT_EQUAL_UINT8(1, bus.registers[2][0x22]);
    TEST_ASSERT_EQUAL_UINT8(0, bus.mux);
    TEST_ASSERT_TRUE(driver.stopAll());
    for (unsigned ch = 0; ch < 3; ++ch) assertStopped(bus, ch);
    TEST_ASSERT_EQUAL_UINT8(114, Array::amplitudeCode(900));
    TEST_ASSERT_EQUAL_UINT8(76, Array::amplitudeCode(600));
    TEST_ASSERT_EQUAL_UINT8(127, Array::amplitudeCode(65535));
}

void test_single_motor_configuration_only_requires_channel_zero()
{
    FakeBus bus;
    ArrayConfig config;
    config.motorCount = 1;
    config.channels[0] = 0;
    Array driver(bus, config);
    Frame frame;
    frame.level[0] = 900;
    frame.level[1] = frame.level[2] = 900; // ignored: these branches are absent

    TEST_ASSERT_TRUE(driver.begin());
    TEST_ASSERT_EQUAL_UINT8(1, driver.motorCount());
    TEST_ASSERT_TRUE(driver.apply(frame));
    TEST_ASSERT_EQUAL_UINT8(114, bus.registers[0][0x23]);
    TEST_ASSERT_EQUAL_UINT8(0, bus.registers[1][0x23]);
    TEST_ASSERT_EQUAL_UINT8(0, bus.registers[2][0x23]);
    TEST_ASSERT_TRUE(driver.stopAll());
    assertStopped(bus, 0);
}

void test_scaled_intensity_preserves_p1_p2_envelopes()
{
    FakeBus bus;
    ArrayConfig config;
    config.motorCount = 1;
    config.outputScalePercent = 40;
    Array driver(bus, config);
    Patterns p;
    TEST_ASSERT_TRUE(driver.begin());
    for (unsigned protocol = 0; protocol < 2; ++protocol)
    {
        TEST_ASSERT_TRUE(protocol == 0 ? p.startP1(0, 12345) : p.startP2(0));
        const uint32_t duration = protocol == 0 ? 5000 : 10000;
        TEST_ASSERT_EQUAL_UINT32(duration, p.durationMs());
        bool sawOn = false;
        bool sawOff = false;
        for (uint32_t t = 0; t <= duration; ++t)
        {
            const Frame &frame = p.update(t);
            const uint8_t expected = (frame.level[0] * 508UL + 5000UL) / 10000UL;
            TEST_ASSERT_TRUE(driver.apply(frame));
            TEST_ASSERT_EQUAL_UINT8(expected, bus.registers[0][0x23]);
            TEST_ASSERT_EQUAL_UINT8(0, bus.mux);
            if (expected) sawOn = true;
            else sawOff = true;
        }
        TEST_ASSERT_TRUE(sawOn);
        TEST_ASSERT_TRUE(sawOff);
        TEST_ASSERT_FALSE(p.active());
        TEST_ASSERT_TRUE(driver.stopAll());
        assertStopped(bus, 0);
    }
    for (const auto &w : bus.writes)
        TEST_ASSERT_EQUAL_UINT(0, w.channel);

    TEST_ASSERT_TRUE(p.startP2(20000));
    TEST_ASSERT_TRUE(driver.apply(p.frame()));
    bus.registers[0][0x03] = 0x80;
    TEST_ASSERT_FALSE(driver.pollFaults());
    TEST_ASSERT_FALSE(driver.ready());
    assertStopped(bus, 0);
}

void test_intensity_scale_rejects_out_of_range_percent()
{
    FakeBus bus;
    ArrayConfig config;
    config.outputScalePercent = 101;
    Array driver(bus, config);
    TEST_ASSERT_FALSE(driver.begin());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ArrayError::CONFIG), static_cast<int>(driver.error()));
    TEST_ASSERT_TRUE(bus.writes.empty());
}

void test_intensity_scale_mapping_and_limits()
{
    TEST_ASSERT_EQUAL_UINT8(0, Array::amplitudeCode(0, 40));
    TEST_ASSERT_EQUAL_UINT8(5, Array::amplitudeCode(100, 40));
    TEST_ASSERT_EQUAL_UINT8(15, Array::amplitudeCode(300, 40));
    TEST_ASSERT_EQUAL_UINT8(25, Array::amplitudeCode(500, 40));
    TEST_ASSERT_EQUAL_UINT8(30, Array::amplitudeCode(600, 40));
    TEST_ASSERT_EQUAL_UINT8(46, Array::amplitudeCode(900, 40));
    TEST_ASSERT_EQUAL_UINT8(51, Array::amplitudeCode(1000, 40));
    TEST_ASSERT_EQUAL_UINT8(51, Array::amplitudeCode(65535, 40));
    TEST_ASSERT_EQUAL_UINT8(0, Array::amplitudeCode(1000, 0));
    TEST_ASSERT_EQUAL_UINT8(127, Array::amplitudeCode(1000, 100));
}

void test_driver_stops_old_motor_before_sweep_handoff()
{
    FakeBus bus;
    Array driver(bus);
    Patterns p;
    driver.begin();
    p.startP2(0);
    driver.apply(p.update(2590)); // right on
    bus.writes.clear();
    driver.apply(p.update(2600)); // center on, right off
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(3, bus.writes.size());
    TEST_ASSERT_EQUAL_UINT(2, bus.writes[0].channel);
    TEST_ASSERT_EQUAL_UINT8(0x23, bus.writes[0].reg);
    TEST_ASSERT_EQUAL_UINT8(0, bus.writes[0].value);
    TEST_ASSERT_EQUAL_UINT(1, bus.writes[1].channel);
}

void test_i2c_failure_latches_and_attempts_stop_on_other_channels()
{
    FakeBus bus;
    Array driver(bus);
    driver.begin();
    Frame f;
    f.level[0] = f.level[1] = f.level[2] = 900;
    driver.apply(f);
    bus.nackChannel = 1;
    f.level[1] = 300;
    TEST_ASSERT_FALSE(driver.apply(f));
    TEST_ASSERT_FALSE(driver.ready());
    TEST_ASSERT_TRUE(driver.shutdownPending());
    TEST_ASSERT_TRUE(driver.outputStopPending());
    assertStopped(bus, 0);
    assertStopped(bus, 2);
    TEST_ASSERT_EQUAL_UINT8(0, bus.mux);
    bus.nackChannel = -1;
    TEST_ASSERT_TRUE(driver.stopAll());
    TEST_ASSERT_FALSE(driver.shutdownPending());
    TEST_ASSERT_FALSE(driver.outputStopPending());
    TEST_ASSERT_FALSE(driver.ready()); // fault cannot silently restart a protocol
    assertStopped(bus, 1);
}

void test_failed_mux_selection_does_not_write_to_previous_motor()
{
    FakeBus bus;
    Array driver(bus);
    driver.begin();
    bus.failedSelect = 1;
    Frame f;
    f.level[1] = 900;
    TEST_ASSERT_FALSE(driver.apply(f));
    for (const auto &w : bus.writes)
        if (w.reg == 0x23) TEST_ASSERT_EQUAL_UINT8(0, w.value);
    TEST_ASSERT_FALSE(bus.multiChannelWrite);
}

void test_hardware_fault_stops_all_three_motors()
{
    FakeBus bus;
    Array driver(bus);
    driver.begin();
    Frame f;
    f.level[0] = f.level[1] = f.level[2] = 600;
    driver.apply(f);
    bus.registers[2][0x03] = 0x80;
    TEST_ASSERT_FALSE(driver.pollFaults());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ArrayError::DRIVER_FAULT), static_cast<int>(driver.error()));
    TEST_ASSERT_EQUAL_UINT8(3, driver.errorMotor());
    TEST_ASSERT_EQUAL_UINT8(0x80, driver.faultBits());
    for (unsigned ch = 0; ch < 3; ++ch) assertStopped(bus, ch);
}

void test_missing_driver_or_invalid_channel_mapping_is_not_ready()
{
    FakeBus bus;
    bus.registers[1][0] = 0;
    Array driver(bus);
    TEST_ASSERT_FALSE(driver.begin());
    TEST_ASSERT_FALSE(driver.ready());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ArrayError::CHIP_ID), static_cast<int>(driver.error()));
    ArrayConfig config;
    config.channels[1] = 0;
    Array duplicate(bus, config);
    TEST_ASSERT_FALSE(duplicate.begin());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ArrayError::CONFIG), static_cast<int>(duplicate.error()));
}

void test_supply_warning_is_reported_but_thermal_warning_stops()
{
    FakeBus bus;
    Array driver(bus);
    driver.begin();
    bus.registers[0][0x03] = 0x20;
    bus.registers[0][0x04] = 0xC0;
    TEST_ASSERT_TRUE(driver.pollFaults());
    TEST_ASSERT_EQUAL_UINT8(0xC0, driver.warningBits());
    bus.registers[0][0x03] = 0x20;
    bus.registers[0][0x04] = 0x08;
    TEST_ASSERT_FALSE(driver.pollFaults());
    TEST_ASSERT_FALSE(driver.ready());
}

void test_absent_haptics_at_startup_do_not_block_short_reference()
{
    // Reproduce both a missing mux and a missing DA7280 on one branch.
    for (unsigned scenario = 0; scenario < 2; ++scenario)
    {
        FakeBus bus;
        bus.nackMux = scenario == 0;
        bus.nackChannel = scenario == 1 ? 1 : -1;
        Array driver(bus);
        TEST_ASSERT_FALSE(driver.begin());
        TEST_ASSERT_TRUE(driver.shutdownPending());
        TEST_ASSERT_FALSE(driver.outputStopPending());
        rmssd::Monitor monitor;
        for (uint32_t t = 0; t <= 60000; t += 1000)
        {
            driver.stopAll(); // repeated failed cleanup does not start playback
            rmssd::Input input = {t, 79.0f, true, rmssd::Motion::LOW_MOTION, true,
                                  driver.outputStopPending(), false, 0.0f};
            monitor.update(input);
        }
        TEST_ASSERT_TRUE(monitor.shortAvailable());
        TEST_ASSERT_EQUAL_UINT32(60000, monitor.validDurationMs());
        TEST_ASSERT_FALSE(driver.ready()); // do not silently enable bad hardware
    }
}

void test_lost_playback_ack_blocks_collection_until_output_stops()
{
    FakeBus bus;
    Array driver(bus);
    TEST_ASSERT_TRUE(driver.begin());
    bus.loseStartAck = true;
    Frame f;
    f.level[1] = 600;
    TEST_ASSERT_FALSE(driver.apply(f));
    TEST_ASSERT_TRUE(driver.outputStopPending());
    TEST_ASSERT_EQUAL_UINT8(1, bus.registers[1][0x22]);
    rmssd::Monitor monitor;
    for (uint32_t t = 0; t <= 200000; t += 1000)
    {
        rmssd::Input input = {t, 79.0f, true, rmssd::Motion::LOW_MOTION, true,
                              driver.outputStopPending(), false, 0.0f};
        monitor.update(input);
    }
    TEST_ASSERT_EQUAL_UINT(0, monitor.sampleCount());
    TEST_ASSERT_FALSE(monitor.shortAvailable());
    bus.nackChannel = -1;
    TEST_ASSERT_TRUE(driver.stopAll());
    TEST_ASSERT_FALSE(driver.outputStopPending());
    assertStopped(bus, 1);
}

void test_boot_check_is_continuous_for_five_seconds_on_motor_one()
{
    FakeBus bus;
    ArrayConfig config;
    config.motorCount = 1;
    config.outputScalePercent = 40;
    Array driver(bus, config);
    Patterns p;
    PlaybackLog playback;
    TEST_ASSERT_TRUE(driver.begin());
    const uint32_t start = UINT32_MAX - 2000;
    TEST_ASSERT_TRUE(p.startBootTest(start));
    playback.started(Protocol::BOOT_TEST);
    TEST_ASSERT_FALSE(p.startBootTest(start)); // no duplicate start while running
    for (uint32_t elapsed = 0; elapsed < 5000; elapsed += 10)
    {
        assertFrame(p.update(start + elapsed), 1000, 0, 0);
        TEST_ASSERT_TRUE(driver.apply(p.frame()));
        TEST_ASSERT_TRUE(driver.pollFaults());
        TEST_ASSERT_EQUAL_UINT8(51, bus.registers[0][0x23]);
        TEST_ASSERT_EQUAL_UINT8(0, bus.mux);
    }
    assertFrame(p.update(start + 5000), 0, 0, 0);
    TEST_ASSERT_FALSE(p.active());
    TEST_ASSERT_TRUE(driver.stopAll());
    playback.finished(true);
    assertStopped(bus, 0);
    TEST_ASSERT_EQUAL_STRING("COMPLETED", playbackName(playback.state(Protocol::BOOT_TEST)));
    TEST_ASSERT_EQUAL_STRING("NOT_STARTED", playbackName(playback.state(Protocol::P1_FLUTTER)));
    TEST_ASSERT_EQUAL_STRING("NOT_STARTED", playbackName(playback.state(Protocol::P2_SWEEP)));
}

void test_boot_undervoltage_fails_and_cannot_restart()
{
    FakeBus bus;
    ArrayConfig config;
    config.motorCount = 1;
    config.outputScalePercent = 40;
    Array driver(bus, config);
    Patterns p;
    PlaybackLog playback;
    TEST_ASSERT_TRUE(driver.begin());
    p.startBootTest(0);
    TEST_ASSERT_TRUE(driver.apply(p.frame()));
    playback.started(Protocol::BOOT_TEST);
    bus.registers[0][0x03] = 0x02;
    TEST_ASSERT_FALSE(driver.pollFaults());
    p.stop();
    playback.finished(false);
    assertStopped(bus, 0);
    TEST_ASSERT_EQUAL_STRING("FAILED", playbackName(playback.state(Protocol::BOOT_TEST)));
    TEST_ASSERT_FALSE(driver.ready());
    TEST_ASSERT_FALSE(driver.apply(p.frame()));
}

void test_playback_outcomes_do_not_confuse_preemption_failure_and_completion()
{
    PlaybackLog playback;
    playback.started(Protocol::P1_FLUTTER);
    playback.started(Protocol::P2_SWEEP);
    TEST_ASSERT_EQUAL_STRING("PREEMPTED", playbackName(playback.state(Protocol::P1_FLUTTER)));
    playback.suppressed(Protocol::P1_FLUTTER);
    TEST_ASSERT_EQUAL_STRING("SUPPRESSED", playbackName(playback.state(Protocol::P1_FLUTTER)));
    TEST_ASSERT_EQUAL_STRING("STARTED", playbackName(playback.state(Protocol::P2_SWEEP)));
    playback.finished(false);
    TEST_ASSERT_EQUAL_STRING("FAILED", playbackName(playback.state(Protocol::P2_SWEEP)));
    playback.finished(true); // a later cleanup cannot turn failure into success
    TEST_ASSERT_EQUAL_STRING("FAILED", playbackName(playback.state(Protocol::P2_SWEEP)));
    playback.failedRequest(Protocol::P1_FLUTTER);
    TEST_ASSERT_EQUAL_STRING("FAILED", playbackName(playback.state(Protocol::P1_FLUTTER)));
    playback.started(Protocol::P1_FLUTTER);
    playback.finished(true);
    TEST_ASSERT_EQUAL_STRING("COMPLETED", playbackName(playback.state(Protocol::P1_FLUTTER)));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_check_is_continuous_for_five_seconds_on_motor_one);
    RUN_TEST(test_boot_undervoltage_fails_and_cannot_restart);
    RUN_TEST(test_playback_outcomes_do_not_confuse_preemption_failure_and_completion);
    RUN_TEST(test_p2_inhale_boundaries_and_ramps);
    RUN_TEST(test_p2_exhale_and_ten_second_completion);
    RUN_TEST(test_p1_independent_bursts_intensity_and_duration_limits);
    RUN_TEST(test_p2_preempts_p1_and_p1_cannot_interrupt_p2);
    RUN_TEST(test_patterns_handle_late_updates_and_millis_wrap);
    RUN_TEST(test_driver_initializes_each_mux_channel_without_replacing_motor_profile);
    RUN_TEST(test_driver_outer_motors_continue_together_after_mux_disconnect);
    RUN_TEST(test_single_motor_configuration_only_requires_channel_zero);
    RUN_TEST(test_scaled_intensity_preserves_p1_p2_envelopes);
    RUN_TEST(test_intensity_scale_rejects_out_of_range_percent);
    RUN_TEST(test_intensity_scale_mapping_and_limits);
    RUN_TEST(test_driver_stops_old_motor_before_sweep_handoff);
    RUN_TEST(test_i2c_failure_latches_and_attempts_stop_on_other_channels);
    RUN_TEST(test_failed_mux_selection_does_not_write_to_previous_motor);
    RUN_TEST(test_hardware_fault_stops_all_three_motors);
    RUN_TEST(test_missing_driver_or_invalid_channel_mapping_is_not_ready);
    RUN_TEST(test_supply_warning_is_reported_but_thermal_warning_stops);
    RUN_TEST(test_absent_haptics_at_startup_do_not_block_short_reference);
    RUN_TEST(test_lost_playback_ack_blocks_collection_until_output_stops);
    return UNITY_END();
}
