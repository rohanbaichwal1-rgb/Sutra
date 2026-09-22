#include <unity.h>
#include <TelemetryReport.h>
#include <math.h>
#include <string.h>
#include <string>

void setUp() {}
void tearDown() {}

telemetry::Snapshot sample() {
    telemetry::Snapshot s{};
    s.uptimeSeconds = 266;
    s.normalHr = 77.7f; s.normalIbi = 761;
    s.hr = 78; s.ibi = 770; s.rmssd = 80;
    s.session = 100; s.shortReference = 90; s.personal = NAN;
    s.finger = s.converged = s.stable = s.fresh = 1;
    s.imuOK = true;
    s.sessionValidMs = s.sessionTargetMs = 100000;
    s.sessionSamples = s.sessionTargetSamples = 20;
    strcpy(s.sessionState, "FROZEN"); strcpy(s.storageState, "SAVED");
    strcpy(s.shortState, "ADAPTING"); strcpy(s.shortGate, "OPEN");
    strcpy(s.hapticState, "IDLE"); strcpy(s.bleState, "CONNECTED");
    return s;
}

void test_each_group_is_one_complete_bounded_line() {
    auto s = sample();
    s.shortBreaks = UINT32_MAX;
    s.shortValidMs = s.shortBlockMs = UINT32_MAX;
    for (unsigned i = 0; i < static_cast<unsigned>(telemetry::Group::COUNT); ++i) {
        char line[telemetry::LINE_CAPACITY];
        auto group = static_cast<telemetry::Group>(i);
        TEST_ASSERT_TRUE(telemetry::formatLine(group,s,line,sizeof(line)));
        TEST_ASSERT_NOT_NULL(strstr(line,telemetry::groupName(group)));
        TEST_ASSERT_NOT_NULL(strstr(line,"[266s]"));
        TEST_ASSERT_EQUAL_PTR(line+strlen(line)-1,strchr(line,'\n'));
    }
}

void test_missing_readings_do_not_look_like_zero() {
    auto s = sample();
    s.hr = s.ibi = s.rmssd = NAN;
    char line[telemetry::LINE_CAPACITY];
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::LMS,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"HR:-- | IBI:-- | RMSSD:--"));
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::P1,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"Session:0/30s | Short:0/30s | 7Session:--"));
}

void test_signed_deviations_and_separate_protocol_timers() {
    auto s = sample();
    s.respiration.valid = s.respiration.baselineReady = true;
    s.respiration.deviation = -25;
    s.p1Ms[0] = 30000; s.p2Ms[1] = 95000;
    char line[telemetry::LINE_CAPACITY];
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::PDR,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"Dev:-25.0%"));
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::SESSION,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"Dev:20.0%"));
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::P1,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"Session:30/30s | Short:0/30s"));
    TEST_ASSERT_TRUE(telemetry::formatLine(telemetry::Group::P2,s,line,sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line,"Session:0/120s | Short:95/120s"));
}

void test_small_buffer_is_cleared_without_overrun() {
    char guarded[10]; memset(guarded,'X',sizeof(guarded));
    TEST_ASSERT_FALSE(telemetry::formatLine(telemetry::Group::PDR,sample(),guarded+1,8));
    TEST_ASSERT_EQUAL_CHAR('X',guarded[0]);
    TEST_ASSERT_EQUAL_CHAR('X',guarded[9]);
    TEST_ASSERT_EQUAL_CHAR(0,guarded[1]);
    TEST_ASSERT_FALSE(telemetry::formatLine(telemetry::Group::PDR,sample(),nullptr,0));
}

void test_mtu_fragmentation_preserves_all_lines_and_delimiters() {
    const uint16_t mtus[] = {0, 22, 23, 185, 247, 517, 65535};
    auto s = sample();
    std::string frame;
    for (unsigned i=0; i<static_cast<unsigned>(telemetry::Group::COUNT); ++i) {
        char line[telemetry::LINE_CAPACITY];
        TEST_ASSERT_TRUE(telemetry::formatLine(static_cast<telemetry::Group>(i),s,line,sizeof(line)));
        frame += line;
    }
    frame += '\n';
    for (auto mtu : mtus) {
        std::string received;
        const size_t limit = mtu >= 23 && mtu <= 517 ? mtu-3 : 20;
        while (received.size() < frame.size()) {
            size_t offset = received.size();
            size_t count = telemetry::notificationChunkSize(frame.size()-offset,mtu);
            TEST_ASSERT_TRUE(count > 0 && count <= limit);
            received.append(frame.data()+offset,count);
        }
        TEST_ASSERT_EQUAL_STRING(frame.c_str(),received.c_str());
        TEST_ASSERT_EQUAL_UINT(0,telemetry::notificationChunkSize(0,mtu));
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_each_group_is_one_complete_bounded_line);
    RUN_TEST(test_missing_readings_do_not_look_like_zero);
    RUN_TEST(test_signed_deviations_and_separate_protocol_timers);
    RUN_TEST(test_small_buffer_is_cleared_without_overrun);
    RUN_TEST(test_mtu_fragmentation_preserves_all_lines_and_delimiters);
    return UNITY_END();
}
