#pragma once
#include <stddef.h>
#include <stdint.h>
#include <PdrObserver.h>
#include <RmssdReferences.h>
#include <HapticPlayback.h>

namespace telemetry {
enum class Group : uint8_t { NORMAL, LMS, PDR, P1, P2, SESSION, SHORT, PERSONAL, SYSTEM, COUNT };
constexpr size_t LINE_CAPACITY = 512;
struct Snapshot {
    float hr, ibi, rmssd, session, shortReference, personal;
    uint32_t uptimeSeconds, maxLoopGapUs;
    uint8_t finger, motion, converged, stable, fresh, savedSessions, episode;
    pdr::Result respiration;
    float normalHr = 0, normalIbi = 0;
    bool imuOK = false, p1Triggered = false, p2Triggered = false;
    bool hapticFault = false;
    haptics::Playback p1Playback = haptics::Playback::NOT_STARTED;
    haptics::Playback p2Playback = haptics::Playback::NOT_STARTED;
    haptics::Playback bootPlayback = haptics::Playback::NOT_STARTED;
    uint8_t motorFaultBits = 0;
    uint32_t hrAgeMs = UINT32_MAX, rmssdAgeMs = UINT32_MAX;
    uint32_t rmssdAccepted = 0, rmssdRejectJump = 0, rmssdRejectRange = 0;
    uint32_t hrRejected = 0, shapeRejected = 0, lastRmssdInput = 0, lastRmssdRejected = 0;
    char freshnessReason[24] = {};
    uint32_t p1Ms[3] = {}, p2Ms[3] = {}; // Session / Short / seven-session
    rmssd::Source p1Source = rmssd::Source::NONE, p2Source = rmssd::Source::NONE;
    uint32_t sessionValidMs = 0, sessionTargetMs = 0;
    unsigned sessionSamples = 0, sessionTargetSamples = 0;
    uint32_t shortValidMs = 0, shortBlockMs = 0, shortBreaks = 0;
    bool postExercise = false;
    uint32_t postExerciseRemainingMs = 0, rearmMs = 0, hapticElapsedMs = 0, hapticDurationMs = 0;
    // Own the labels so queued snapshots never point into mutable live state.
    char sessionState[48] = {}, storageState[16] = {};
    char shortState[16] = {}, shortGate[48] = {}, hapticState[32] = {}, bleState[16] = {};
};
const char *groupName(Group group);
// Complete newline-terminated line, shared by USB serial and BLE.
// False indicates an insufficient buffer; no partially formatted line is emitted.
bool formatLine(Group group, const Snapshot &snapshot, char *output, size_t capacity);
size_t notificationChunkSize(size_t remaining, uint16_t mtu);
}
