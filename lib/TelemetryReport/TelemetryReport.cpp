#include "TelemetryReport.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

namespace telemetry {
namespace {
class Line {
    char *buffer_; size_t capacity_, used_ = 0; bool ok_ = true;
public:
    Line(char *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {
        if (capacity_) buffer_[0] = 0;
    }
    void append(const char *format, ...) {
        if (!ok_ || used_ >= capacity_) { ok_ = false; return; }
        va_list args; va_start(args,format);
        int n = vsnprintf(buffer_+used_,capacity_-used_,format,args);
        va_end(args);
        if (n < 0 || (size_t)n >= capacity_-used_) { ok_ = false; return; }
        used_ += n;
    }
    void value(float number, const char *unit = "", unsigned decimals = 1) {
        if (isfinite(number)) append("%.*f%s",(int)decimals,number,unit);
        else append("--");
    }
    bool finish() {
        append("\n");
        if (!ok_ && capacity_) buffer_[0] = 0;
        return ok_;
    }
};
float deviation(float value, float reference) {
    return isfinite(value) && isfinite(reference) && reference > 0 ?
        rmssd::Monitor::deviation(value,reference) : NAN;
}
const char *yn(bool value) { return value ? "Y" : "N"; }
void age(Line &line, uint32_t milliseconds) {
    if (milliseconds == UINT32_MAX) line.append("--");
    else line.append("%lums", static_cast<unsigned long>(milliseconds));
}
const char *protocolAction(const Snapshot &s, bool p1) {
    const auto playback = p1 ? s.p1Playback : s.p2Playback;
    if (s.hapticFault) return "MOTOR_FAULT";
    if (playback == haptics::Playback::STARTED) return "PLAYING";
    if (p1 ? s.p1Triggered : s.p2Triggered) return "WAIT_RECOVERY";
    if (p1 && s.p2Playback == haptics::Playback::STARTED) return "P2_PRIORITY";
    if (!isfinite(s.session) && !isfinite(s.shortReference) && !isfinite(s.personal)) return "NO_REFERENCE";
    if (!s.fresh || !s.imuOK) return "INVALID_INPUT";
    if (s.motion != 0) return "MOTION_PAUSED";
    if (s.postExercise) return "POST_EXERCISE";
    return "MONITORING";
}
void hold(Line &line, uint32_t elapsed, float reference, uint32_t required) {
    if (isfinite(reference)) line.append("%lu/%lus",(unsigned long)(elapsed/1000),(unsigned long)(required/1000));
    else line.append("--");
}
}
const char *groupName(Group group) {
    switch (group) {
    case Group::NORMAL: return "Normal";
    case Group::LMS: return "LMS";
    case Group::PDR: return "PDR";
    case Group::P1: return "P1";
    case Group::P2: return "P2";
    case Group::SESSION: return "SessionBaseline";
    case Group::SHORT: return "Short";
    case Group::PERSONAL: return "PersonalBaseline";
    default: return "System";
    }
}
bool formatLine(Group group, const Snapshot &s, char *output, size_t capacity) {
    Line line(output,capacity);
    line.append("[%lus] %s | ",(unsigned long)s.uptimeSeconds,groupName(group));
    switch (group) {
    case Group::NORMAL:
        line.append("HR:"); line.value(s.normalHr,"bpm");
        line.append(" | IBI:"); line.value(s.normalIbi,"ms",0);
        line.append(" | Finger:%s | Motion:%s | IMU:%s",yn(s.finger),
            s.motion == 0 ? "LOW" : s.motion == 1 ? "MODERATE" : "HIGH",s.imuOK ? "OK" : "ERROR");
        break;
    case Group::LMS:
        line.append("HR:"); line.value(s.hr,"bpm");
        line.append(" | IBI:"); line.value(s.ibi,"ms",0);
        line.append(" | RMSSD:"); line.value(s.rmssd,"ms");
        line.append(" | Converged:%s | RMSSDstable:%s | Fresh:%s",yn(s.converged),yn(s.stable),yn(s.fresh));
        line.append(" | Input:%s | HRage:",s.freshnessReason);
        age(line,s.hrAgeMs);
        line.append(" | RMSSDage:"); age(line,s.rmssdAgeMs);
        line.append(" | RRok:%lu | RRjumpReject:%lu | RRrangeReject:%lu | HRreject:%lu | ShapeReject:%lu | LastRR:%lums | LastRejectRR:%lums",
            (unsigned long)s.rmssdAccepted,(unsigned long)s.rmssdRejectJump,
            (unsigned long)s.rmssdRejectRange,(unsigned long)s.hrRejected,
            (unsigned long)s.shapeRejected,(unsigned long)s.lastRmssdInput,
            (unsigned long)s.lastRmssdRejected);
        break;
    case Group::PDR: {
        const auto &p = s.respiration;
        line.append("Rate:"); line.value(p.valid ? p.rate : NAN,"br/min");
        line.append(" | Q:%.0f | Status:%s | Src:%s | Rest:",p.quality,pdr::statusName(p.status),
            p.sources == 3 ? "RIAV+RIFV" : p.sources == 1 ? "RIAV" : p.sources == 2 ? "RIFV" : "--");
        line.value(p.baselineReady ? p.baseline : NAN,"br/min");
        line.append(" | RestValid:%lu/180s | Dev:",(unsigned long)(p.baselineValidMs/1000));
        line.value(p.valid && p.baselineReady ? p.deviation : NAN,"%");
        line.append(" | Support:%s | Hold:%lu/20s",yn(p.support),(unsigned long)(p.supportMs/1000));
        break;
    }
    case Group::P1:
    case Group::P2: {
        bool p1 = group == Group::P1;
        const uint32_t *timers = p1 ? s.p1Ms : s.p2Ms;
        const uint32_t required = p1 ? rmssd::P1_HOLD_MS : rmssd::P2_HOLD_MS;
        line.append("Session:"); hold(line,timers[0],s.session,required);
        line.append(" | Short:"); hold(line,timers[1],s.shortReference,required);
        line.append(" | 7Session:"); hold(line,timers[2],s.personal,required);
        line.append(" | Triggered:%s | Source:%s",yn(p1 ? s.p1Triggered : s.p2Triggered),
            rmssd::sourceName(p1 ? s.p1Source : s.p2Source));
        line.append(" | Playback:%s | Action:%s",
            haptics::playbackName(p1 ? s.p1Playback : s.p2Playback),protocolAction(s,p1));
        if (!p1) line.append(" | RateMatch:%s",pdr::syncName(s.respiration.sync));
        break;
    }
    case Group::SESSION:
        line.append("RMSSD:"); line.value(s.session,"ms");
        line.append(" | Dev:"); line.value(deviation(s.rmssd,s.session),"%");
        line.append(" | Valid:%lu/%lus | Samples:%u/%u | State:%s | Storage:%s",
            (unsigned long)(s.sessionValidMs/1000),(unsigned long)(s.sessionTargetMs/1000),
            s.sessionSamples,s.sessionTargetSamples,s.sessionState,s.storageState);
        break;
    case Group::SHORT:
        line.append("RMSSD:"); line.value(s.shortReference,"ms");
        line.append(" | Dev:"); line.value(deviation(s.rmssd,s.shortReference),"%");
        line.append(" | State:%s | Valid:%lus (min %lu, target %lu) | Block:%.1f/5s | Breaks:%lu | Gate:%s",
            s.shortState,(unsigned long)(s.shortValidMs/1000),
            (unsigned long)(rmssd::MIN_SAMPLES*rmssd::SAMPLE_MS/1000),
            (unsigned long)(rmssd::WINDOW_MS/1000),s.shortBlockMs/1000.0f,
            (unsigned long)s.shortBreaks,s.shortGate);
        break;
    case Group::PERSONAL:
        line.append("RMSSD:"); line.value(s.personal,"ms");
        line.append(" | SavedSessions:%u/7 | Dev:",s.savedSessions);
        line.value(deviation(s.rmssd,s.personal),"%");
        break;
    case Group::SYSTEM:
        line.append("BLE:%s | LoopMax:%.1fms | PostExercise:%s",s.bleState,s.maxLoopGapUs/1000.0f,yn(s.postExercise));
        if (s.postExercise) line.append("(%lus LOW remaining%s)",
            (unsigned long)((s.postExerciseRemainingMs+999)/1000),s.motion == 1 ? ", PAUSED" : "");
        if (s.episode) line.append(" | Rearm:%lu/60s",(unsigned long)(s.rearmMs/1000));
        line.append(" | Haptic:%s",s.hapticState);
        if (s.hapticDurationMs) line.append(" %lu/%lus",(unsigned long)(s.hapticElapsedMs/1000),(unsigned long)(s.hapticDurationMs/1000));
        line.append(" | BootTest:%s | FaultBits:0x%02X",haptics::playbackName(s.bootPlayback),(unsigned)s.motorFaultBits);
        break;
    default: break;
    }
    return line.finish();
}
size_t notificationChunkSize(size_t remaining, uint16_t mtu) {
    // ATT notification overhead is 3 bytes; MTU defaults to 23.
    const size_t capacity = (mtu >= 23 && mtu <= 517 ? mtu : 23)-3;
    return remaining < capacity ? remaining : capacity;
}
}
