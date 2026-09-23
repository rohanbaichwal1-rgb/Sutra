#pragma once
#include "HapticPatterns.h"

namespace haptics {
enum class Playback : uint8_t { NOT_STARTED, STARTED, COMPLETED, FAILED, PREEMPTED, SUPPRESSED };
inline const char *playbackName(Playback state) {
    switch (state) {
    case Playback::STARTED: return "STARTED";
    case Playback::COMPLETED: return "COMPLETED";
    case Playback::FAILED: return "FAILED";
    case Playback::PREEMPTED: return "PREEMPTED";
    case Playback::SUPPRESSED: return "SUPPRESSED";
    default: return "NOT_STARTED";
    }
}
// Command outcomes, not a measurement of physical vibration. Threshold latches
// remain owned by the RMSSD monitor and are never reset by a playback failure.
class PlaybackLog {
public:
    Playback state(Protocol p) const { return states_[static_cast<unsigned>(p)]; }
    void started(Protocol p) {
        if (active_ != Protocol::IDLE) states_[static_cast<unsigned>(active_)] = Playback::PREEMPTED;
        active_ = p;
        states_[static_cast<unsigned>(p)] = Playback::STARTED;
    }
    void finished(bool success) {
        if (active_ != Protocol::IDLE)
            states_[static_cast<unsigned>(active_)] = success ? Playback::COMPLETED : Playback::FAILED;
        active_ = Protocol::IDLE;
    }
    void failedRequest(Protocol p) {
        finished(false);
        states_[static_cast<unsigned>(p)] = Playback::FAILED;
    }
    void suppressed(Protocol p) { states_[static_cast<unsigned>(p)] = Playback::SUPPRESSED; }
private:
    Protocol active_ = Protocol::IDLE;
    Playback states_[4] = {};
};
} // namespace haptics
