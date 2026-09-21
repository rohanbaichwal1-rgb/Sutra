#pragma once
#include <stdint.h>

namespace telemetry {
// POD copy owned by the sensor loop; BLE never reads live detector state.
struct Snapshot {
    float hr, ibi, rmssd, session, shortReference, personal;
    uint32_t uptimeSeconds, maxLoopGapUs;
    uint8_t finger, motion, converged, stable, fresh, savedSessions, episode;
};
bool begin();
void publish(const Snapshot &snapshot); // bounded, zero-wait queue overwrite
const char *state();
}
