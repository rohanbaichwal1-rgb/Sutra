#pragma once
#include <TelemetryReport.h>

namespace telemetry {
bool begin();
void publish(const Snapshot &snapshot); // bounded, zero-wait queue overwrite
const char *state();
}
