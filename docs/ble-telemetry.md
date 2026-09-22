# BLE telemetry

The device advertises as **Sutra** after the required sensors initialize. Use a
generic BLE GATT explorer on Android, connect, open the service below, and read
or subscribe to the characteristics. Select UTF-8/text display rather than hex.
This is a custom service, not the standard Bluetooth Heart Rate Service, so an
app that only supports fitness heart-rate straps will not automatically decode it.

Service UUID: `7d2a0000-8f5a-4b10-9b6d-6f7574726100`.

Serial and BLE share the same nine labeled text groups, each on its own line.
Every line starts with device uptime (`[266s]`), and a blank line separates reports.
The group characteristics below support READ of the complete latest line.
Their UUIDs share the service suffix; the first eight digits identify the group:

| UUID prefix | Group | Contents |
|---|---|---|
| `7d2a0101` | Normal | Reference detector HR/IBI, finger, motion, IMU |
| `7d2a0102` | LMS | LMS HR/median IBI, RMSSD, convergence, stability, freshness |
| `7d2a0103` | PDR | Respiratory rate, quality, status, sources, resting baseline, deviation, support timer |
| `7d2a0104` | P1 | Session/short/seven-session timers, triggered flag, source |
| `7d2a0105` | P2 | Session/short/seven-session timers, triggered flag, source, respiratory rate match |
| `7d2a0106` | SessionBaseline | Frozen RMSSD, deviation, current session time/sample progress, storage status |
| `7d2a0107` | Short | Adapting RMSSD, deviation, qualifying time, block progress, breaks, collection gate |
| `7d2a0108` | PersonalBaseline | Seven-session RMSSD, saved session count, deviation |
| `7d2a0109` | System | BLE, maximum loop gap, post-exercise wait, active rearm timer, haptics |
| `7d2a01ff` | All groups | NOTIFY-only newline-delimited text stream containing all nine groups |

For live output, enable notifications on
`7d2a01ff-8f5a-4b10-9b6d-6f7574726100` and select UTF-8/text display.
The stream splits long lines across notifications to fit the negotiated ATT MTU
(default 20 payload bytes). A terminal that joins incoming text displays complete
lines; a GATT explorer that logs each packet separately may display fragments.
Requesting MTU 247 or 517, when supported by the app, reduces fragmentation.
The separate READ fields always contain complete lines; the app must support
long reads if a line exceeds its read payload. Reconnect and refresh the app's
cached services after uploading: these grouped UUIDs replace the old scalar/CSV fields.

Example (illustrative values):

```text
[266s] Normal | HR:77.7bpm | IBI:761ms | Finger:Y | Motion:LOW | IMU:OK
[266s] LMS | HR:78.0bpm | IBI:770ms | RMSSD:80.0ms | Converged:Y | RMSSDstable:Y | Fresh:Y
[266s] P1 | Session:0/30s | Short:0/30s | 7Session:-- | Triggered:N | Source:NONE
[266s] SessionBaseline | RMSSD:100.0ms | Dev:20.0% | Valid:100/100s | Samples:20/20 | State:FROZEN | Storage:SAVED
```

`--` means unavailable; booleans are Y/N. RMSSD deviation is positive for a drop;
PDR deviation is signed relative to the resting respiratory rate. `Normal` uses
the existing non-LMS reference beat detector; it is separate from the LMS values
used by the RMSSD logic. PDR remains context only and never gates P1/P2; see
[pdr-observer.md](pdr-observer.md).

Snapshots are produced about once per second, not as a lossless sample/beat log.
Bluetooth delivery is best effort; congested connections can lose notifications,
and slow connections may skip older reports. Group timestamps identify their
snapshot; separate characteristic reads are not an atomic read of the entire report.
This prototype permits unpaired reads/subscriptions and offers no write commands
or remote haptic control. Advertising resumes after disconnection.

## Timing

The Arduino sensor loop stays on core 1. It copies a fixed-size snapshot into a
one-element FreeRTOS queue without waiting; a new snapshot replaces an unsent
older one. A priority-1 task on core 0 initializes BLE, formats text and sends
notifications, paced at 10 ms per packet. BLE callbacks update connection/MTU
state and clear subscriptions on disconnect.
No Bluetooth API or connection wait runs inside beat detection.

This reduces direct blocking but does not guarantee unchanged HR/IBI. Radio
interrupts, shared memory, power noise and scheduling still need hardware checks.
The current detector timestamps processed samples with `millis()`, and the
MAX3010x library has a four-slot software ring. Delayed reads can bunch timestamps
or lose buffered samples. This change does not alter that existing acquisition
or detection algorithm. `LoopMax` makes scheduling gaps visible; it is a diagnostic,
not proof of beat accuracy or an overflow detector.

Compare serial HR, IBI, RMSSD, sample rate and LoopMax with BLE disabled,
advertising, connected/subscribed, and repeatedly reconnecting. Include simultaneous
haptic output in the hardware check. Add `-DSUTRA_BLE_ENABLED=0` to `build_flags`
for the disabled comparison. Changing this build flag requires uploading, which
clears saved sessions under this project's erase-on-upload setting.

## Validation

Build: `pio run -e seeed_xiao_esp32s3`.
Phone discovery/subscription/reconnection and timing under BLE load need the
actual device for validation.
