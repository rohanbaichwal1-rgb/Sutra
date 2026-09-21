# BLE telemetry

The device advertises as **Sutra** after the required sensors initialize. Use a
generic BLE GATT explorer on Android, connect, open the service below, and read
or subscribe to the characteristics. Select UTF-8/text display rather than hex.
This is a custom service, not the standard Bluetooth Heart Rate Service, so an
app that only supports fitness heart-rate straps will not automatically decode it.

Service UUID: `7d2a0000-8f5a-4b10-9b6d-6f7574726100`.

Each characteristic has READ and NOTIFY properties and a readable description.
The UUIDs share the service suffix; their first eight digits identify the value:

| UUID prefix | Text value | Units/meaning |
|---|---|---|
| `7d2a0001` | `77.7` | LMS HR, bpm |
| `7d2a0002` | `772.0` | Median recent LMS IBI, ms; not a stream of individual RR intervals |
| `7d2a0003` | `78.9` | Fresh RMSSD, ms |
| `7d2a0004` | `90.2` | Frozen current-session baseline, ms |
| `7d2a0005` | `85.1` | Adapting short reference, ms |
| `7d2a0006` | `88.4` | Seven-session personal baseline, ms |
| `7d2a0008` | `1,0,1,1,1,3,0` | Finger,motion,converged,stable,fresh,saved sessions,episode |
| `7d2a0009` | `120,9500` | Device uptime seconds, maximum loop-entry gap in microseconds during the report interval |

`--` means unavailable. Boolean fields are 0/1. Motion is 0 LOW, 1 MODERATE,
2 HIGH. Episode is 0 none, 1 P1, 2 P2. Values are snapshots about once per second,
not a lossless sample/beat log. Separate notifications are not an atomic group
on the phone. Notifications fit the default 20-byte payload without MTU negotiation.
This prototype permits unpaired reads/subscriptions and offers no write commands
or remote haptic control. Advertising resumes after disconnection.

## Timing

The Arduino sensor loop stays on core 1. It copies a fixed-size snapshot into a
one-element FreeRTOS queue without waiting; a new snapshot replaces an unsent
older one. A priority-1 task on core 0 initializes BLE, formats text and sends
notifications. BLE callbacks only change atomic flags.
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
