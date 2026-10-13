# BLE telemetry

## Enable or disable BLE

Near the top of `src/main.cpp`, set:

```cpp
#define SUTRA_BLE_ENABLED 1 // 1 = ON, 0 = OFF
```

With `0`, main does not start the BLE task or publish BLE snapshots, so the
device does not advertise or accept BLE connections. Serial telemetry continues
and reports `BLE:OFF`; sensor processing and haptic settings are unchanged.
This main-file switch controls calls into the separately compiled BLE module;
it is not a guarantee that every Bluetooth library is excluded from compilation.
Only 0 and 1 are accepted. Rebuild and upload after changing the setting.
An explicit `-DSUTRA_BLE_ENABLED=0` or `=1` in PlatformIO `build_flags` overrides
the main-file default. Uploading erases saved sessions under the existing
erase-on-upload setting; an ordinary reset does not change this switch.

## Connection

The device advertises as **Sutra** after the required sensors initialize. Use a
generic BLE GATT explorer on Android, connect, open the service below, and read
or subscribe to the characteristics. Select UTF-8/text display rather than hex.
This is a custom service, not the standard Bluetooth Heart Rate Service, so an
app that only supports fitness heart-rate straps will not automatically decode it.

Service UUID: `7d2a0000-8f5a-4b10-9b6d-6f7574726100`.

Serial keeps its complete labeled groups. BLE publishes one compact selected-data
line approximately once per second; it does not mirror the Serial diagnostics.
The existing READ UUIDs remain stable. `7d2a0102` returns the latest compact line;
the other legacy group characteristics return `--` so they do not expose
additional live variables.

| UUID prefix | Group | Contents |
|---|---|---|
| `7d2a0101` | Legacy Normal | `--` |
| `7d2a0102` | Selected telemetry | Latest compact BLE line |
| `7d2a0103` | Legacy PDR | `--` |
| `7d2a0104` | Legacy P1 | `--` |
| `7d2a0106` | Legacy Session | `--` |
| `7d2a0107` | Legacy Short | `--` |
| `7d2a0108` | Legacy Personal | `--` |
| `7d2a0109` | Legacy System | `--` |
| `7d2a01ff` | Selected stream | Newline-delimited compact BLE line |

Retired UUID slot `7d2a0105` is unused; other group UUIDs have not moved.

For live output, enable notifications on
`7d2a01ff-8f5a-4b10-9b6d-6f7574726100` and select UTF-8/text display.
The stream can split long lines across notifications to fit the negotiated ATT MTU
(default 20 payload bytes). A terminal that joins incoming text displays complete
lines; a GATT explorer that logs each packet separately may display fragments.
Requesting MTU 247 or 517, when supported by the app, reduces fragmentation.
The selected READ field contains a complete line; the app must support long reads
if it exceeds its read payload. Reconnect and refresh the app's cached services
after uploading.

Example (illustrative values):

```text
Time:266s | LMS_HR:78.0bpm | LMS_IBI:770ms | PersonalBaseline:100.0ms | DEV:20.0% | ShortDEV:12.5% | Cal100:100/100s | Session30:0/30s | Short30:0/30s | P1:NO | PDR:14.2brpm
```

`--` means unavailable. RMSSD deviation is positive for a drop;
PDR BRPM is available only when RIFV passes its existing coverage, modulation, and
quality checks. PDR is display-only with no P1/follow-up connections; see
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
