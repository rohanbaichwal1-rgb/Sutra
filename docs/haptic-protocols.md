# Three-motor haptic actuation

The PoC uses the existing RMSSD P1/P2 events. It does not invent a State Alpha
classifier or confidence score. P1 runs for five seconds; P2 runs for six complete
ten-second breathing cycles (60 seconds). P2 replaces an active P1; a P1 request
cannot interrupt P2.

## Hardware mapping

The existing XIAO ESP32S3 I2C bus uses GPIO 5 for SDA and GPIO 6 for SCL at 400 kHz.
The MAX30101 and BMI270 remain on that root bus. The TCA9548A address is `0x70`.

| Logical motor | Position | Mux channel | DA7280 address |
|---|---|---|---|
| Motor 1 | Left | 0 | `0x4A` |
| Motor 2 | Center / P6 | 1 | `0x4A` |
| Motor 3 | Right | 2 | `0x4A` |

The driver selects exactly one channel for each addressed motor, checks I2C
results, and deselects all channels before returning to the sensor code. Each
DA7280 continues its amplitude independently after deselection, allowing both
outer motors to run during exhale. These are three separate driver modules;
the mux does not provide motor power or directly drive an LRA.

This setup targets the onboard LRAs of the supplied SmartElex modules. It retains
the existing/default DA7280 actuator voltage, current, impedance, and frequency
registers; the SmartElex manual does not publish a motor-specific replacement
profile. It does not substitute SparkFun's `defaultMotor()` values. This is not
a calibration of the attached motors. If the onboard motors have been replaced,
their electrical parameters must be configured from the replacement datasheet.

Amplitude is sent through DA7280 direct-register-override mode, with LRA BEMF
sensing, frequency tracking, acceleration, and rapid stopping enabled. The
0–100% envelope is mapped to the supported nonnegative 0–127 command range.
Percentages describe commanded amplitude relative to the configured driver
full scale, not measured mechanical strength. Driver supply-limit warnings are
reported because delivered strength can be lower than requested.

## P1: chaotic staccato flutter

- 90% amplitude on each motor's active taps.
- Each motor has its own staggered start and independent tap/gap timer.
- Nominal tap lengths vary from 60 to 250 ms; gaps vary from 50 to 450 ms.
- Taps may overlap across motors, without a common periodic beat.
- A fresh ESP32 random seed is used for each five-second intervention.
- All outputs stop at completion. A final tap is shortened only if it can still
  meet the 60 ms minimum; otherwise that tap is omitted.

## P2: resonant pacing sweep

| Time within each cycle | Motor 1 | Motor 2 | Motor 3 |
|---|---|---|---|
| 0–1.3 s | 10% → 50% | Off | Off |
| 1.3–2.6 s | Off | Off | 10% → 50% |
| 2.6–4.0 s | Off | 60% | Off |
| 4.0–6.0 s | Off | 60% → 30% | Off |
| 6.0–10.0 s | 30% → 0% | Off | 30% → 0% |

Ramps are linear in commanded amplitude. Cycle timing is derived from elapsed
time, so late loop calls do not accumulate breathing-cycle drift. The firmware
services envelopes about every 10 ms and also between serial report lines.
I2C transfers and loop scheduling introduce timing/quantization error; this is
not a hardware-timed waveform generator. The left/center/right mounting produces
the intended spatial cue; individual LRAs are not commanded to move physically
toward or away from P6.

## Runtime status and faults

The compact report shows `Haptic:OFF`, `P1_FLUTTER`, or `P2_SWEEP`, plus elapsed
and total intervention seconds. Protocol starts/completion and hardware errors
are separate event lines. Reference collection/recovery sees the entire
intervention as active, including gaps between taps.

Driver fault registers are checked before starting and every 100 ms while
playing. I2C failures and motor/temperature faults stop the sequence and attempt
zero-amplitude/inactive writes on all three channels. Faults latch until reboot.
Failed stop writes are retried every 250 ms. `FAULT_STOP_PENDING` is displayed
and reference adaptation stays blocked if a motor commanded into playback has
not acknowledged shutdown. Playback is tracked before sending the start command,
so losing its acknowledgement also requires a confirmed stop. A startup failure
before any playback command displays `FAULT` and does not count as an active
intervention: missing haptic hardware no longer prevents short-reference
collection. Haptics stay disabled until reboot. Software cannot guarantee
stopping an unreachable powered driver; the playback tracking is local to this
firmware run and is not a measurement of physical vibration. The I2C transfer
timeout is bounded at 10 ms.

## Files and validation

- `lib/HapticPatterns`: motor envelope timing, durations and gap constants.
- `lib/HapticArray`: mux mapping and DA7280 register transport.
- `src/HapticWireBus.h`: Arduino Wire adapter.
- `src/main.cpp`: RMSSD triggers, hardware setup, playback servicing, and status.
- `test/test_haptics`: phase boundaries, independent bursts, rollover, priority,
  simultaneous outer outputs, mux isolation, and fault/shutdown behavior.

Build: `pio run -e seeed_xiao_esp32s3`

Host tests: `pio test -c platformio-test.ini -e native`

Host tests use simulated time and I2C registers. Actual motor amplitude, mounting,
physical timing, and sensor quality during vibration require a hardware run.

## Hardware references

- [SmartElex module manual](https://robu.in/wp-content/uploads/2024/12/SmartElex-Haptic-Motor-Driver-%E2%80%93-DA7280-Manual.pdf)
- [Renesas DA7280 datasheet, sections 5.6 and 6.2](https://www.renesas.com/en/document/dst/da7280-datasheet)
- [TI TCA9548A datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf)
