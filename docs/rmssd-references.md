# RMSSD references in the prototype

Current RMSSD uses the existing 60-second interval window and beat detector.
P1/P2 has three independent comparison paths:

| Reference | Becomes available | Behaviour |
|---|---|---|
| Session baseline | 100 qualifying seconds in this startup | Median of 20 readings; frozen for this startup |
| Short reference | 60 qualifying seconds in the latest five minutes | Median of up to 60 recent readings; keeps adapting |
| Long-term personal baseline | Seven successfully saved startup sessions | Median of the latest seven frozen session medians; persisted in flash |

The session path works as soon as this startup's session baseline completes,
even before seven saved sessions. Before that, the short path can work alone.
An established seven-session baseline is restored and available at startup.
Sessions simulate days for prototype testing; no calendar dates are used.

## Current session and saved sessions

Each startup collects one reading after each complete five-second block of LOW
motion, fresh valid LMS-HR/RMSSD, successful IMU reads, stable RMSSD and converged
LMS. Post-exercise recovery and active haptic output block collection. Pending
triggers and episode latches do not block otherwise qualifying resting data.

Completed session blocks accumulate without expiring. Signal/contact/motion
interruptions discard an incomplete block; report gaps over 2.5 seconds cannot
count as observed data. At 20 readings (100 qualifying seconds), the session
median freezes. Later readings and contact changes cannot change it. A new
startup begins a new session measurement.

`SavedSessions` receives exactly that frozen session median once per startup,
never the changing short reference. Staying powered on cannot contribute another
value. Resets and firmware-upload restarts are new startup opportunities, but
only completed measurements count. After seven saves, each completed startup
replaces the oldest saved value. Changing the long-term median resets its pending
trigger holds so elapsed time cannot carry across a reference change.

## Persistence

`src/NvsBaselineStore.h` uses ESP32 Preferences/NVS in namespace `sutra-baseline`.
Alternating `history0` and `history1` records contain seven medians, ring position,
count, version, generation and CRC32. Writes are committed and verified by reading
back before in-memory history advances. The previous slot remains a fallback if
the newest record is corrupt or incomplete. Writes occur once per completed
startup measurement, not once per sensor sample.

Saved sessions survive resets, power-off and battery discharge. An unfinished
session measurement and the short buffer live in RAM and restart after power
loss. This project's USB/esptool uploads now use `--erase-all`: each upload clears
all flash, including saved sessions, before programming. This also applies when
re-uploading an unchanged binary. Ordinary resets and power cycles retain history.
The 100-second session duration is a shortened prototype testing setting.

If neither existing record is valid or storage cannot be read, `Storage:ERROR`
leaves long-term detection unavailable without silently overwriting history.
Session and short detection can still operate. A failed save reports `SAVE_RETRY`
and retries no more than once per ten seconds when resting quality permits,
using the same frozen session median and preserving previous saved history.

## Adapting short reference

One reading is stored per complete five-second qualifying block and expires at
age five minutes. At least 12 retained readings (60 qualifying seconds) are
needed. Collection continues toward 60 readings. Unlike session collection, short
collection does not require LMS convergence separately from the fresh-signal and
RMSSD-stability checks. A positive reference is needed for percentage comparisons.

The short reference adapts through pending triggers and episode recovery whenever
resting quality gates pass. There is no preliminary 10% freeze or frozen short
snapshot. Active haptics, invalid/stale data, unstable RMSSD, non-LOW motion and
post-exercise recovery pause collection. Readings keep expiring during pauses,
so `ShortValid` can decrease. Contact changes clear the short buffer but retain
episode latches, the completed session baseline and saved history.

P1/P2 compares directly against the current live median. If adaptation reduces
deviation below a trigger threshold, that path's pending hold resets. The fixed
session baseline provides a separate comparison for sustained drops.

## Motion and recovery

HIGH motion starts/restarts a 60-second LOW-motion recovery requirement after
recovery is armed by the first qualifying resting block. Startup motion before
that block does not start post-exercise recovery. MODERATE never starts or resets
this wait; it pauses the countdown and preserves accrued LOW time. LOW resumes
it. Unobserved time does not count. Both MODERATE and HIGH interrupt collection
blocks and P1/P2 continuous holds.

## P1/P2 logic

Each available reference has its own continuous timers:

- P1: current RMSSD at least 20% below that reference for 30 seconds.
- P2: current RMSSD at least 30% below that reference for 120 seconds.

One completed path suffices; elapsed time cannot transfer between paths. Invalid
input, non-LOW motion or post-exercise recovery breaks holds. RMSSD stability is
required for adaptation and episode recovery, but not for timing a drop. Each
level fires once per episode; P2 takes haptic priority.

Sources identify `SESSION`, `SHORT_TERM`, `LONG_TERM`, or explicit combinations
such as `SESSION+SHORT_TERM`. `LONG_TERM` means the seven-session personal value,
not the individual current-session baseline.

After an episode, rearm after 60 continuous seconds of stable, qualified recovery
with less than 10% drop against every available reference, at least one reference
available, and no active intervention. Exactly 10% does not qualify. The short
reference continues adapting during recovery when quality permits.

## Serial output

- `SessionBaseline:<value>ms (FROZEN)` after completion; `--` while collecting.
- `SessionValid:50/100s | Samples:10/20` shows this startup's progress.
- `State:COLLECTING`, `PAUSED:<reason>` or `FROZEN`; `Storage:SAVED` confirms saving.
- `SavedSessions:0/7` through `7/7` and `Long-term:<value>ms` once established.
- `SessionDevPct`, `ShortDevPct`, and `DevPct` compare against session, live short,
  and seven-session personal values, respectively.
- Short status is `ADAPTING`, `COLLECTING`, or `PAUSED`; `ShortGate` shows the
  blocker. `ShortBlock` and `ShortBreaks` show block progress and interruptions,
  including interruptions that clear between reports.
- `P1[Session/Short/7Session]` and `P2[Session/Short/7Session]` show independent timers.
  `PostExercise` shows remaining LOW time, with `PAUSED` on MODERATE.
- The idle `State:MONITORING` field is omitted. After a trigger, `Episode:P1/P2`
  and `Rearm:0/60s` appear until recovery completes. Rearming needs 60 seconds of
  stable, qualified data below 10% drop against all available references, with
  no active haptics. This is separate from post-exercise recovery.

## LMS convergence and RMSSD stability

LMS convergence checks 20 once-per-second filter-weight observations, smoothed
with an EMA factor of 0.2. The maximum minus minimum must be at most 20 weight
units for ten seconds. Earliest convergence is roughly 30 seconds after reset
with settled data; drifting weights take longer. Five seconds outside the range
loses convergence. Finger-contact changes reset the convergence history.

RMSSD stability uses a three-reading median pre-filter only for this check, then
20 valid readings sampled at most once per second. Their range must be no more
than the larger of 8 ms or 12% of their mean for ten seconds. At a mean of 100 ms,
the allowed range is 12 ms. Roughly 30 seconds of steady valid RMSSD can establish
stability from empty history; five seconds outside the range loses it. Invalid
RMSSD pauses this check without resetting its history/hold timestamps, so these
are not strictly continuous valid-data windows across gaps. Independent fresh
signal gates still prevent invalid/stale data from entering baseline collection.

Beat-timing changes, motion/contact noise, missed or extra detections, artifact
rejections, and readings entering/leaving the 60-second RMSSD window affect its
range. The stability flag itself tests RMSSD range, not motion or convergence;
baseline collection checks those conditions separately. Finger-contact changes
reset stability history. A stable number does not establish measurement accuracy.

`Haptic:FAULT_STOP_PENDING` blocks collection if shutdown of commanded output is
uncertain. Startup-unavailable haptics show `FAULT` and allow collection. See
[haptic-protocols.md](haptic-protocols.md) for actuation details.

## Validation and future daily baseline

Firmware: `pio run -e seeed_xiao_esp32s3`

Host tests: `pio test -c platformio-test.ini -e native`

Tests simulate readings, timing, storage and I2C. Sensor accuracy, physical
actuation and actual battery-loss recovery require hardware testing. A future
calendar-day implementation needs dates and consistent daily measurement
selection; seven startup sessions do not establish seven days of data.
