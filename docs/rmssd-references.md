# RMSSD references in the prototype

Current RMSSD uses the existing 60-second interval window and beat detector.
P1 has three independent comparison paths:

| Reference | Becomes available | Behaviour |
|---|---|---|
| Session baseline | 100 qualifying seconds in this startup | Median of 20 readings; frozen for this startup |
| Short reference | 60 qualifying seconds in the latest five minutes | Median of up to 60 recent readings; keeps adapting |
| Long-term personal baseline | Seven successfully saved startup sessions | Median of the latest seven frozen session medians; persisted in flash |

The session path works as soon as this startup's session baseline completes,
even before seven saved sessions. Before that, the short path can work alone.
An established seven-session baseline is restored and available at startup.
Sessions simulate days for prototype testing; no calendar dates are used.

```mermaid
flowchart TD
    IBI[Accepted individual IBI] --> Gate{300-2000 ms and<br/>within 20% of last accepted IBI?}
    Gate -- No --> Reject[Exclude from RMSSD/PDR]
    Gate -- Yes --> Window[60 s RMSSD window]
    Window --> RMSSD[sqrt(mean(successive IBI difference squared))]
    RMSSD --> Quality{Fresh signal, valid IMU,<br/>LOW motion, no post-exercise wait?}
    Quality -- No --> Pause[Break holds / cancel pending follow-up]
    Quality -- Yes --> Refs[Compare session, short, and personal references]
    Refs --> P1{Drop >=10% for 30 s?}
    P1 -- Yes --> P1Fire[Trigger P1: flutter and 5 s delay]
    P1 -- No --> Refs
    P1Fire --> Follow[Wait 5 s from P1 trigger]
    Follow --> Sweep[Stop flutter; start 10 s follow-up once]
    Follow -. signal/contact/activity/gap fault .-> Cancel[Cancel follow-up for this trigger]
    Sweep --> Rearm[60 s stable recovery below 10% drop]
    Cancel --> Rearm
```

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
collection requires neither LMS convergence nor a flat/stable RMSSD trend.
Fresh valid LMS-HR/RMSSD and successful IMU reads are still required. A positive
reference is needed for percentage comparisons.

The short reference adapts through pending triggers and episode recovery whenever
signal/motion quality gates pass. There is no preliminary 10% freeze or frozen short
snapshot. Active haptics, invalid/stale data, non-LOW motion and
post-exercise recovery pause collection. Readings keep expiring during pauses,
so `ShortValid` can decrease. Contact changes clear the short buffer but retain
episode latches, the completed session baseline and saved history.

P1 compares directly against the current live median. If adaptation reduces
deviation below a trigger threshold, that path's pending hold resets. The fixed
session baseline provides a separate comparison for sustained drops.

## Motion and recovery

HIGH motion starts/restarts a 60-second LOW-motion recovery requirement after
recovery is armed by the first qualifying resting block. Startup motion before
that block does not start post-exercise recovery. MODERATE never starts or resets
this wait; it pauses the countdown and preserves accrued LOW time. LOW resumes
it. Unobserved time does not count. Both MODERATE and HIGH interrupt collection
blocks and P1 continuous holds, and cancel a pending follow-up.

## P1 logic

Each available reference has its own continuous 30-second hold at a drop of
at least 10%. Equality qualifies: a 100 ms baseline requires RMSSD <=90 ms.
There is one trigger and one boot-lifetime trigger counter, not a second stress
threshold/timer. Both vibration stages belong to that same P1 trigger.

One completed path suffices; elapsed time cannot transfer between paths. Invalid
input, non-LOW motion or post-exercise recovery breaks holds. RMSSD stability is
required for fixed session collection and episode recovery, but not for short
adaptation or timing a drop. P1 fires once per episode.

P1 starts the existing five-second flutter immediately. A nonblocking five-second
delay starts at the trigger timestamp, not the first motor write. Once scheduled,
the follow-up does not depend on the RMSSD deviation, reference availability, or
baseline changes during that delay. Signal/contact/activity failures permanently
cancel follow-up for that trigger. Unobserved gaps over the existing 2.5-second
limit cancel too.
The motor-active collection gate alone does not cancel follow-up; actual invalid
sensor readings or disqualifying motion during vibration do.

At five seconds, hardware is confirmed stopped before the unchanged ten-second P1
follow-up sweep starts once. It is not retried on hardware failure.
Cancellation does not clear the episode latch. After follow-up starts, it runs
to completion unless a hardware fault stops it. PDR does not gate either stage.

Sources identify `SESSION`, `SHORT_TERM`, `LONG_TERM`, or explicit combinations
such as `SESSION+SHORT_TERM`. `LONG_TERM` means the seven-session personal value,
not the individual current-session baseline.

After an episode, rearm after 60 continuous seconds of stable, qualified recovery
with less than 10% drop against every available reference, at least one reference
available, and no active intervention. Exactly 10% does not qualify. The short
reference continues adapting during recovery when quality permits.

## Serial and BLE output

The LMS line now includes `Input` (freshness/quality reason), `HRage`, and
`RMSSDage` in milliseconds (`--` if no accepted beat exists in the current
history). `Input` distinguishes `NO_CONTACT`, `IMU_INVALID`, `HR_ACQUIRING`,
`HR_STALE_OR_INVALID`, `RMSSD_WARMUP`, `RMSSD_STALE`, and `OK`.
`Fresh` retains its original HR/RMSSD freshness meaning; IMU validity is also
required for baseline/trigger input, so `Input:IMU_INVALID` may coexist with
`Fresh:Y`. A warmup means insufficient valid RMSSD window data, including after
rhythm recovery; it does not necessarily mean the device just booted.

Diagnostic counts are cumulative since boot, retained across contact/rhythm
resets: `RRok` (accepted into RMSSD), `RRjumpReject` (>20% versus the last accepted
RMSSD interval after seeding), `RRrangeReject` (outside 300–2000 ms), `HRreject`
(intervals rejected by acquisition, rhythm, or refractory checks), and
`ShapeReject` (pulse regions failing width/amplitude checks). `LastRR` is the
last interval offered to RMSSD; `LastRejectRR` is the last rejected there, not
necessarily the latest candidate. Zero means none has been seen yet.
These diagnostics do not relax rejection/freshness rules or carry stale values
into P1 timers. New logs are needed to distinguish the remaining dropout causes.

P1 additionally shows both playback outcomes and next-action status; System retains
the startup motor-test result and driver fault bits. See
[haptic-protocols.md](haptic-protocols.md) for these command-status semantics.

Serial and BLE use separate lines for Normal, LMS, PDR, P1, SessionBaseline,
Short, PersonalBaseline and System. Each line includes device uptime.

- `SessionBaseline | RMSSD:<value>ms` with `State:FROZEN` after completion;
  `RMSSD:--` while collecting. `Valid:50/100s | Samples:10/20` shows this startup's
  progress. `State:PAUSED:<reason>` explains pauses; `Storage:SAVED` confirms saving.
- `PersonalBaseline` shows `SavedSessions:0/7` through `7/7` and the stored median
  once established.
- `Dev` on each reference's line compares current fresh RMSSD with that reference.
  Positive values mean a drop; unavailable/stale inputs display `--`.
- `Short` shows `State:ADAPTING`, `COLLECTING`, or `PAUSED`. `Gate` explains the
  blocker. `Valid` is qualifying time still in the rolling window; `Block` and
  `Breaks` show block progress and interruptions, including brief interruptions.
- `P1` shows independent `Session`, `Short`, and `7Session` hold timers, followed
  by `Triggered`, `Source`, playback/action, `FollowUp`, `FollowUpPlayback`, and
  `TriggerCount`. Unavailable reference paths show `--`.
- `System` includes `PostExercise` with remaining LOW time and `PAUSED` on MODERATE.
  `Rearm:0/60s` appears only after a trigger until recovery completes. Rearming
  needs 60 seconds of stable, qualified data below 10% drop against all available
  references, with no active haptics. This is separate from post-exercise recovery.

BLE group UUIDs and notification setup: [ble-telemetry.md](ble-telemetry.md).

## LMS convergence and RMSSD stability

HR freshness is measured from an accepted beat, not a rejected crest used to
reseed timing. After three seconds without accepted beats, LMS HR/IBI displays
`--`. A stale acquired rhythm can be replaced after four consecutive
morphology-qualified candidate intervals (300–1800 ms) whose largest interval
is no more than 15% above the smallest. Accepted beats clear that recovery
evidence, so isolated notches or missed beats do not replace a fresh rhythm.
Recovery seeds the seven-interval history with those four observed intervals
and waits for another accepted beat before publishing HR again. RMSSD and PDR
beat history restart to avoid mixing the old and new rhythm; pending collection
blocks/trigger holds are interrupted, but completed reference samples remain.
The existing HR smoothing factors and step limits are unchanged. This recovery
heuristic still requires validation on optical waveforms and simultaneous Polar
recordings; these summary logs cannot establish improved detector accuracy.

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
