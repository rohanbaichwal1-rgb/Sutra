# RMSSD references in the PoC

The existing 60-second RMSSD calculation, beat detector, and five-minute frozen
resting baseline are retained. The latter is now displayed as **Session
baseline**. It still averages one RMSSD reading every five seconds during five
qualifying minutes and pauses when its existing qualification gates fail.

The `LONG_TERM` trigger source currently means this session-baseline substitute.
Event logs also print `LongTermBasis:SESSION_BASELINE` to make that distinction
explicit. No daily measurements, calendar clock, persistent baseline, or
seven-day readiness state are implemented in this PoC.

## Short-term reference

- Median of up to 60 RMSSD readings from the preceding five wall-clock minutes.
- One reading is admitted after each complete five-second qualifying block.
  A block requires LOW motion, fresh valid LMS-HR and RMSSD, stable RMSSD,
  no intervention or active P1/P2 condition/episode, and no motion recovery.
- At least 36 such blocks (three valid minutes) are required. Collection
  continues toward 60 readings; invalid periods do not count as valid time.
- Old readings expire even during invalid input or a frozen episode. The
  frozen reference is a separate snapshot and does not expire mid-episode.
- A positive reference is required for percentage comparisons. A current
  RMSSD of zero remains a valid input when the upstream validity gate permits it.
- Input freshness requires an accepted RMSSD interval within three seconds,
  valid LMS-HR, and successful accelerometer/gyro reads. A report gap exceeding
  2.5 seconds breaks continuous timers; elapsed time without observations does
  not count toward sample collection or trigger holds.

`POST_EXERCISE_RECOVERY_MS` in `lib/RmssdReferences/RmssdReferences.h` defaults to
120 seconds of continuous LOW motion after any confirmed MODERATE/HIGH motion.
Renewed motion restarts that wait. This conservative PoC rule blocks reference
collection, trigger timers, and unfreezing during recovery. It does not attempt
to infer a physiological end to exercise from HR.

## Freeze, triggers, and recovery

A drop of at least 10% below the short reference starts a preliminary hold.
Collection pauses immediately, holding that reference constant during the
decision period. After ten continuous qualifying seconds, the reference is
marked `FROZEN`. This alone does not trigger a haptic intervention. Loss of the
preliminary condition before ten seconds cancels the hold.

Each reference has its own P1 timer (at least 20% for 30 continuous seconds) and
P2 timer (at least 30% for 120 continuous seconds). A short-path timer may start
during the preliminary hold against the same held value, but can trigger only
after freezing. Thus a sustained 20% drop can trigger P1 at 30 seconds, without
adding another ten seconds. RMSSD stability gates reference updates and recovery,
but is not required while timing a drop.

Either independently completed path triggers the corresponding level. Sources
are `LONG_TERM`, `SHORT_TERM`, or `BOTH`; `BOTH` means both paths completed that
level's duration at the firing time. P1 and P2 retain separate source records.
One path's partial hold cannot be transferred to the other. Each level fires
at most once per episode. P2 takes actuator priority if both levels fire together.

The reference stays frozen throughout the episode and intervention. A freeze
before P1 clears after 60 continuous seconds of stable, qualified recovery below
10% deviation from the short reference, with no pending P1/P2 condition. After
P1/P2, both available references must be below 10% for the same recovery period.
An active haptic pattern blocks recovery completion. At exactly 10%, recovery
does not qualify.

Invalid input or motion resets partial holds but preserves the frozen reference
and fired-level latches. Contact changes clear the rolling readings and partial
holds, while preserving any frozen episode. The retained session baseline follows
its existing reset behavior. When adaptation resumes, only unexpired rolling
readings may be reused; a long episode can require a fresh three-minute collection.

Serial output is grouped into four lines per report: timestamped readings,
quality/convergence, references/deviations, and timing/state. `DevPct` refers to
the session baseline; `ShortDevPct` refers to the short reference. `ShortValid`
describes the unexpired rolling readings, so it may decrease while a frozen
snapshot remains usable. P1/P2 timers show Long/Short elapsed seconds and their
required duration. Trigger sources are printed in the P1/P2 event messages.
Detailed reference-detector, filter, raw IMU, temperature, and startup-parameter
prints are commented out in `src/main.cpp` for optional troubleshooting.

## Future seven-day baseline

The session baseline can later be supplemented or replaced with the median of
the latest seven valid daily resting values, each based on at least five valid
minutes under consistent morning or sleep/rest conditions. That implementation
needs daily measurement selection, dates and persistence. Readiness should be
`UNAVAILABLE` below three valid days, `PROVISIONAL` at three to six, and
`ESTABLISHED` with seven valid days. It must remain separate from the rolling
short-term reference.

## Validation

Firmware: `pio run -e seeed_xiao_esp32s3`

Deterministic host tests through PlatformIO:
`pio test -c platformio-test.ini -e native`

The separate test configuration does not change the firmware's target environment.
Tests use simulated time and readings; they do not validate sensor accuracy or
physical haptic behavior.
