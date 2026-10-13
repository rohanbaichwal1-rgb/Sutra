# HIGH-motion HR experiment

The independent `lib/HighMotionProcessor` module is enabled for walking tests.
It never supplies intervals to RMSSD, PDR, references, P1, or haptics. Existing
LOW/MODERATE/HIGH classification and all original signal processing continue.

## Switches

Defaults in `HighMotionProcessor.h` (can be overridden with build definitions):

| Definition | Default | Effect |
| --- | --- | --- |
| `HIGH_MOTION_PROCESSOR_ENABLED` | 1 | Copy existing sensor readings and run shadow processing |
| `HIGH_MOTION_OUTPUT_OVERRIDE` | 1 | Presentation-only replacement of `snapshot.hr` during HIGH motion (user's test setting) |
| `HIGH_MOTION_DEBUG` | 1 | Separate, best-effort Serial diagnostic lines (user's test setting) |

Override does not replace displayed IBI, physiological inputs, validity gates,
or convergence gates. During HIGH motion it publishes unavailable HR if the
new processor is invalid; it does not fall back to an uncertain shadow value.
Set override to zero for shadow-only comparisons. Disabled
processing removes integration calls at compile time.

Debug rotates four short lines, one per second. It skips a line if the Serial
transmit buffer lacks room; normal telemetry messages and fields are untouched.
Reason and lastRejection are separate so accepted beats do not erase rejection
diagnostics. Counts are cumulative per boot. `maxUs` is the observed maximum
PPG-enqueue or poll call duration, not the complete firmware loop duration.
The fourth line reports the last candidate's amplitude, amplitude floor,
rise duration, residual correlation, and cancellation ratio.

## Processing

Three accelerometer axes each have four independent NLMS taps. Each axis has
its own DC reference. Coefficient bounds, bounded error and updates, leakage,
and finite-value checks protect the filter. Gyroscope references are excluded
until recordings establish their value. There is no dynamic allocation or new
task. Fixed buffers hold 32 IMU samples, 16 pending PPG samples, and 7 intervals.

The independent processor runs whenever contact has settled, including outside
HIGH motion, to keep its own filter and beat history warm. Only HIGH motion
can select its output. Classification transitions do not reset its state;
contact loss clears it immediately. This adds background CPU work without
changing the existing LOW/MODERATE calculations. A DC
remover and low-pass stage precede a pulse-onset/local-crest detector. Rise
confirmation uses a meaningful fall from the tracked crest instead of the
first downward sample. An unfinished rise expires at `maxRiseUs`; tiny ripples
cannot keep the detector stuck indefinitely. Onset and amplitude thresholds
use power after low-pass filtering, while motion-coupling confidence retains
the independent residual/reference statistics. Rise
duration, amplitude consistency, residual accelerometer correlation, and
cancellation energy gate candidates. Four consistent measured intervals are
required for readiness. Implausible intervals restart acquisition; no intervals
are split, filled, reconstructed, or passed into existing physiological logic.
Median intervals drive a time-dependent HR EMA; displayed shadow IBI remains
the latest measured accepted interval. Missing references, sample discontinuity,
overflow, numerical errors, and stale beats invalidate output. A stale rhythm
clears beat acquisition but preserves the healthy filter. Minor morphology,
amplitude, and refractory rejections do not erase a fresh accepted HR and never
refresh its expiry. Motion-coupling failures still invalidate immediately.

These checks reduce common artifact locks; they cannot prove a pulse is cardiac
or guarantee prevention of half/double-rate locking without real recordings.
Confidence is a heuristic, not a calibrated probability or clinical metric.

## Timing limitations

The existing driver returns FIFO samples without hardware timestamps. The
initial sample is anchored to host receipt, with older samples backdated using
an independently measured receive rate over two-second windows. Subsequent
timestamps advance on that estimated clock, with host phase corrections
bounded to 5% of a sample period. Receipt phase errors over 20 ms invalidate
the sample instead of concealing a possible dropout. The initial
period is the existing hardware's observed 10 ms stream. No existing sample
rate, acquisition sequence, sensor configuration, or task is changed.

An accelerometer timestamp is the midpoint of its existing read transaction.
PPG processing waits for bracketing valid IMU measurements and linearly
interpolates only across gaps of at most 30 ms. Invalid reads interrupt that
bracket; there is no extrapolation. Waiting longer than 40 ms or FIFO batches
larger than four samples invalidates processing. All timestamps are flagged
estimated, even after receive-rate measurement. Host timestamps cannot detect
every internal sensor/FIFO overrun, sensor delay, or driver-buffer loss.
Constant latency offsets are configurable but zero until measured. The digital
filter's pulse delay is not subtracted from intervals: differences between
actual filtered-crest timestamps are retained. Changing pulse shape can still
bias those intervals. This path is for HR testing, not HRV.

## Experimental parameters

All defaults below belong only to `highmotion::Config`. Original parameters
are untouched. Fixed capacities and tap count are in `Processor`.

| Parameter | Default | Purpose |
| --- | --- | --- |
| learningRate | 0.015 | Conservative NLMS adaptation gain |
| coefficientBound | 30 | Absolute bound per tap |
| leakagePerSecond | 0.02 | Time-scaled coefficient decay |
| normalization | 25 | Denominator regularization |
| referenceScale | 1000 | Convert g residuals to milli-g |
| referenceDcSeconds | 0.8 | Axis gravity/offset removal |
| ppgDcSeconds | 0.8 | Independent PPG baseline removal |
| lowPassSeconds | 0.025 | High-frequency noise attenuation |
| maxAdaptError | 150 | Gradient error bound in PPG counts |
| maxCoefficientStep | 0.1 | Per-sample per-tap update bound |
| motionEnergyFloor | 4 | Minimum squared multi-tap reference energy |
| minPulseAmplitude | 3 | Minimum PPG pulse height |
| noiseMultiplier | 0.65 | RMS-relative amplitude floor |
| minRiseSlope | 20 | Minimum pulse onset slope, counts/s |
| noiseSlopeMultiplier | 2 | RMS-relative onset slope floor, per second |
| peakDropAmplitudeFraction | 0.15 | Crest confirmation fall relative to pulse excursion |
| peakDropRmsFraction | 0.10 | Crest confirmation fall relative to filtered RMS |
| minAmplitudeRatio / maxAmplitudeRatio | 0.35 / 2.8 | Allowed height relative to accepted pulse EMA |
| maxResidualCorrelation | 0.85 | Reject motion-correlated residuals |
| maxCancellationRatio | 0.95 | Reject near-total cancellation |
| intervalTolerance | 0.25 | Maximum deviation from interval median |
| hrSmoothingSeconds | 2 | HR EMA response time |
| minConfidence | 0.45 | Publication confidence floor |
| minIbiUs / maxIbiUs | 300000 / 1500000 | Measured interval plausibility, 40-200 bpm |
| minRiseUs / maxRiseUs | 30000 / 450000 | Pulse onset-to-crest duration |
| staleUs | 2500000 | Maximum beat age |
| warmupUs | 2000000 | Settle filters before acquisition |
| maxImuGapUs | 30000 | Largest interpolation bracket |
| maxPpgGapUs | 40000 | Largest continuous PPG interval |
| alignmentWaitUs | 40000 | Maximum bracketing wait |
| imuLatencyUs / ppgLatencyUs | 0 / 0 | Unvalidated constant acquisition delay corrections |
| initialSamplePeriodUs | 10000 | Initial estimated FIFO sample spacing |
| maxEstimatedBatch | 4 | Largest accepted estimated FIFO batch |
| acquisitionIntervals | 4 | Consistent measured intervals before readiness |
| timestampJitterUs | 20000 | Maximum host receipt phase error tolerated by estimated clock |
| timestampCorrectionFraction | 0.05 | Maximum clock phase correction per sample, as fraction of period |

Receive-rate measurement accepts periods from 2500 to 20000 us. Signal-power
and correlation statistics use a one-second EMA. Accepted pulse amplitude
uses an EMA gain of 0.15. These are independent HIGH-motion constants.

## Validation boundary

Native tests check synthetic pulse acquisition, stale invalidation, transitions,
missing references, FIFO uncertainty, motion-correlated artifacts, overflow,
nonfinite samples, and microsecond timer wraparound. These tests establish
mechanics, not walking accuracy. Compare synchronized firmware and Polar H10
recordings offline before tuning or enabling override. Polar is never a runtime
input. Measure loop timing and FIFO loss on the ESP32-S3: additional processing
has a bounded operation count but still consumes CPU time.

## Initial verification results (2026-10-13, shadow implementation)

- Default shadow firmware build: passed; RAM 59740 bytes (18.2%), flash
  994333 bytes (29.7%). Final artifacts use default switches.
- Processing-disabled firmware build: passed; RAM 58452 bytes.
- Debug/output-override compilation: passed; never uploaded or activated.
- New HIGH-motion native tests: 8/8 passed.
- Existing beat recovery: 6/6; references: 39/39; haptics: 24/24;
  personal baseline: 11/11; telemetry: 11/11 passed.
- PDR: 27 assertions passed, four failed (initial-window signal pause,
  competing-rate leakage, long interpolation hole, and initial warmup).
  Runner also reported SIGILL. The four assertions were already failing
  during earlier PDR work; neither the PDR implementation nor its tests
  were edited for this task.
- The all-suite PlatformIO invocation encountered `UndefinedEnvPlatformError`
  after beat recovery; each remaining suite was run individually.
- Existing-file edits for this task: additive sensor-copy, shadow poll,
  optional debug and presentation hooks in `src/main.cpp`; HIGH-motion
  include/source entries in `platformio-test.ini`. `platformio.ini` and
  `RmssdReferences.cpp/.h` were not edited. Prior worktree edits are retained.
- Sensor configuration and task scheduling code are untouched. Device loop
  overhead, sensor latency, and walking accuracy against Polar H10 remain
  unmeasured. The build retains the existing third-party I2C buffer warning.

## Walking-run follow-up (2026-10-13)

`BLE_HR_20261013_165719.txt` contains 480 reports from 16:57:22 to 17:05:24 IST.
All 225 HIGH-motion reports have unavailable HR; 136/145 MODERATE and 104/110
LOW reports have HR. There are 81 motion-state transitions. No Serial
diagnostics or raw sensor data were supplied, so the dominant runtime rejection
reason and HR accuracy of the changed code cannot be established from this run.

The follow-up changes keep the independent processor warm across classification
transitions, tolerate bounded host timestamp jitter, retain fresh HR after minor
candidate rejection, and avoid filter restarts solely because beat history went
stale. They do not relax motion-coupling gates or fill missing intervals. The
user's override/debug settings remain one. Tests cover these scenarios and
selection with override enabled. Another device run with Serial capture is
required to validate walking accuracy and CPU overhead.

Follow-up verification: HIGH-motion 11/11 and telemetry 11/11 tests passed.
References had 38 passing tests and one failing assertion in
`test_moderate_motion_continues_running_p1_hold_but_high_motion_resets_it`
(`Expected 14000 Was 0`); its implementation and tests were untouched here.
Firmware build passed with override/debug one: RAM 59764 bytes (18.2%),
flash 995277 bytes (29.8%). No firmware was uploaded. Only the HIGH-motion
module, its tests, and this document were edited in this follow-up.

## Serial-log detector follow-up

The 17:23:47 Serial recording has 102 HIGH-motion diagnostic summaries: two
valid and 100 invalid. Invalid status snapshots include 46 AMPLITUDE, 32
MORPHOLOGY, ten MOTION_COUPLING, six INTERVAL, five ACQUIRING, and one STALE.
These are sampled status counts, not per-beat rejection totals. Alignment
diagnostics report zero drops and a roughly 10 ms sample period. This identifies
pulse detection/qualification as the observed obstacle rather than alignment.

The detector now confirms a tracked crest with amplitude/RMS hysteresis,
expires incomplete rises, and evaluates height/onset relative to filtered
signal power. The new ripple fixture and previous tests pass (12/12).
Motion-coupling gates, interval bounds, readiness and stale expiry remain in
place. No original LMS or physiological calculation was edited. Main's only
new change in this follow-up is the fourth separate Serial diagnostic page.
Synthetic success does not establish real walking accuracy or coverage;
the recording contains no raw waveform for replay of these detector changes.

Final follow-up build passed with override/debug one: RAM 59804 bytes (18.3%),
flash 995697 bytes (29.8%). HIGH-motion tests passed 12/12 and diff whitespace
checks passed. Firmware was not uploaded. The existing third-party I2C buffer
warning remains.
