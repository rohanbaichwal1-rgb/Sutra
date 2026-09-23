# PPG-derived respiration observer

PDR supplies context only. It never triggers, suppresses, delays, rearms or changes
thresholds for RMSSD P1/P2. Trigger messages annotate the latest `PDR_SUPPORT`.
RIIV and inhale/exhale phase synchronization are not implemented.

## Inputs and estimator

After LMS rhythm acquisition, each beat accepted by both the existing robust IBI
check and RMSSD artifact check sends its crest timestamp, cleaned crest amplitude
and unsmoothed individual IBI to PDR. Reconstructed acquisition intervals are not
sent. RIAV uses the existing cleaned crest amplitude, not peak-to-trough amplitude;
RIFV uses individual IBIs, not the displayed median IBI or smoothed HR. RIFV and
RMSSD share inputs and therefore are not independent evidence.

The sensor loop queues small records without waiting. A separate priority-1 task
on core 0 maintains a bounded beat ring and estimates approximately once a second.
Context changes are queued immediately, otherwise every 100 ms. Queue overflow
discards backlog/history and prevents a positive P2 summary based on lost events.
An unavailable worker or output older than 2.5 seconds reports `STALE`.

The estimator interpolates to 4 Hz, removes a linear trend, and searches sinusoids
from 4 to 30 breaths/minute in 0.5/min steps using Hann-weighted least squares.
The upper limit also stays below 45% of the mean beat frequency. Resampling does
not create information beyond the beat rate. The nominal window is 60 seconds;
the 0.5/min search spacing is not a claim of 0.5/min physical resolution.

At least 80% of the window must have usable interpolated samples. No extrapolation
is used. Beat gaps over 2.5 seconds clear history; segments over 1.8 times their
local IBI are excluded to avoid concealing missed beats. Contact/signal failure or
non-LOW motion clears the respiratory window and partial holds. Startup and these
interruptions require a fresh window, although completed baseline blocks remain.

Per-channel quality is explained weighted variance, on a 0–100 scale. Normalized
modulation must also have standard deviation at least 0.5% for RIAV or 0.3% for
RIFV. Channels need quality >=70. Two usable channels must agree within 2/min;
their rates are then weighted by quality. A single usable channel is allowed but
quality is capped at 75. Two disagreeing channels produce `DISAGREE`, not a guessed
average. These are prototype heuristics, not calibrated probabilities of correct
respiration or stress. Periodic motion/contact artifacts can still fool them.

## Resting baseline and support

PDR's baseline is separate from every RMSSD reference and is measured in breaths
per minute. It collects 36 complete five-second blocks (180 qualifying seconds),
takes their median, and freezes for the startup. It is RAM-only and starts fresh
after reset. No RMSSD saved-session/NVS format changes are made.

Collection requires valid PDR, LOW motion, fresh LMS/IMU inputs, converged LMS,
no post-exercise recovery, no active/latched P1/P2 episode and no haptic output or
uncertain shutdown. Ten consecutive rate estimates must span no more than the
larger of 1 breath/minute or 10% of their mean. RMSSD stability is not an additional
requirement. Invalid time discards a partial block but retains completed blocks.
The initial 60-second window and rate stabilization precede the 180-second count;
this is not 180 seconds from power-on.

Signed deviation is `100 * (rate - baseline) / baseline`. Absolute deviation >=20%
for 20 continuous seconds sets `PDR_SUPPORT`. Missing/invalid data, interruption,
post-exercise recovery, a smaller deviation or active haptics resets the hold.
The 20-second hold is on the estimated rate; windowing adds response latency.
Both increases and decreases count as respiratory change, not proof of stress.
Missing PDR is not negative evidence and never blocks P1/P2.

## P2 rate-match summary

Tracking starts only when the motor protocol successfully starts. The first 30
seconds are excluded from scoring. After that, the estimator uses only data from
the current intervention after that exclusion, never the preceding resting waveform.
At least 20 seconds of observed usable estimates are required. If at least 80%
of that qualifying time is within 6 +/- 1 breaths/minute, the completed protocol
reports `RATE_MATCH`; otherwise adequate data reports `NO_RATE_MATCH`.
Insufficient data, aborted/faulted playback, changed contact or queue loss reports
`UNKNOWN`. A report gap does not count as observed breathing. The summary remains
visible until the next P2 starts. Shorter than 30-second segments cannot be scored.
The current 10-second P2 test playback therefore reports `UNKNOWN`; the observer's
scoring requirements are unchanged.

This is an exploratory rate match, not verified breathing synchronization or
proof of the 4-second inhale/6-second exhale ratio. Motor vibration may create
periodic optical artifacts at the target rate. Validate against a respiratory
reference and motor-only controls before interpreting the summary as adherence.
Feature delay and polarity require calibration before adding phase alignment.

## Output and validation

Serial reports `PDR`, `Q`, `Status`, `Src`, `Rest`, `RestValid:0/180s`, signed `Dev`,
`Support`, and `Hold:0/20s` on its own PDR line. The P2 line includes `RateMatch`.
Missing values are `--` with a status explaining PDR availability.
The BLE fields are documented in [ble-telemetry.md](ble-telemetry.md).

`pio test -c platformio-test.ini -e native` includes synthetic modulation,
disagreement, flat inputs, baseline/support timing, interruptions, rollover and
P2 results. `pio run -e seeed_xiao_esp32s3` builds the device firmware. These checks
do not validate respiratory accuracy, physiological confidence or haptic artifacts.
The original processing-time beat timestamps remain a source of timing error;
compare `LoopMax` and physiological readings with and without BLE/PDR load.

Background: [respiration algorithm assessment](https://pmc.ncbi.nlm.nih.gov/articles/PMC5390977/),
[respiratory quality indices](https://pubmed.ncbi.nlm.nih.gov/28268418/), and
[slow-breathing feature delays](https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2019.01190/full).
