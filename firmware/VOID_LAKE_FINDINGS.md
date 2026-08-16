# Void Lake Test Flight — Analysis Findings & Fixes

Analysis of `Void_Lake_flight_log (copy).csv` (57 716 rows, 446 s, dumped by
rocket-cli), performed 2026-07/2026-08. All state-estimator findings were
produced by replaying the logged barometer data through the actual flight code
(`air-brakes-controller-core::baro_state_estimator`); diagnostics figure in
[`kf_diagnostics.png`](kf_diagnostics.png).

**Flight timeline (from the log, t = seconds since log start):**
liftoff ~11.7 · stage detect 12.19 · burnout ~14.7 · redundant-FC ejection
charge 28.47 · apogee ~28.4–29.2 (plateau), ~1 514 m AGL · touchdown ~217 ·
logging stops ~257 (Landed mode).

---

## 1. GPS altitude frozen during ascent — FIXED

**Symptom:** the `altitude` column is unusable in flight: it wiggles around pad
height (271.7–274.0 m) for the entire ascent while lat/lon, DOPs, and sat
counts keep updating, drops out during boost, and only reads correctly again
near apogee (1 707.6 m).

**Root cause:** the CAM-M8Q boots into its default "portable" dynamic platform
model, whose navigation filter rejects launch-grade vertical dynamics. The
firmware never configured the receiver — `gps_task` only listened.

**Fix** (`src/tasks/gps_task.rs`): on every boot, send UBX-CFG-NAV5 with
`dynModel = 8` ("airborne <4g") three times at 1 s intervals. The setting does
not survive a power cycle. Not verifiable from ground data — check altitude
tracks on the next sky-view test.

## 2. `Coasting` flight stage never produced — FIXED

**Symptom:** telemetry, CAN `VLStatus`, and the flight log jump straight from
`PoweredAscent` to `MainDeployed`; `FlightStage::Coasting` existed in the enum
but no code path ever produced it.

**Root cause:** the baro-only estimator's single `Ascent` state covers burn and
coast; both stage-mapping sites in `armed_mode.rs` translated it
unconditionally to `PoweredAscent`.

**Fix:** burnout detection inside the estimator (low-passed d(KF velocity)/dt,
latched; see finding 6 for the hardening), exposed as
`RocketStateEstimator::is_coasting()`, mapped to `FlightStage::Coasting` in
both armed_mode sites. (Superseded 2026-08-15: `FlightStage` is now an honest
`RocketState` mirror; coasting is no longer on the wire.)
Additionally the airbrakes start signal and the CAN
`RocketStateMessage.is_coasting` field (previously filled with an
ascending-and-subsonic proxy that was true mid-burn) now use the real flag, so
airbrakes can no longer be commanded under thrust. (Superseded 2026-08-16:
`RocketStateMessage` is removed entirely — nothing on the bus consumed it.)

## 3. KF ran at half its design rate — velocity 2× too high — FIXED

**Symptom:** flight KF velocity was ~1.95–2.0× true everywhere (peak 684 m/s on
a ~250 m/s flight; −14 m/s indicated under main vs −7 m/s true). Verified by
replay: under-main ratio 2.04 as flown, 1.06 with corrected DT.

**Root cause:** `read_imu_baro_loop` joins each IMU read with a *full* MS5607
read (2 280 µs pressure conversion at OSR=1024 + 600 µs temperature), which
overruns the ~2.34 ms IMU data-ready period, so every other IMU edge was
skipped: the IMU+baro stream ran at 213.5 Hz while the KF's hardcoded
`DT = 1/416 s` assumed 416 Hz. The velocity state settles at
(actual dt / assumed dt) ≈ 1.95× true to keep tracking altitude. Every velocity
threshold (ignition >10, descent <−2, 0.85 Mach airbrakes gate) was effectively
halved in real terms, and the airbrakes MPC received 2× velocity — which
explains the full-extension command through the whole coast on a flight that
undershot its target apogee. Altitude tracking was unaffected (velocity absorbs
the error), which is why it went unnoticed. HIL builds bypass the real baro and
were already running at full rate — flight and HIL disagreed until this fix.

**Fix** (`src/drivers/ms5607.rs`): pressure OSR 1024 → 512 (1.17 ms conversion)
and temperature converted only every 32nd read (~75 ms; raw D2 cached between
refreshes). Worst-case read is now ~1.8 ms, so the loop services every IMU edge
(~427 Hz true ODR). KF measurement variance updated 0.244 → 0.45 m²
(datasheet noise ratio OSR512/OSR1024 applied to the 2026-06-11 bench value).

**Residual / pre-flight actions:**
- Verify cadence on hardware: logged `timestamp_us` deltas should be ~2.34 ms
  with no every-other-edge skipping (HIL builds cannot verify this).
- Re-bench R at OSR=512 with `baro_bench` and replace the scaled estimate.
- ~2.6 % velocity bias remains (true ODR ~427 Hz vs assumed 416); a
  timestamp-aware `predict` would remove it if ever needed.
- SD logging throughput doubles (~28 KB/s); watch for `SD write queue depth`
  warnings on the bench.

## 4. Pyro ejection transient corrupted the KF near apogee — FIXED

**Symptom:** 12 baro samples at t=28.469–28.521 read up to 1 465 m low — the
redundant flight computer's ejection-charge overpressure hitting the static
port. Ungated, they dragged KF altitude down ~170 m and velocity to −995 m/s
with a 1–2 s recovery, right at apogee.

**Why it matters even though the backup fired at apogee:** (a) the same
transient occurs from VLF5's own charges every flight, inside the
`DrogueDeployed → MainDelay` altitude-comparison window; (b) a backup that
*misfires early* (the scenario redundancy exists for) would scramble the
healthy computer's estimator mid-coast; (c) the airbrakes control loop exits
permanently on a single sampled `vertical_velocity <= 0`, so one transient
during coast would end apogee control.

**Fix** (`BaroAltitudeKF::update`): innovation gate — reject measurements with
|innovation| > 75 m and coast on the prediction. The threshold sits in the
order-of-magnitude gap between worst genuine dynamics mismatch (~45 m,
measured at the old half rate; smaller at full rate) and the transient
(200–1 465 m). Anti-lockout: after 1 s of consecutive rejections, force-accept
with inflated altitude covariance so a genuinely diverged filter snaps to the
measurement (velocity state preserved) instead of flying blind.

**Validation on flight data:** as-logged replay — 11 rejections, all inside
the transient, zero false rejections in ~50 k samples; apogee velocity
excursion −995..+300 → −124..+63 m/s; stage timing unchanged. At the fixed
427 Hz rate (finding 7) — 22 rejections, all in-window, zero false.

> **2026-08-13 (estimator rework, phase A):** gate widened 75 → 500 m for the
> slow (~1 s bandwidth) deployment filter, whose genuine boost tracking lag
> reaches ~330 m for a 16 g motor with no Mach lockout. The gate is now pure
> bus input validation; sub-gate transients barely move the slow filter
> (a 25-sample 500 m offset shifts altitude ~30 m / velocity ~15 m/s) and no
> deployment decision reads short-term velocity anymore. Re-validated by a
> full flight replay (`void_lake_flight_replay` test): apogee detected at
> 1500.2 m AGL through the real blast transient.

## 5. Liftoff pressure transient false-latched Coasting at the fixed rate — FIXED

**Symptom (found only in the 427 Hz interpolated replay):** a genuine baro
disturbance ~0.4 s after liftoff detect (plume/rail aerodynamics; visible in
the raw measurement) sags KF velocity 82 → 48 m/s, holding the burnout
detector's low-passed acceleration below the −4 m/s² latch threshold for
~0.5 s — mid-burn. At 213.5 Hz the slower-in-wall-time filter smeared it out;
at the corrected rate Coasting latched at burn+0.44 s, which would have
re-enabled airbrakes under thrust via the new start gate.

**Second mechanism:** after every 1 Hz logger stall (finding 6), the KF
re-converges its altitude deficit as a ~150 ms positive-acceleration burst,
nudging the filter above threshold once per second — a strict
consecutive-sample persistence counter could never complete during coast.

**Fix** (`baro_state_estimator`): coasting latches only after 1 s of
*consecutive* low-passed acceleration below threshold. The liftoff dip holds
only ~0.5 s — half the required persistence — so it cannot latch mid-burn;
real burnout accrues straight through.

> **2026-08-13 (estimator rework, phase A) — detector deleted entirely.**
> Coasting is now a burn timer: latched `MAX_BURN_TIME_US` after ignition
> detection, reading no KF output at all. Both failure mechanisms above are
> structurally impossible — the liftoff pressure dip and the stall
> re-convergence bursts have nothing to reset. (The interim accel-based
> detector went through clamp+leaky and strict-counter variants earlier the
> same day; see git history.)

## 6. Sensor stream stalls 104 ms every second — NOT FIXED (cause unknown)

The IMU+baro stream freezes for ~104 ms at exactly 1.000 s intervals throughout
armed mode (~10 % of samples lost; 22 at the old rate, ~44/s at the new).
No timestamps exist inside the gaps despite a 512-deep logger channel, so the
*publisher* (sensor task) stalls — the flight estimator experienced the same
freezes, this is not just a logging artifact. Suspects: a 1 Hz task starving
the executor or a shared bus (GPS NMEA burst parsing, CAN status, slow-record
heartbeat). The reworked deployment estimator (slow filter + gate + timers,
2026-08-13) degrades gracefully through the stalls — no deployment decision
depends on a stall-free stream anymore. The root cause is still worth
hunting: the stalls cost ~10 % of the science data, and the phase-B fusion
estimator (IMU dead-reckoning for airbrakes) will care much more than the
deployment path does.

## 7. Measurement-level observations (no code change)

- **Baro aero overshoot during boost:** baro-derived climb rate peaked at
  ~350 m/s while coast energy bounds true burnout speed near 145–200 m/s —
  static-port position error at high dynamic pressure. Error vanishes as speed
  → 0, so apogee altitude is accurate. No baro-only filter can remove this;
  keep margin on velocity-derived gates during boost.
- **KF is well-tuned at rest:** pad innovation σ 0.499 m vs model 0.501 m,
  NIS 0.99 (ideal 1.0) — the bench-measured R matched the field exactly.
- **Corrected-DT filter quality:** velocity ratio 1.04–1.06 vs reference,
  ~165 ms group delay. NIS during dynamics is ≫1 (boost ~600) — expected for a
  constant-velocity model; thresholds on KF outputs should assume it.
- **Descent detection shifts ~0.8 s earlier** at the corrected rate (the
  −2 m/s threshold is now true velocity, not half). Within the apogee plateau
  at |v| < 10 m/s — benign, but expect earlier drogue timing than this log.
- **Flight profile discrepancy:** this flight deployed with apogee 1 514 m AGL,
  but current `main.rs` sets `drogue_chute_minimum_altitude_agl: 2000.0`, under
  which this flight would have ended `FailedToReachMinApogee` with no
  deployment. Confirm the constant matches the next flight's expected apogee.

---

## 8. Accelerometer clipped at ±16 g during boost — HARDWARE CEILING

The LSM6DSM's thrust-axis reading railed at exactly −16.0 g for 191 samples
(~0.9 s cumulative) during a burn whose mean acceleration was only ~5 g:
motor vibration spikes ride the thrust past the rail, one-sided (positive
swings only reached +4 g). ±16 g is the LSM6DSM's hardware maximum — this is
not a configuration fix. The clipping is asymmetric, so it rectifies into a
bias through exactly the window that calibrates the fusion estimator's
dead-reckoner and seeds Stage 2 (see ESTIMATOR_REWORK_PLAN.md, the tilt fix
follow-up). For reference, the LC'25 recorder measured 23+ g peaks on that
flight; the LC'26 vehicle is expected to accelerate well below the ±16 g
rail (per sim), so clipping on the real flight should be limited to
vibration spikes like Void Lake's, not sustained. Deployment is immune
(baro-only +
timers); the exposure is the fusion estimator's burn-phase velocity
integration — its lockout-exit re-convergence onto clean subsonic baro is
the recovery mechanism, and gyro (orientation) is unaffected. Options if
more margin is wanted: a high-g accelerometer on a future board spin, or
in-flight clip detection (raw sample at the i16 rail) to discount the accel
channel while clipped.

## Validation methodology

- **Replay harness:** logged pressure → `BaroData::altitude_asl()` → the real
  estimator/KF code, at the logged cadence. A Python replica of the KF
  (verified to <1 mm / <6 mm/s against the Rust output over the full flight)
  provided innovation/covariance internals for the diagnostics figure.
- **Fixed-rate dataset:** the log interpolated to 427 Hz — midpoints inserted
  only within contiguous runs, the 1 Hz stall gaps preserved as gaps — to test
  the post-fix configuration. This dataset is what exposed finding 5.
- **Unit tests:** 36 in `air-brakes-controller-core`, including coasting
  detection, transient rejection, and gate anti-lockout.

## Pre-flight checklist (no further test flight planned)

1. Bench-verify sample cadence ≈ 2.34 ms (finding 3) and absence of SD queue
   warnings.
2. Re-bench baro noise at OSR=512; update `BARO_ALTITUDE_MEASUREMENT_VARIANCE`.
3. Sky-view check that GPS altitude tracks (finding 1).
4. HIL replay: `HIL: starting airbrakes` must appear only after
   `flight_stage PoweredAscent -> Coasting`.
5. Confirm `FLIGHT_PROFILE` minimum-deployment altitude vs expected apogee.
6. Optional: add a defmt log line on innovation-gate rejection for visibility
   during ground pyro tests.
