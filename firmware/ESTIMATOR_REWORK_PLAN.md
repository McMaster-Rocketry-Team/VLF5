# Estimator Rework Plan — Two-Filter Architecture

Decided 2026-08-13. Deployment reliability and airbrakes accuracy are separated
into two estimators. The deployment path is a slow, trusted, COTS-style
altimeter (Phase A, built). The airbrakes path was first built as a 6-state
EKF (Phase B v1, built, kept as history below), then redesigned 2026-08-15
into the much simpler Phase B v2 after a red-team review of the design.
Deployment never depends on the complex filter.

```
                 ┌────────────────────────────────────────────────┐
 IMU+baro 416 Hz │ SLOW BARO KF (~1 s bandwidth) + state machine  │→ pyros, stage,
 ───────────────►│ gate → ignition → mach timer → peak-drop apogee│  telemetry,
        │        │ → deploy → landed                              │  airbrakes GATE
        │        └────────────────────────────────────────────────┘
        │        ┌────────────────────────────────────────────────┐
        └───────►│ AIRBRAKES ESTIMATOR (v2)                       │→ MPC state
                 │ gyro-only tilt · dead reckoning through the    │  (alt, vertical
                 │ mach lockout · 2-of-3 vote that baro is honest │  velocity, tilt)
                 │ again · small alt/velocity filter built fresh  │
                 │ at that moment                                 │
                 └────────────────────────────────────────────────┘
```

## Phase A — deployment estimator rework (`baro_state_estimator`)

> **Status: implemented 2026-08-13, uncommitted, holding for review.**
> Q = 0.115 (τ = 1.00 s, gains 0.0024/0.0012, stationary velocity noise
> 0.012 m/s std); gate widened to 500 m (slow-filter boost lag reaches
> ~330 m at 16 g — see updated rationale in altitude_kf.rs). Validated by 10
> new/reworked unit tests plus a full replay of the real Void Lake flight log
> (`void_lake_flight_replay`, CSV now in the crate's test_data): ignition,
> burn-timer coasting, apogee at 1500.2 m AGL through the real ejection-blast
> transient, main at 457 m AGL, landed. All 40 crate tests pass; all six
> firmware build configs compile. `FlightProfile` is now a struct carrying
> both timers (`mach_lockout_duration_us`, `max_burn_time_us`) plus the
> `DeploymentProfile` enum; sim-derived values for the real rocket are still
> TODO in main.rs FLIGHT_PROFILE.

The philosophy shift: the filtered altitude is *trusted*; robustness comes from
bandwidth + input validation + timers, not from derivative-signal detectors.

1. **Retune to ~1 s bandwidth**: `PROCESS_ACCELERATION_VARIANCE` 1150 → ~0.11
   (σ_a ≈ 0.34 m/s²). Stationary velocity noise drops ~0.37 → ~0.04 m/s.
   Deployment thresholds go from ~5σ to ~50σ margins.
2. **Keep the innovation gate + force-accept unchanged.** The gate is input
   validation for the raw bus (a single pressure=0 sample decodes to ~30 km and
   would still kick the slow filter ~70 m / ~35 m/s); bandwidth cannot replace
   it.
3. **Mach lockout becomes a timed stage after ignition** (COTS "Mach delay"):
   `OnPad → (ignition) → MachLockout(duration) → Ascent`. Delete the
   velocity-threshold entry (`MACH_LOCKOUT_ENTER`, `lockout_done`) — the slow
   filter lags ~100+ m/s under boost, so velocity entry would trigger past
   Mach 1 on garbage data. Freeze + reseed at exit stays as built.
   Config: duration = sim (time from ignition detect until back below
   Mach 0.75) × ~1.4, must end >5 s before earliest apogee. `None` = subsonic.
4. **Apogee = peak-drop**: track running max of filtered altitude in Ascent;
   descent detected when altitude < peak − 30 m sustained 0.5 s. Deletes the
   velocity descent threshold. Min-apogee check uses (peak − pad).
5. **Coasting = burn timer**: `samples_since_ignition > max_burn_time` (sim,
   +margin). Deletes the entire accel low-pass / coast counter machinery — and
   with it the finding-6 flight dependency: coasting no longer needs a
   stall-free stream (the stall fix reverts to a data-quality issue).
   Timer runs through lockout (it doesn't need the KF), so the Coasting stage
   is reported correctly even while locked out.
6. **Ignition, landed, min-apogee, delays: unchanged.** Ignition triggers
   ~1–1.5 s later on the slow filter (harmless; lockout duration is measured
   from detection). Landed |v| < 2 m/s for 5 s gets ~10× more noise margin.
7. **Accepted consequences**: main fires ~1 s of descent late (≈25–35 m low at
   drogue rates — bake margin into `main_chute_altitude_agl`); drogue fires a
   few seconds past apogee at ~20–30 m/s descent (normal for COTS logic).
8. **Tests**: retune flight sims; add ejection-blast-at-apogee transient test;
   adapt the supersonic-garbage test to timer lockout; Void Lake CSV replay of
   the deployment chain.

## Phase B v1 — airbrakes fusion estimator (6-state EKF) — SUPERSEDED

> **This version was built and works, but is superseded by the v2 design
> below (2026-08-15).** It is kept here because the bugs we found and fixed
> in it are the evidence behind the v2 design decisions. The code and its
> flight-replay tests stay in the repo until v2 replaces them.

> **Status: implemented 2026-08-14, uncommitted, holding for review.**
> `state_estimator2` restructured into `fusion_estimator`: the ascent
> estimator promoted (public `AscentStateEstimator` + slimmed `FusionConfig`,
> just the ignition accel threshold), the dead outer deployment machine and
> descent KF deleted. Added a baro innovation gate (100 m, tilt-only update
> on rejection, 2 s force-accept) to the velocity EKF. The plot-only test is
> replaced by two asserting replays of real flights, both resampled onto the
> 416 Hz grid: **Void Lake** (subsonic: lockout must not engage; coast
> velocity mean error 3.4 m/s vs baro reference, apogee −49 m/−1.9 s,
> through the real ejection blast) and **LC'25** (supersonic: lockout must
> engage and release; coast velocity 2.3 m/s, apogee −23 m/−2.5 s). Noise
> parameters kept and documented as replay-validated. Firmware untouched —
> wiring is Phase C.
>
> **2026-08-14 — tilt-absorption fix (the "last year's flight" tilt bug).**
> Diagnosed on the Void Lake log (a `#[ignore]`d `void_lake_tilt_diagnostics`
> test prints the decomposition): during the boost baro overshoot unwind, the
> EKF reconciled the apparent climb deficit by pushing tilt ~20 deg above its
> own gyro-derived axis measurement — i.e. ~20 deg of sustained angle of
> attack, aerodynamically impossible for a stable rocket in high-q coast —
> and inventing ~150 m/s of horizontal velocity, because the model had
> nowhere else to put a baro error. (The baro's own unwind rate, ~50 m/s^2 of
> apparent decel vs the ~15 physics allows, independently proves the error is
> a sensor artifact. GPS could NOT serve as truth here: auditing the log
> shows the receiver's pre-airborne-4g nav filter froze gps_alt at pad
> altitude through the whole ascent, produced position-jump artifacts, and
> dropped fix just before apogee — consistent with "we lost GPS on the way
> up", and all evidence for this fix is IMU/baro-internal.) Fix: a 6th EKF
> state — the static-port position-error coefficient `c`, with the altitude
> measurement modeled as `z + c*speed^2` (position error is proportional to
> dynamic pressure; the s^2 time signature makes `c` observable during
> boost). After: tilt tracks its axis measurement to -0.2 deg (Void Lake) /
> -3.8 deg (LC'25), mean coast tilt drops 65 -> 43 deg, and apogee timing
> improved (-1.0 s vs -1.9 s). Both replays now assert tilt-vs-measurement
> divergence < 10 deg, so this exact failure mode is regression-guarded.
>
> **Follow-up investigation (same day):** two chart observations checked.
> (1) The ~52 deg tilt excursion at t~13-14 is the Stage-2 *entry transient*
> — present in both filters (the before-filter rings against the 0 deg clamp
> there and then step-jumps 50 deg); it happens mid-burn, when nothing
> consumes fusion output. Root: the dead-reckoner integrates through the
> liftoff shake, handing Stage-2 a seed with ~50 m/s of spurious off-axis
> velocity; the seed speed is now the reckoner velocity *projected onto the
> rocket axis* (zero-AoA argument). (2) The after-filter's ~170 m/s burnout
> peak is consistent with truth: back-integrating honest late-coast baro
> through a fitted drag model gives 170-185 m/s — the 281 m/s baro reference
> at burnout is the corrupt signal. Genuine residual: c converges
> wrong-signed (-3.1e-3 vs the port's physical +2-3e-3) because the boost
> transient dominates its observability window on a no-lockout flight,
> biasing mid-coast vertical velocity ~10-15 m/s low (decays to 0 by
> apogee). Sign-clamping c was tested and REJECTED: starving the sink
> instantly false-latches apogee at Stage-2 entry (and a naive state clamp
> corrupts the covariance). On lockout flights c never fuses boost-window
> baro, so the contamination is largely absent; accepted and documented in
> the code.
>
> **2026-08-14 — lockout-exit transient + coast-constraint ratchet fix.**
> Found on the LC'25 replay charts (tilt jumped +10 deg in one step at
> lockout exit, froze at 21 deg for 20 s, then slewed 21->75 deg in 2.5 s
> near apogee). Two coupled bugs: (1) the first baro sample after lockout
> exit dumped 5.7 s of accumulated dead-reckoning drift through the
> outage-grown cross-covariances into tilt/speed/c (the tilt-bias state
> moved in lockstep, proving the yank came through the altitude row);
> (2) the coast monotonic constraint refreshed its references only in
> predict(), so every measurement-driven tilt decrease was silently undone
> on the next step — any upward overshoot became a permanent ratchet floor,
> with omega pinned at -8.6 deg/s fighting the clamp while the bias state
> absorbed the growing measurement offset. Fixes: `reseed_altitude()` at
> lockout exit (z := baro - c*s^2, z cross-covariances cut — the outage
> drift belongs to z alone, mirroring the slow filter's exit reseed), and
> the constraint references now refresh after update() too, so monotonicity
> binds only the dynamics, never measurement corrections. After: tilt-vs-
> axis error over LC'25 coast tightened from -21..+15 deg to 0..+6.8 deg
> (steady ~4.5 deg = learned tilt bias), c stays ~+0.17e-3 instead of
> tripling at exit, velocity/apogee metrics unchanged, all 41 tests pass.
> Note: `constraints_enabled` is only ever set at lockout exit, so
> non-lockout flights (Void Lake) never run the coast constraints —
> unchanged by this fix, possibly worth revisiting.

## Phase B v2 — airbrakes estimator, simplified design (CURRENT PLAN)

> **Status: implemented 2026-08-15 (`airbrakes_estimator` module + MPC
> restore), uncommitted, holding for review.** All five pieces built as
> specified below, plus the three red-team-mandated tests. Replay results
> (both flights fed RAW timestamps, no resampling): **LC'25** — filter born
> at t=15.9 s via the vote (not the timer; inertial vote flipped at 14.8 s,
> just after the true 0.75 M crossing at ~14.6 s), coast velocity mean
> error 2.31 m/s, apogee −2.3 s / −14.8 m. The baro-rate vote flickered
> ~95 times through the transonic region (the predicted zero-crossing
> spoof) and never held — the sustain requirement did its job. **Void
> Lake** — born right after thrust alignment (subsonic profile), apogee
> −1.2 s / −17.2 m through the real ejection blast, coast velocity 9.3 m/s
> (this airframe's large port error, corrected with a fixed c-hat=2.5e-3),
> clip counter caught the real ±16 g railing. **Clipped-accel LC'25** —
> with the accel clamped to ±16 g, the lying inertial vote alone did NOT
> open the lockout (born at 15.9 s, still honest), and velocity recovered
> to within 5 m/s of reference by birth+5 s. The v1 6-state EKF remains in
> the repo (all its tests still pass) until this replaces it in Phase C.
> One deviation from the design text: lockout ENTRY became unnecessary —
> "born subsonic" means the baro is never fused before birth at all, which
> is strictly more conservative than any entry rule (the entry paragraph
> below is kept for the record). The vote-truth-table test is merged into
> the LC'25 replay test.
>
> Before we settled on
> this design, it was red-teamed: 8 independent reviews looked for holes and
> proposed simpler alternatives, and the serious claims were checked by
> replaying candidate designs against our two real flight logs (Void Lake,
> subsonic; LC'25, Mach 2). 29 problems were raised, 4 were confirmed with
> hard evidence, and this design includes their fixes. The single most
> important result: our first idea for "how do we know the baro is honest
> again" was replayed against the real Mach 2 flight and NEVER passed — it
> would have silently turned every flight into the plain timeout we
> explicitly wanted to avoid. That check is redesigned below.

### What this estimator is for, in one paragraph

The airbrakes controller (MPC) needs three numbers: altitude, vertical
velocity, and tilt (how far the rocket leans from straight up). It only
needs them to be accurate in one window: after the motor burns out and the
rocket has slowed below Mach 0.8, until apogee. That is the only time the
airbrakes are allowed to move. The hard part: the baro lies whenever the
rocket is near or above the speed of sound (shock waves change the pressure
at the sensor port), so we must ignore it while it lies — and reliably
notice when it starts telling the truth again, without just trusting a
stopwatch.

### Piece 1 — tilt: integrate the gyro, no filter

Tilt comes from adding up gyro rotations step by step, starting from a
known position on the pad. No Kalman filter is involved, and none is
needed: the old 6-state EKF could not improve absolute tilt either — its
tilt input WAS this same gyro signal.

- **On the pad**, measure the gyro's zero offset ("bias") — the small fake
  rotation it reports even when sitting still. This calibration is the
  whole error budget: with a clean bias the tilt is good to ~1–3 degrees at
  apogee; with a bad one it can be off by 12 degrees (measured on the Void
  Lake log). The trap we measured: the rocket slowly sways on the rail, so
  a single short window can average the sway into the "bias" and look
  perfectly quiet while doing so. Fix: average over 30–60 seconds, split
  into several windows, and require the window AVERAGES to agree with each
  other (not just each window to be quiet). Carry the leftover disagreement
  as an explicit error margin for the Mach check in Piece 3.
- **At ignition**, find "which way is the rocket pointing relative to
  gravity" from the direction of the thrust push, exactly as the existing
  Stage 1 code does today. Keep it.
- **After that**, just integrate. The gyro never came close to its limit in
  flight (peak 663 deg/s of a 2000 deg/s range).

### Piece 2 — timekeeping: use real timestamps everywhere

Everything today assumes samples arrive at exactly 416 per second
(a hard-coded time step). The red team confirmed two ugly facts: samples
have NOT always arrived on time (the Void Lake log has 104 ms gaps about
once per second, ~8% of flight time lost), and our test suite is blind to
this because it resamples the logs onto a perfect grid before replaying
them. With no filter correcting tilt (Piece 1), a timing error becomes a
permanent angle error — integrating the real flight gyro with the assumed
time step instead of the real one ended up 80 degrees off at apogee.

Fixes: add a timestamp to every measurement; integrate with the real time
between samples (limited to a sane range); add a replay test that feeds
the raw, gap-filled log WITHOUT resampling; log the achieved sample rate
in flight so a regression is caught on the bench, not at apogee.

### Piece 3 — deciding "the baro is honest again" (the crux)

While the rocket is fast, we ignore the baro and track altitude and
velocity by dead reckoning — integrating the accelerometer (rotated by the
Piece-1 tilt) the same way we integrate the gyro. This drifts slowly, but
on the real Mach 2 flight the drift was small (about +126 m at apogee with
a clean accelerometer).

Two timers from the flight simulation bound the decision window: before
T_min the rocket physically cannot be subsonic, and at T_max we give up
waiting and proceed anyway. Between the timers, we declare the baro honest
when **any 2 of these 3 checks agree, continuously, for about 1 second**:

1. **Inertial speed check**: the dead-reckoned TOTAL speed (not the
   vertical part divided by cos(tilt) — the red team showed that form
   amplifies tilt errors enough to eat the whole safety margin) is below
   0.75 Mach. We use 0.75 to leave margin below the real 0.8 requirement.
2. **Slow filter check**: the deployment filter's speed estimate is below
   0.75 Mach. This filter already exists, is already trusted to fire
   pyros, and its slowness makes it read HIGH during deceleration — so it
   errs late, which is the safe direction.
3. **Baro rate check**: the baro's climb rate (with the port correction
   from Piece 4) matches the dead-reckoned vertical velocity to within
   ~15 m/s. This is the direct "is the baro currently sane" test — a
   shock-corrupted baro has a wildly wrong slope.

Why a 2-of-3 vote instead of "all checks must pass"? Because we tested
"all must pass" against the real Mach 2 flight and one check — comparing
absolute baro altitude against dead-reckoned altitude within 150 m —
NEVER passed (the gap was already 163 m at the crossing and growing).
Dead-reckoned altitude drifts even when the baro is fine, and drift is
harmless — refusing the baro because of it is wrong. With "all must
pass", any single stuck check silently turns the system into a plain
timeout. A 2-of-3 vote survives one stuck or lying check in either
direction.

Entry into the lockout is the mirror image: dead-reckoned total speed
above 0.9 Mach, or a sim-derived timer after ignition, whichever comes
first.

Optional extra vote if we want more independence: in coast, the
accelerometer directly measures drag, and drag maps to airspeed through
the same drag table the MPC already owns. That gives an instant airspeed
reading with no integration, no baro, and no sensitivity to boost
accelerometer clipping. Cheap to add; decide during implementation.

### Piece 4 — altitude & vertical velocity: a small filter, born subsonic

Once Piece 3 says the baro is honest, we construct a tiny 2-state filter
(altitude + vertical velocity) AT THAT MOMENT: altitude starts from the
median of the first few corrected baro readings, velocity starts from dead
reckoning. From then on it predicts with the accelerometer and corrects
with the baro through the existing 100 m sanity gate.

The point of "born subsonic": the filter simply does not exist during the
period when the baro lies. All three bugs we found and fixed in the v1 EKF
lived in the code paths where a running filter crossed the lockout
boundary (garbage leaking through covariances at exit, the ratchet freeze,
the boost error sink). If there is no filter to poison, that entire class
of bugs cannot be written.

Details the red team forced into the spec:

- **The baro port correction is mandatory, not optional.** The baro reads
  high by roughly ĉ × speed², where ĉ is a fixed property of the airframe
  (LC'25 measured +0.7e-3; get LC'26's from CFD/sim or its first flight).
  At the Mach 0.8 crossing this error is ~50 m or more — bigger than our
  error budget — so the correction is applied to every baro reading used
  anywhere: the Piece 3 rate check, the filter's start value, and every
  update. Without it, a replay showed vertical velocity reading 50–63 m/s
  high at burnout on the subsonic test flight.
- **The "force accept" path must re-anchor velocity too.** Today, when the
  baro disagrees with the filter for 2 s straight, the filter snaps its
  altitude to the baro but leaves velocity untouched — so a velocity error
  survives forever. Fix: route that path through the same "start fresh"
  logic (re-anchor altitude AND re-open the velocity correction).
- **Apogee is latched only with persistence and a healthy baro**: vertical
  velocity below 1 m/s for half a second, while recent baro samples are
  changing and passing the gate. Today a single noisy sample latches it
  permanently.

### Piece 5 — the MPC: restore the 2D version, fix its density model

Restore the original MPC (git 78bfd5f): it takes altitude plus a 2D
velocity vector, and simulates the rest of the flight with a drag table to
pick the brake extension. Horizontal velocity is computed from vertical
velocity and tilt (capped at 80 degrees so the math stays sane near
apogee, where the brakes have no authority left anyway).

Bug found by the red team, must fix: the MPC's air-density formula is a
straight-line fit only valid to 3000 m, but LC'26 flies to 6+ km. At
5.7 km it under-reads density by 8.7%, which under-predicts drag, which
over-predicts apogee, which OVER-extends the brakes — a one-sided error in
the harmful direction. Replace with the standard atmosphere formula
(one line).

Also: count accelerometer samples that hit the ±16 g limit during boost
and set a "degraded" flag if any occur (Void Lake hit the limit; LC'26's
sim says it won't, but we should know if it does).

### Considered and not chosen (so we don't re-litigate later)

- **Fixed gains instead of a Kalman filter** for Piece 4 (the pattern COTS
  altimeters fly): fine, and even simpler — decide during implementation;
  the born-subsonic structure matters much more than this choice.
- **Pre-computing the MPC into a lookup table** (all math checked on the
  ground, ~20 lines at runtime): a real option for flight-readiness review,
  not chosen now because we want the runtime 2D MPC restored first.
- **Pure feedforward (no closed loop at all)**: rejected — it is open-loop
  against exactly the things that vary on flight day (density, drag, wind,
  mass), which the closed loop corrects for free.
- **Estimating the port coefficient ĉ in flight** (the v1 approach):
  rejected — on no-lockout flights the boost transient teaches it the
  wrong sign, and a fixed per-airframe constant does the job.

### What carries over from v1 unchanged

Pad calibration buffer + ignition detection, Stage 1 thrust alignment, the
dead reckoner, the ĉ-corrected reseed logic, the 100 m gate structure, and
both real-flight replay test harnesses. The 6-state EKF, the tilt/bias/c
states, and the coast monotonic constraints are deleted.

## Phase C — integration (VLF5 armed_mode)

> **Status: implemented 2026-08-15, uncommitted, holding for review.**
> All four items below are wired; the v1 `fusion_estimator` module is
> deleted (43 crate tests, 73 firmware-common-new tests, and all three
> firmware build configs pass). Notes from the implementation:
> the airbrakes estimator gets each sample's IMU-capture timestamp from
> the sensor pubsub (`SensorReading.timestamp_us`, taken at the IMU
> data-ready interrupt — the honest dt source) and the slow filter's speed
> as vote 2; the MPC reads the airbrakes estimator's altitude + 2D
> velocity and FALLS BACK to the slow filter's vertical-only state
> whenever the vertical filter is not running — which is also how HIL
> keeps working: the bench IMU never feels ignition, so in HIL builds the
> estimator stays OnPad, the gate runs on the slow-filter conditions
> alone, and the MPC uses the fallback path. The SD fast record (storage
> v5) logs both estimators plus the vote bits / born / apogee / clip flags
> per sample, so the real flight will produce the same exit truth table
> the LC'25 replay does; rocket-cli exports the new columns.

- Run both estimators off the same IMU+baro subscription (now timestamped —
  Piece 2). Slow estimator owns: pyros, FlightStage, telemetry
  altitude/speed, all gates. Airbrakes estimator owns: MPC input (altitude +
  2D velocity vector — and tilt, which finally makes the telemetry
  `tilt_deg` field real).
- Airbrakes gate = burn timer expired (slow) AND slow-KF subsonic (lags high
  during coast = opens late = safe) AND ascending AND the airbrakes
  estimator past its lockout exit (Piece 3 vote passed or T_max).
- SD fast record: log both estimators (slow alt/vel + airbrakes
  alt/vel/tilt + which Piece-3 votes are passing) → storage format v5 +
  rocket-cli columns.
- CAN RocketStateMessage: velocity/altitude from the airbrakes estimator,
  coasting from burn timer.

## Phase D — validation & prerequisites

- All crate unit tests + Void Lake replay through both paths.
- New tests required by the v2 design (all from red-team findings):
  - **Raw-timestamp replay**: feed the Void Lake log with its real gaps, no
    resampling — catches any hidden "assumes 416 Hz" logic.
  - **Clipped-accel replay**: LC'25 with the accelerometer artificially
    clipped at 16 g — assert the Piece 3 vote still refuses to exit early
    and that the T_max path recovers within ~2 s.
  - **Vote truth table**: replay LC'25 and assert each of the three votes
    flips at a sensible time and none flips while genuinely supersonic.
- HIL replay (hil-single/dual) on hardware; lockout None + test-rocket burn
  time for the replay profile.
- Still open before flight: sensor stall root cause (stalls now also
  degrade Piece 1/2 accuracy, not just data quality), OSR=512 baro noise
  re-bench (R constant), sim-derived numbers (T_min, T_max, max burn time,
  main altitude margin), ĉ for the LC'26 airframe (CFD/sim or first
  flight), LC'26 sim margin to the ±16 g accelerometer limit.

## Order

A first (small, flight-critical, independently shippable, DONE) → C's
slow-path integration → B v2 (timestamps first, then pieces 1/3/4, then the
MPC restore) → C's airbrakes integration → D throughout.
