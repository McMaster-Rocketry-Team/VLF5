# Estimator changes we want to make later

Written 2026-08-15, from the walkthrough discussion. These are agreed
directions, not yet implemented. Do them together — they interact.

## 1. Make `MachLockout` a real `RocketState` variant

Today the deployment estimator reports `Ascent` during its Mach lockout,
carrying **stale frozen** altitude/velocity, and callers must remember to
check `in_mach_lockout()` before using the numbers. Nothing forces them to.

Change: add `RocketState::MachLockout { launch_pad_altitude_asl }` with
**no altitude and no velocity fields**, so reading stale numbers becomes
impossible instead of just discouraged. Remove `in_mach_lockout()` once no
caller needs it.

Touches: every `match` on `RocketState` (~6 sites in armed_mode +
telemetry), and item 2 below.

## 2. Telemetry reports the rocket state honestly — no derived enums

Today armed_mode derives `FlightStage` (PoweredAscent/Coasting/...) from
`RocketState` + the coasting flag, and folds states together (e.g.
`FailedToReachMinApogee` is reported as `Landed`, MachLockout as ascent).
The ground sees a translation, not the truth.

Change: the flight part of the reported state mirrors `RocketState`
variants 1:1, including `MachLockout` (item 1) and
`FailedToReachMinApogee` as their own values. Coasting stays a separate
boolean (it is a timer flag, orthogonal to the state). Mode-level values
(LowPower, SelfTest, Armed) stay as they are.

Touches (wire formats): `FlightStage` in `vl_status.rs` (CAN), VLP
telemetry packet, `flight_data_record.rs` / `flight_storage.rs` (storage
version bump), rocket-cli decoding, ground station display.

## 3. Airbrakes must not be blocked by the deployment estimator's Mach lockout

Today the airbrakes gate requires `!in_mach_lockout`. The slow lockout is
sized with ~1.4x margin, so on a nominal flight it expires *after* the
airbrakes estimator's vote has already passed — the padded dumb timer, not
the smart vote, decides when braking starts, wasting several seconds of
the control window. The vote was built precisely to avoid that.

Change: remove `!in_mach_lockout` from the gate. The slow lockout then
only protects the deployment KF (its real job), and its margin no longer
costs airbrakes time — which also dissolves the "how much margin can we
afford" sizing tension for `mach_lockout_duration_us`.

Things that must move with it (this is why items 1-3 go together):

- The gate's `vv <= 0.85 * speed_of_sound` sanity check currently reads
  the slow filter's velocity, which is stale/frozen during the lockout.
  Move that check to the airbrakes estimator's own KF velocity (it exists
  exactly when `baro_trusted()` is true). With item 1, the stale read is
  impossible anyway because `MachLockout` carries no velocity.
- New gate sketch: `coasting` (slow filter burn timer — keep, thrust
  protection stays on the most trusted source) `&& ab_baro_trusted &&
  ab_vv > 0 && ab_vv <= 0.85 * sos(ab_alt)`.
- Do a deliberate safety pass: this removes one independent layer (the
  sim timer) from the "never open supersonic" protection. Remaining
  layers: T_min floor, the sustained 2-of-3 vote, T_max-forced births
  flagged via `born_forced`, and the 0.85*sos check on the airbrakes KF.
- Known weakness to fix alongside: while the slow filter is locked out,
  its frozen (low) velocity makes vote V2 spuriously "subsonic", so the
  2-of-3 vote degenerates to "V1 or V3". Pass `None` as
  `deployment_speed` while the slow filter is in lockout, so V2 honestly
  abstains and the vote requires V1 *and* V3 in that window.

## 4. Gravity direction from the same 32-window structure as gyro bias

Today gravity (pad orientation) and pad altitude come from the first half
of the 2 s pre-ignition ring buffer (~1 s of data). The gyro bias already
has a much better structure: 32 screened 2 s windows covering the last
minute. Use that same structure for the accelerometer (and baro) too:
collect accel/baro window means alongside gyro means, screen them the
same way (rail sway pollutes the gravity direction exactly like it
pollutes the bias), and take gravity + pad altitude from the surviving
windows. The 2 s ring buffer then has only one remaining job: replaying
the dead reckoner through the ignition shake (the rewind trick).

## 5. Remove the fallback-bias logic — require full calibration on pad

The `screen_bias` fallback (fewer than 3 finished windows -> use the ring
buffer mean with a pessimistic 0.3 deg/s spread) exists for "powered on
seconds before launch", which is not a real operational scenario — the
rocket always sits armed on the rail for minutes. Remove the fallback;
instead, don't allow ignition detection until enough screened windows
exist (calibration-complete is a precondition of being launch-ready, and
can be surfaced as a self-test/arming condition). Same for the "all
windows disagree -> median + pessimistic spread" branch: keep the
screening, but treat a totally disagreeing pad as not-launch-ready
rather than silently flying with a guess.

## 6. Remove the accel-clipping logic

`ACCEL_CLIP_LIMIT`, the `accel_clipped` counters, the `AB_ACCEL_CLIPPED`
flag and its CSV column. Clipping the +/-16 g accelerometer happens on
small high-thrust rockets (Void Lake), never on a large airbrakes-class
rocket (LC'26 sims stay well under 16 g). The logic adds a per-sample
check and a field that rides through three states for a condition that
cannot occur on the airframe this estimator flies on. (Wire format: the
flag bit and CSV column go with it — storage version bump, or leave the
bit reserved.)

## 7. Vote threshold at Mach 0.8, not 0.75

`VOTE_MACH` is 0.75 today, sold as "margin below the real Mach 0.8
requirement". But 0.8 is already the number with margin built in, and the
vote stack has margin layered on top anyway: V1 subtracts an explicit
uncertainty margin, V2 lags high, the 1 s sustain delays the decision,
and the born filter converges before the gate opens. Detecting at 0.75
double-counts margin and costs control-window seconds (speed decays
slowly near the crossing, so 0.05 Mach is not free). Change `VOTE_MACH`
to 0.8. (The slow-filter 0.85*sos gate check is a separate constant and
stays.)

## 8. `FlightEstimators` composition struct

One struct in air-brakes-controller-core that owns both estimators and the
policy connecting them. Build it TOGETHER with items 1-3 — its gate is the
post-rework gate. Firmware then holds one struct behind one mutex.

```rust
pub struct FlightEstimators {
    deployment: RocketStateEstimator,
    airbrakes: AirbrakesEstimator,
}

pub struct ImuSample {
    pub acc: Vector3<f32>,   // m/s^2
    pub gyro: Vector3<f32>,  // rad/s (firmware converts units at the edge)
}

pub struct AirbrakesMPCStates {
    pub altitude_asl: f32,
    pub velocity: Vector2<f32>, // [horizontal, vertical] m/s
}

impl FlightEstimators {
    pub fn new(profile: FlightProfile, config: AirbrakesConfig) -> Self;

    /// The ONLY mutating function; call once per ~416 Hz sample. Baro
    /// always present, IMU optional (airbrakes estimator skipped, its
    /// measured-dt path bridges). Returns the pyro command passed through
    /// from the deployment estimator UNTOUCHED. Inside: the V2 feed
    /// policy — deployment speed goes to the vote, `None` (abstain)
    /// while the slow filter is in Mach lockout (item 3's fix).
    pub fn update(
        &mut self,
        timestamp_us: u64,
        imu: Option<&ImuSample>,
        baro_altitude_asl: f32,
    ) -> Option<PyroSelect>;

    /// `Some` exactly when the airbrakes are permitted to open:
    /// coasting (slow filter burn timer) && the airbrakes filter is
    /// alive (baro trusted, pre-apogee) && ascending && vv <= 0.85 *
    /// speed_of_sound(alt), all on the airbrakes filter's own state.
    /// Permission and state availability are one Option — "permitted
    /// but no state" cannot be expressed.
    pub fn airbrakes_mpc_states(&self) -> Option<AirbrakesMPCStates>;

    pub fn state(&self) -> RocketState;

    // read-only component access for everything else (is_coasting,
    // votes, birth, tilt, log-flag assembly, ...)
    pub fn deployment_estimator(&self) -> &RocketStateEstimator;
    pub fn airbrakes_estimator(&self) -> &AirbrakesEstimator;
}
```

Decisions made:
- No `airbrakes_permitted()` / `mpc_state()` pair — `airbrakes_mpc_states`
  replaces both.
- No log-state helper: the caller assembles the fast-record payload from
  the `airbrakes_estimator()` reference.
- No `&mut` component accessors: "data crosses between the estimators by
  value, inside update, once per sample" is enforced by the API.

Stays in firmware: the `airbrakes_started` latch (act-once is control
flow), the HIL fallback (with no `mpc_state` fallback chain, the
hil-replay build assembles MPC state from the slow filter under its own
cfg — the hack lives entirely in firmware), unit conversion, the mutex.

Note: the struct inherits both time bases — deployment half is
sample-clocked (assumes the real ~416 Hz stream, which armed mode
provides), airbrakes half is wall-clock. Replay tests against the 500 Hz
LC'25 log keep resampling for the deployment half, raw timestamps for
the airbrakes half (the existing test harness already does this).
### What armed_mode looks like after (items 1-3 + 8 + 9 together)

Setup: ONE mutex around one struct.

```rust
let estimators = BlockingMutex::<NoopRawMutex, _>::new(RefCell::new(
    FlightEstimators::new(FLIGHT_PROFILE.clone(), AIRBRAKES_CONFIG.clone()),
));
```

The 416 Hz update loop:

```rust
let imu = reading.data.0.as_ref().map(|imu| ImuSample {
    acc: imu.acc,
    gyro: imu.gyro * (PI / 180.0), // deg/s -> rad/s at the edge
});
let (pyro, state, mpc_states, kf_state, ab_log) = estimators.lock(|s| {
    let mut est = s.borrow_mut();
    let pyro = est.update(reading.timestamp_us, imu.as_ref(), baro_altitude_asl);
    let dep = est.deployment_estimator();
    let kf_state = (dep.altitude_asl(), dep.vertical_velocity());
    let ab = est.airbrakes_estimator();
    let ab_log = /* caller assembles (alt, vv, tilt_deg, AB_* flags) from ab */;
    (pyro, est.state(), est.airbrakes_mpc_states(), kf_state, ab_log)
});
kf_state_sender.send(kf_state);
airbrakes_state_sender.send(ab_log);
if let Some(pyro) = pyro { let _ = fire_signal.try_send(pyro); }

// The gate, was 6 clauses + a HIL cfg override:
if !airbrakes_started && mpc_states.is_some() {
    start_airbrakes_signal.signal(());
    airbrakes_started = true;
}
```

The MPC future — run/stop condition and state source are the same Option
(this also fixes today's quirk where the ab estimator's early apogee
latch silently switches the MPC to the laggy slow-filter fallback for a
few seconds):

```rust
start_airbrakes_signal.wait().await;
let pad = estimators.lock(|s| s.borrow().deployment_estimator().launch_pad_altitude_asl());
let airbrakes_mpc = AirBrakesMPC::new(ROCKET_PARAMETERS.clone(), pad + target_agl);
loop {
    let Some(s) = estimators.lock(|s| s.borrow().airbrakes_mpc_states()) else { break };
    let ext = airbrakes_mpc.update(s.altitude_asl, s.velocity);
    /* publish ext to CAN + watch + telemetry */
    ticker.next().await;
}
/* retract to 0 */
```

Telemetry/CAN futures use component accessors: `est.state()` (honest
variants per items 1-2), `deployment_estimator().is_coasting()`, and the
CAN rocket-state message keeps reading `airbrakes_estimator().velocity()`
directly (it wants the 2D state whenever the filter is alive, gate or
not).

Deleted from armed_mode: the `in_mach_lockout` extraction, the inline
0.85*sos check, the `Some(kf_vertical_velocity)` plumbing, the MPC
`unwrap_or` fallback, and both `#[cfg(feature = "hil-replay")]`
overrides. Ordering: item 9 must land WITH or BEFORE this — without the
synthesized IMU, `airbrakes_mpc_states()` is never `Some` on the HIL
bench and the MPC path would never start.

## 9. HIL: synthesize the IMU too — delete the application-layer overrides

HIL's rule is "substitute at the sensor boundary only" (baro value swapped
for the scripted trajectory in imu_baro_task; everything downstream is the
production path). The IMU breaks that rule today: the bench IMU never
feels ignition, so the airbrakes estimator stays OnPad forever, and the
workarounds live at the application layer instead — the
`ab_baro_trusted = true` cfg override and the MPC slow-filter fallback in
armed_mode.

Fix at the boundary: synthesize accel/gyro from the same script clock as
the baro. The trajectory is analytic, so specific force is exact per
phase: pad and terminal descent `[0,0,+g]`, burn `[0,0,BURN_ACCEL+g]`,
ballistic coast `[0,0,0]`. Gyro zero, plus an optional injected constant
bias so the pad calibration is exercised for real. Keep the REAL IMU read
for pacing and replace only the values — genuine DRDY timestamps keep the
measured-dt path honest. Reuse the deterministic hash-noise with the
sigmas measured from the Void Lake pad data (accel ~0.05-0.10 m/s^2,
gyro ~0.07-0.16 deg/s).

Result: the whole airbrakes estimator flies on the bench (pad calibration
-> ignition -> Stage1 -> dead reckoning -> birth -> Tracking -> apogee),
`airbrakes_mpc_states()` is genuine, and BOTH armed_mode HIL overrides
are deleted — armed_mode becomes identical to flight, which is what
HIL.md already claims. This supersedes item 8's "HIL fallback stays in
firmware cfg" note: with this, there is no fallback at all.

The 15 s scripted pad hold gives ~7 bias windows, satisfying item 5's
calibration-complete precondition (bump PAD_DURATION_S if item 5 ends up
requiring more).

Optional phase 2: a supersonic script variant (Mach ~1.5 profile,
synthetic port error c*v^2 plus shock garbage on the baro, lockout
configured) to exercise the 2-of-3 vote and born-subsonic birth on the
bench, not just in desktop replays.
