# VLF5 In-Process HIL Plots

Desk-test the **entire** avionics flow on real hardware by faking **only the sensor
values a static bench can't produce**: the barometer reading and the IMU's accel/gyro
values, both synthesized at the sensor boundary from one scripted single-deploy vertical
trajectory. GPS, magnetometer, pyro GPIO, CAN, SD, USB, and the LoRa radio all run for
real, and the IMU chip itself is still read on its real data-ready interrupt — genuine
pacing and timestamps, only the values are swapped. The board boots into **SelfTest**
exactly like a flight build, and you fly the plot from **rocket-cli** over the real radio
just like a real flight ([OPERATOR.md](OPERATOR.md)). Everything downstream of the two
sensor seams exercises the production path — armed mode is identical to flight, with no
HIL overrides anywhere above the seams. Monitor with **defmt RTT** and/or the rocket-cli
telemetry stream; dump the flight log afterward with rocket-cli.

> **The real pyro task drives real pyro GPIO in HIL.** At apogee the drogue/main FETs are
> energized for real. **Never flash a HIL build with live e-matches connected.**

## Features

| Feature | Trajectory | Config flown | Arming | Duration |
|---------|-----------|--------------|--------|----------|
| `hil-dual` | **Osiris replay** — the real OpenRocket run, Mach 1.91, 9.5 km (`hil/osiris.rs`) | the real `FLIGHT_CONFIG` | **boots Armed** (no ground station) | ~8.5 min |
| `hil-single` | analytic script, ~2.9 km (`hil/baro_sim.rs`) | HIL single-deploy stand-in | `arm` over the radio | ~1.5 min |

`hil-dual` is the only profile that exercises the **Mach lockout** — nothing
else on the bench goes supersonic — and the only one that flies the numbers
the rocket will actually fly. It replays the same CSV the host-side estimator
tests replay (`air-brakes-controller-core/src/tests/osiris_sim.rs`), so a
disagreement between bench and test suite is a real signal rather than two
different flights.

> `hil-dual` implies **`boot-armed`**: the board enters Armed straight out of
> reset with no operator, and the pyro task energizes the drogue and main FETs
> on its own about a minute later, every time it is reset. `boot-armed` will
> not compile into a flight build (see the `compile_error!` in `main.rs`), but
> on the bench it removes the last human check. **No e-matches. Ever.**

Enable **exactly one** of `hil-dual` / `hil-single` (both pull in base `hil-replay`). Bare
`hil-replay` alone does not compile.

Flight builds (no HIL feature) are unchanged. The **only** difference in a HIL build is at
the sensor boundary in `imu_baro_task`: instead of reading the MS5607 over SPI, the baro
pressure is synthesized from the scripted trajectory, and the LSM6DSM's just-read
accel/gyro values are swapped for the scripted specific force and angular rate (same
script clock, so the two sensors tell one consistent story). The boot mode, mode machine,
estimators, airbrakes MPC, pyro queue, GPS, mag, CAN, SD logger, telemetry, and radio
config are all identical to flight.

## Run

`hil-single` needs the **GCM** plugged in and antennas on both boards, to arm
over the radio. `hil-dual` needs neither — it arms itself.

From `VLF5/firmware` (probe attached):

```bash
# Osiris replay, self-arming — just watch RTT for ~8.5 minutes
cargo run --release --bin main --features hil-dual -- \
  --probe 0483:374b:066BFF525086874967123919

# Short analytic plot, armed over the radio
cargo run --release --bin main --features hil-single
```

Regenerate the Osiris table (after a new `.ork`, or a different sim) with:

```bash
scripts/gen_osiris_hil_table.py > src/hil/osiris_table.rs
```

### Verified `hil-dual` run, 2026-08-17

Wall clock; the 20 s pad hold means trajectory time is wall − 20 s.

> **This run predates the accelerometer ignition trigger** added to the
> deployment estimator (`FlightProfile::ignition_detection_acc_threshold`,
> 8 g on this profile). Two rows will move on the next bench run: ignition
> detection from 21.12 s to ~20.15 s, and the Mach lockout exit with it,
> from 47.12 s to ~46.15 s — the lockout duration itself is unchanged, it is
> the anchor that moved. Everything from apogee onward is driven by altitude
> and by the apogee call rather than by ignition, so it should be
> substantially unchanged; that has not been measured yet.

| wall | event | vs truth |
|---|---|---|
| 0.01 s | `enter armed mode` (boot-armed) | — |
| 6.06 s | pad calibration complete, 3 windows | 14 s before ignition |
| 21.12 s | ignition detected, pad altitude −63.0 m | matches the ISA pressure altitude of the Osiris pad |
| 25.61 s | burnout latched | axial sign crosses at t=5.21 s; latch is a tail-off latch, see `osiris_sim.rs` |
| 38.94 s | vertical filter born, **not forced**, alt 6967 m, vv 229.0 m/s | trajectory t=18.94 s, matching the host suite's 18.87 s |
| 47.12 s | Mach lockout over | **26.0015 s** wall for a 26.0000 s config |
| 59.64 s | airbrakes estimator retired (descending) | true apogee t=39.58 s → **+0.06 s** |
| 62.68 s | descent detected, peak 9347.4 m AGL | truth 9329.6 m pressure-AGL → **+17.8 m** |
| 63.68 s | **PyroDrogue** | apogee call + **1.0026 s** for a 1.000 s config |
| 386.39 s | **PyroMain** | truth 457.2 m AGL crossing at t=366.4 s → **−0.01 s** |
| 497.15 s | `landed` | 5 s stillness persistence after touchdown |

No panics, hard faults or errors; the session ends on the harness timeout.

Against the previous run (same trajectory, before the deployment timers took
their time from the sample timestamp):

| | was | now | config |
|---|---|---|---|
| Mach lockout | 25.3320 s | **26.0015 s** | 26.000 s |
| drogue delay | 0.9770 s | **1.0025 s** | 1.000 s |
| `PyroMain` | 386.3891 s | 386.3895 s | — (an altitude crossing, not a timer) |
| `landed` | 497.0170 s | 497.1461 s | — |

`PyroMain` is the control: it is triggered by the 457.2 m AGL crossing rather
than by a timer, and it does not move. `landed` moves +129 ms, which is
exactly the 5 s stillness persistence that used to be 2080 samples — 4.872 s
at the part's real 427.02 Hz.

**The sample clock is not 416 Hz.** `imu_bench` measures this board's LSM6DSM
at **427.02 Hz** (2341.8 µs, ±1.5 µs), 2.65 % above the nominal
`SAMPLES_PER_S = 416`.

**Resolved for the timers.** The deployment estimator used to count its Mach
lockout, pyro delays and apogee/landing persistence in *samples*, so every one
of them expired 2.65 % early in wall time — measured on the bench, the 26 s
Mach lockout ran **25.332 s** and the 1 s drogue delay ran **0.977 s**. Those
timers now read the sample timestamp instead, and the same bench run measures
**26.0015 s** and **1.0026 s**: about one sample long each, which is a timer
that fires on the first sample *after* it expires rather than before. The
airbrakes estimator was already fully measured-dt; the last two pieces of it
that were not (the pre-ignition rewind buffer, sized in samples, and the
ignition low pass, a biquad designed at the nominal rate) are now spans of
measured time as well.

**Still present for the KF, on purpose.** `BaroAltitudeKF` integrates a fixed
`DT = 1/416` per sample while samples arrive every 1/427 s, which biases its
vertical velocity low by 2.6 %. That is the deliberate half of the split: the
filter whose output fires the pyros cannot have its bandwidth moved by a
clock, so it is fed one fixed step per sample and nothing else. The visible
cost is that anything gated on the filter *settling* moves with the sample
rate — landing detection spreads 2.3 s across 380–480 Hz (`cargo test -p
air-brakes-controller-core timers_are_independent_of_the_sample_rate`), all of
it after touchdown and after both pyros have fired.

**Resolved: the air-density math differed between host and board.**
`approximate_air_density` was written `x.powf(4.256)`. The method form
resolves to the inherent `f32::powf` under std and to whatever `F32Ext` trait
is in scope under `no_std`, so the host suite and the firmware computed
different numbers from the same source line — with `micromath` in scope the
board's density ran up to 39% low at altitude, inflating the drag check's
inverted airspeed by 28%.

That first showed up here as a filter born at trajectory t=22.1 s against the
host suite's 18.9 s. Everything else was ruled out against the SD log first:
the logged accelerometer matched the baked table to <1%, logged tilt matched
the host model, and integrating the logged sensors reproduced the true
altitude to 40 m. The fix is `libm::powf` called by name — no
inherent-vs-trait resolution, so the tests and the rocket run the same
arithmetic — applied alongside the other three divergent call sites (`sqrt`,
`tan`, `powi`), after which `micromath` left the dependency list entirely.

Re-flown 2026-08-16: born at trajectory **t=18.89 s**, matching the host
suite, recovering **3.17 s** of airbrakes control window. Deployment was
unchanged to the millisecond (drogue 63.639 s, main 386.389 s, landed
497.017 s), which is the expected result — the deployment estimator is
baro-only and never reads air density.

If `--connect-under-reset` fails to attach on a freshly-replugged probe, retry (or omit it).

Then fly it from rocket-cli over the radio, exactly like [OPERATOR.md](OPERATOR.md) — either the
TUI (`rocket-cli ground-station`) or the headless stream:

```bash
rocket-cli control --frequency 920000000 --vlp-key <base64 vlp.key>
# then, one per line: (mode self-test) / target-apogee 2500 / mode low-power / arm
```

The board boots straight into **SelfTest** (self-test result telemetry every 2 s). Send
`arm` — the script clock latches on entry to Armed and plays the trajectory through both
synthesized sensors (pad → burn → coast → apogee → deploy → descent → landed). Pyro fires
drive **real GPIO** (safe only with no e-matches connected).

Optional: clear the SD log before a clean run (USB-C to the board):

```bash
rocket-cli clear-flight-log
```

## What is simulated

Vertical 1D rocket (always upright). One scripted trajectory, one **Arm-relative flight
clock** shared by both synthesized sensors (t below is time since `arm`); pre-arm modes
read the stationary pad, and leaving Armed resets the clock so a re-arm replays cleanly.

| Phase | Time since Arm | Physics |
|-------|-----------|---------|
| Pad | 0–15 s | 200 m ASL |
| Burn | 15–18 s | +80 m/s² for 3 s |
| Coast / free-fall | after burnout | −9.81 m/s², descent capped at −25 m/s |
| Ground | after touchdown | Back at pad altitude |

Rough outcome: apogee ~3.3 km AGL; full flight ~3–4 minutes wall time.

### Barometer (`baro_sim::generate_baro`)

Baro pressure is ISA from the scripted altitude plus measured per-sample sensor noise
(sigma ~0.36 m, measured on this VLF5's MS5607). The real baro is never touched in HIL,
so a bench baro fault can't abort the simulated flight.

### IMU values (`imu_sim::generate_imu`)

The LSM6DSM is still **read for real** on its real data-ready interrupt — that read paces
the sample loop (nominally 416 Hz, measured 427.02 Hz on this part) and stamps the
samples, so everything downstream that reads a timestamp sees genuine timing, jitter and
all: both estimators' integrations, and now the deployment estimator's timers too. Only
the *values* are replaced. The trajectory is
analytic, so the specific force (what an accelerometer measures) is exact per phase, on
device +Z — the scripted "up" (the estimator self-calibrates its mounting from pad
gravity + thrust direction, so the axis choice is arbitrary; what matters is that pad
gravity and burn thrust agree on one axis, i.e. a vertical rail):

| Phase | Time since Arm | Specific force (m/s²) | Gyro (deg/s) |
|-------|-----------|---------|---------|
| Pad | 0–15 s | [0, 0, +9.81] | bias + noise |
| Burn | 15–18 s | [0, 0, 80 + 9.81] | bias + noise |
| Ballistic coast | 18–45 s | [0, 0, 0] | bias + noise |
| Terminal descent / ground | after ~45 s | [0, 0, +9.81] | bias + noise |

Units match the real driver output at the seam: accel in m/s², gyro in deg/s. Noise is
the same deterministic hash-noise the baro uses, with per-axis sigmas measured from the
Void Lake pad data (accel 0.07 m/s², gyro 0.1 deg/s), plus a **constant injected gyro
bias of 0.1 deg/s about X** so the pad bias calibration is exercised for real — a broken
calibration shows up as phantom tilt, not as nothing.

With the IMU synthesized, the **whole airbrakes estimator** flies on the bench: pad
gyro-bias calibration (the 15 s pad hold completes ~7 of its 2 s bias windows before
ignition) → ignition detection → Stage1 thrust-vector alignment → dead reckoning →
vertical-filter birth → Tracking → apogee latch. The MPC runs on that estimator's own
state, and `armed_mode` carries no HIL overrides — it is identical to flight.

Future work: a supersonic script variant (Mach ~1.5 profile, synthetic static-port error
`c·v²` plus shock garbage on the baro, Mach lockout configured) would exercise the
lockout-exit drag check and the born-subsonic birth on the bench, not just in desktop
replays.

### Everything else is real

| Subsystem | HIL behavior |
|-----------|--------------|
| IMU chip (`imu_baro_task`) | **Real** LSM6DSM read, real data-ready interrupt clocks the loop — only the just-read values are swapped |
| GPS (`gps_task`) | **Real** (no fix indoors is expected; the module still reports) |
| Pyro (`pyro_task`) | **Real GPIO** — drogue/main FETs fire for real (no e-matches!) |
| Mag / CAN / SD / USB | **Real** |
| LoRa radio + VLP | **Real** — the operator drives every uplink from rocket-cli |
| Boot mode | **SelfTest** (same as flight) |

The sensor boundary is the only seam — two functions in `sensor_tasks.rs`, sharing one
`HilSimState` script clock. In flight builds `read_baro_or_sim` reads the MS5607 and
`imu_values_or_sim` passes the real IMU reading through untouched; in HIL the former
returns the scripted baro and the latter keeps the real reading's DRDY timestamp while
swapping in the scripted accel/gyro values.

## Flight-log gating

Flight-data logging runs **only from Arm through Landed** (`AvionicsMode::should_log`). SelfTest,
LowPower, and Demo do not write to the SD card, so preflight doesn't fill the log and downloads
stay small. The mode stays `Armed` for the entire ascent/coast/deploy/descent, then auto-switches
to `Landed` — so the log spans the whole flight and the landed GPS beacon.

## RTT checklist

Watch for panics (`[ERROR]`, `panicked`, `Firmware exited`) while the plot runs.

1. Boot: `HIL-DUAL` / `HIL-SINGLE` warning; `enter self test mode`; `IMU initialized`;
   `Barometer initialized`; `LoRa initialized`.
2. Operator preflight over the radio: `target-apogee` → `SetTargetApogee … persisting to SD`;
   optional `mode low-power` → `enter low power mode`.
3. Operator `arm` → `enter armed mode`; the script clock starts.
4. ~15 s after arm, ignition: the airbrakes estimator's own path lights up —
   `ignition detected, rewinding pad buffer`, `gyro bias: screened over 7 windows`,
   `launch angle: ~0 deg`, then `vertical filter born` — followed by the slow filter's
   `Armed -> Ascent`.
5. Telemetry `deployment_kf_altitude_agl` climbs (not stuck near 0); `airbrakes_kf_vertical_velocity`
   tracks vertical speed.
   Airbrakes start ~20 s (the slow filter's burn timer declares coasting; the airbrakes
   filter has been alive since ~16 s).
6. Pyro at apogee (`apogee latched at …`, then drogue/main) — the real `pyro_task` drives
   the FETs (SD log `pyro_*_fire` edges confirm).
7. `terminal state Landed` → `enter landed mode` after ~30 s settle.

### Expected timings (time since Arm, analytic approximations)

| Event | Dual | Single |
|-------|------|--------|
| AB vertical filter born | ~16 s (ignition + Stage1 + baro ring) | same |
| Airbrakes start | ~20 s (burn-timer coasting), vy ~220 m/s | same |
| Drogue | ~43 s, ~3.3 km AGL | ~43 s, ~3.3 km AGL |
| Main | ~157 s, ~457 m AGL | back-to-back at apogee |
| Landed | ~176 s still + 30 s → mode switch | similar |

Airbrakes: HIL exercises the real start gate and the MPC on the airbrakes estimator's own
KF state (no fallback — the flight path exactly); the scripted trajectory is open-loop, so
the brakes are commanded but do not change the baro/IMU profile. The near-apogee
validation deploy (forced 100% below Mach 0.1 when the MPC never went full) does *not*
fire on the bench: the scripted flight overshoots the 3000 m target, so the MPC commands
full extension early. To exercise it, arm with a target apogee well above the scripted
~3.3 km — then expect `forcing 100% for validation` in the last few seconds of ascent.

Either way, expect one `retiring airbrakes estimator (descending: …, below horizon: …,
deployment apogee: …)` line at apogee, and the `airbrakes_*` telemetry/SD columns to go
0/NaN from there through the whole descent. That line firing more than once, or the ab
columns coming back after it, is a bug.

## SD dump with rocket-cli

Keep USB-C connected (VID `c0de` / PID `cafe`). After the flight (or while Landed):

```bash
rocket-cli list-flight-log
rocket-cli download-flight-log hil_flight_log.csv
```

Expect a tagged log (IMU + SLOW) spanning Armed → ascent → deploy → Main/Landed (preflight is
not logged). The log is append-only across flights — `clear-flight-log` between runs to keep
downloads small.

### SD write latency warnings

`SD write took ~55ms (queue depth 0)` means a block write exceeded the 40 ms warn threshold. With
queue depth 0 and no "queue full" / offline messages, logging is still healthy; this card is just
slower than the warn budget under high-rate logging.

## Source layout

| Path | Role |
|------|------|
| `src/hil/mod.rs` | Feature gates + overview + `HilSimState` (shared Arm-relative script clock, per-sensor sample counters) |
| `src/hil/baro_sim.rs` | `trajectory_altitude_asl` (the shared phase constants) + `generate_baro` |
| `src/hil/imu_sim.rs` | Per-phase specific force + `generate_imu` (accel m/s², gyro deg/s, injected gyro bias) |
| `src/hil/noise.rs` | Deterministic hash-noise shared by both sims |
| `src/tasks/sensor_tasks.rs` | The two seams inside `imu_baro_task`: `read_baro_or_sim`, `imu_values_or_sim` |
| `Cargo.toml` | `hil-replay` / `hil-dual` / `hil-single` |

Related monorepo pieces: baro-only `RocketStateEstimator` (deployment), the
`AirbrakesEstimator` (runs its full pad-to-apogee path on the bench), vertical
`AirBrakesMPC`, the real `VLPAvionics` daemon over LoRa, SD config block for target apogee.
