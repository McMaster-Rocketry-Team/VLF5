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

| Feature | Deployment profile | Expected pyro |
|---------|-------------------|---------------|
| `hil-dual` | Drogue at apogee, main at ~457 m AGL | `PyroDrogue` near apogee, then `PyroMain` near main altitude |
| `hil-single` | Both pyros at apogee | `PyroDrogue` then `PyroMain` back-to-back at apogee |

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

You need the **GCM** plugged into the laptop and antennas on both boards, same as a real flight.

From `VLF5/firmware` (probe attached):

```bash
# Single-deploy plot
cargo run --release --bin main --features hil-single

# Dual-deploy plot
cargo run --release --bin main --features hil-dual
```

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
the 416 Hz loop and stamps the samples, so the estimators' measured-dt integration sees
genuine timing, jitter and all. Only the *values* are replaced. The trajectory is
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
