# VLF5 In-Process HIL Plots

Desk-test the **entire** avionics flow on real hardware by faking **only the barometer
reading**. IMU, GPS, magnetometer, pyro GPIO, CAN, SD, USB, and the LoRa radio all run
for real — the board boots into **SelfTest** exactly like a flight build, and you fly the
plot from **rocket-cli** over the real radio just like a real flight ([OPERATOR.md](OPERATOR.md)).
Because the barometer is the one sensor a static bench can't exercise (it can't feel
altitude), its value is replaced by a scripted single-deploy vertical trajectory; every
other decision, task, and GPIO exercises the production path. Monitor with **defmt RTT**
and/or the rocket-cli telemetry stream; dump the flight log afterward with rocket-cli.

> **The real pyro task drives real pyro GPIO in HIL.** At apogee the drogue/main FETs are
> energized for real. **Never flash a HIL build with live e-matches connected.**

## Features

| Feature | Deployment profile | Expected pyro |
|---------|-------------------|---------------|
| `hil-dual` | Drogue at apogee, main at ~457 m AGL | `PyroDrogue` near apogee, then `PyroMain` near main altitude |
| `hil-single` | Both pyros at apogee | `PyroDrogue` then `PyroMain` back-to-back at apogee |

Enable **exactly one** of `hil-dual` / `hil-single` (both pull in base `hil-replay`). Bare
`hil-replay` alone does not compile.

Flight builds (no HIL feature) are unchanged. The **only** difference in a HIL build is the
barometer: instead of reading the MS5607 over SPI, `imu_baro_task` synthesizes the pressure
from the scripted trajectory. The boot mode, mode machine, estimator, airbrakes MPC, pyro
queue, GPS, mag, CAN, SD logger, telemetry, and radio config are all identical to flight.

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
`arm` — the baro flight clock latches on entry to Armed and plays the trajectory (pad → burn
→ coast → apogee → deploy → descent → landed). Pyro fires drive **real GPIO** (safe only with
no e-matches connected).

Optional: clear the SD log before a clean run (USB-C to the board):

```bash
rocket-cli clear-flight-log
```

## What is simulated

### Barometer only (`baro_sim::generate_baro`)

Vertical 1D rocket (always upright). Baro pressure is ISA from the scripted altitude plus
measured per-sample sensor noise (sigma ~0.36 m). The flight clock is **relative to entering
Armed** (t below is time since `arm`); pre-arm modes read the noisy pad altitude.

| Phase | Time since Arm | Physics |
|-------|-----------|---------|
| Pad | 0–15 s | 200 m ASL |
| Burn | 15–18 s | +80 m/s² for 3 s |
| Coast / free-fall | after burnout | −9.81 m/s², descent capped at −25 m/s |
| Ground | after touchdown | Back at pad altitude |

Rough outcome: apogee ~3.3 km AGL; full flight ~3–4 minutes wall time.

### Everything else is real

| Subsystem | HIL behavior |
|-----------|--------------|
| IMU (`imu_baro_task`) | **Real** LSM6DSM read, real data-ready interrupt clocks the loop |
| GPS (`gps_task`) | **Real** (no fix indoors is expected; the module still reports) |
| Pyro (`pyro_task`) | **Real GPIO** — drogue/main FETs fire for real (no e-matches!) |
| Mag / CAN / SD / USB | **Real** |
| LoRa radio + VLP | **Real** — the operator drives every uplink from rocket-cli |
| Boot mode | **SelfTest** (same as flight) |

The barometer is the single seam: in flight `read_baro_or_sim` reads the MS5607; in HIL it
returns `HilBaroState::next(mode)`. The real baro is never touched in HIL, so a bench baro
fault can't abort the simulated flight.

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
3. Operator `arm` → `enter armed mode`; baro flight clock starts.
4. ~15 s after arm: ascent / `HIL: starting airbrakes` / `Armed -> PoweredAscent`.
5. Telemetry `alt_agl` climbs (not stuck near 0); `air_speed` tracks vertical speed.
6. Pyro at apogee (`HIL: estimator requested pyro PyroDrogue` / `PyroMain`) — the real
   `pyro_task` then drives the FETs (SD log `pyro_*_fire` edges confirm).
7. `terminal state Landed` → `enter landed mode` after ~30 s settle.

### Expected timings (time since Arm)

| Event | Dual | Single |
|-------|------|--------|
| Airbrakes start | ~15.6 s, ~215 m ASL, vy ~38 m/s | same |
| Drogue | ~36 s, ~3295 m AGL | ~36 s, ~3295 m AGL |
| Main | ~150 s, ~457 m AGL | back-to-back at apogee |
| Landed | ~173 s still + 30 s → mode switch | similar |

Airbrakes: HIL exercises the real start gate and MPC; the replayed trajectory is open-loop, so
the brakes are commanded but do not change the baro profile.

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
| `src/hil/mod.rs` | Feature gates + overview |
| `src/hil/baro_sim.rs` | `trajectory_altitude_asl` + noise + `generate_baro` + `HilBaroState` (Arm-relative clock) |
| `src/tasks/sensor_tasks.rs` | `read_baro_or_sim` seam (real MS5607 vs simulated) inside `imu_baro_task` |
| `Cargo.toml` | `hil-replay` / `hil-dual` / `hil-single` |

Related monorepo pieces: baro-only `RocketStateEstimator`, vertical `AirBrakesMPC`, the real
`VLPAvionics` daemon over LoRa, SD config block for target apogee.
