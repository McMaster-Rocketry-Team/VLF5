# VLF5 In-Process HIL Plots

Desk-test the full avionics path on hardware **without** real IMU/baro SPI, GPS UART, or pyro GPIO. Fake sensors drive the real armed-mode estimator, airbrakes MPC, pyro queue, telemetry, and SD logger. **VLP runs over the real LoRa radio + GCM** — you fly the plot from **rocket-cli** exactly like a real flight ([OPERATOR.md](OPERATOR.md)). Monitor with **defmt RTT** and/or the rocket-cli telemetry stream; dump the flight log afterward with rocket-cli.

**Never flash a HIL build with live e-matches connected.**

## Features

| Feature | Deployment profile | Expected pyro |
|---------|-------------------|---------------|
| `hil-dual` | Drogue at apogee, main at ~457 m AGL | `PyroDrogue` near apogee, then `PyroMain` near main altitude |
| `hil-single` | Both pyros at apogee | `PyroDrogue` then `PyroMain` back-to-back (~3 s apart with HIL pyro pulse) |

Enable **exactly one** of `hil-dual` / `hil-single` (both pull in base `hil-replay`). Bare `hil-replay` alone does not compile.

Flight builds (no HIL feature) are unchanged and still use `FlightProfile::Dual` by default. HIL and flight builds use the **same** real VLP daemon, mode machine, estimator, airbrakes, pyro queue, and radio config — only the sensors, GPS, and pyro GPIO are faked, and HIL boots into `LowPower` (so you arm over the radio).

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
# then, one per line: mode self-test / target-apogee 2500 / mode low-power / arm
```

The HIL sensor replay latches its flight clock when you send **arm**, then plays the trajectory
(pad → burn → coast → apogee → deploy → descent → landed). Pyro fires drive **no GPIO**.

Optional: clear the SD log before a clean run (USB-C to the board):

```bash
rocket-cli clear-flight-log
```

## What is simulated

### Sensor profile (`sample_at(t)`)

Vertical 1D rocket (always upright). Baro pressure is ISA from altitude; IMU is a simple vertical accel stub for logging/CAN. The flight clock is **relative to entering Armed** (t below is time since `arm`).

| Phase | Time since Arm | Physics |
|-------|-----------|---------|
| Pad | 0–15 s | 200 m ASL |
| Burn | 15–18 s | +80 m/s² for 3 s |
| Coast / free-fall | after burnout | −9.81 m/s², descent capped at −25 m/s |
| Ground | after touchdown | Back at pad altitude |

Rough outcome: apogee ~3.3 km AGL; full flight ~3–4 minutes wall time.

### Replaced vs real tasks

| Real task | HIL replacement |
|-----------|-----------------|
| `imu_baro_task` | `sensor_replay_task` (416 Hz when Armed) |
| `pyro_task` | `hil_pyro_monitor` (logs fires, 3 s continuity pulse, **no GPIO**) |
| `gps_task` | `hil_gps_stub` |
| Boot mode | `LowPower` (skips SelfTest hardware checks; arm over the radio) |

`vlp_avionics_daemon_task` (the real LoRa radio + VLP), CAN, USB, SD, mag, and ADC all run on
real hardware — HIL does **not** fake the radio. The operator drives every uplink from rocket-cli.

## RTT checklist

Watch for panics (`[ERROR]`, `panicked`, `Firmware exited`) while the plot runs.

1. Boot: `HIL-DUAL` or `HIL-SINGLE` warning; `enter low power mode`; `LoRa initialized`.
2. Operator `mode self-test` → `enter self test mode`; `target-apogee` → `SetTargetApogee … persisting to SD`; `mode low-power` → `enter low power mode`.
3. Operator `arm` → `enter armed mode`; sensor replay goes to 416 Hz; flight clock starts.
4. ~15 s after arm: ascent / `HIL: starting airbrakes` / `Armed -> PoweredAscent`.
5. Telemetry `alt_agl` climbs (not stuck near 0); `air_speed` tracks vertical speed.
6. Pyro per plot table (`HIL: FIRE PyroDrogue` / `PyroMain`).
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

Expect a large tagged log (IMU + SLOW) spanning LowPower → Armed → ascent → deploy → Main/Landed.
The log is append-only across flights — `clear-flight-log` between runs to keep downloads small.

### SD write latency warnings

`SD write took ~55ms (queue depth 0)` means a block write exceeded the 40 ms warn threshold. With
queue depth 0 and no "queue full" / offline messages, logging is still healthy; this card is just
slower than the warn budget under 416 Hz logging.

## Source layout

| Path | Role |
|------|------|
| `src/hil/mod.rs` | Feature gates + short overview |
| `src/hil/sensor_replay.rs` | `sample_at(t)` + 416 Hz publisher, Arm-relative flight clock |
| `src/hil/pyro_monitor.rs` | Safe pyro fire logging (no GPIO) |
| `src/hil/gps_stub.rs` | Fake GPS so mode loops do not stall |
| `Cargo.toml` | `hil-replay` / `hil-dual` / `hil-single` |

Related monorepo pieces: baro-only `RocketStateEstimator`, vertical `AirBrakesMPC`, the real
`VLPAvionics` daemon over LoRa, SD config block for target apogee.
