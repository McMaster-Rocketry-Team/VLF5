# VLF5 In-Process HIL Plots

Desk-test the full avionics path on hardware **without** real IMU/baro SPI, LoRa, GPS UART, or pyro GPIO. Fake sensors and VLP uplinks drive the real armed-mode estimator, airbrakes MPC, pyro queue, telemetry, and SD logger. Monitor with **defmt RTT**; dump the flight log afterward with **rocket-cli**.

**Never flash a HIL build with live e-matches connected.**

## Features

| Feature | Deployment profile | Expected pyro |
|---------|-------------------|---------------|
| `hil-dual` | Drogue at apogee, main at ~457 m AGL | `PyroDrogue` near apogee, then `PyroMain` near main altitude |
| `hil-single` | Both pyros at apogee | `PyroDrogue` then `PyroMain` back-to-back (~3 s apart with HIL pyro pulse) |

Enable **exactly one** of `hil-dual` / `hil-single` (both pull in base `hil-replay`). Bare `hil-replay` alone does not compile.

Flight builds (no HIL feature) are unchanged and still use `FlightProfile::Dual` by default.

## Run

From `VLF5/firmware` (probe attached):

```bash
# Dual-deploy plot
cargo run --release --bin main --features hil-dual

# Single-deploy plot
cargo run --release --bin main --features hil-single
```

Or flash explicitly:

```bash
cargo build --release --bin main --features hil-dual --target thumbv7em-none-eabihf
probe-rs run --chip STM32H743VIHx target/thumbv7em-none-eabihf/release/main
```

If `--connect-under-reset` fails to attach, omit it (as above).

Optional: clear the SD log before a clean run (USB-C to the board):

```bash
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- clear-flight-log
```

## What is simulated

### Sensor profile (`sample_at(t)`)

Vertical 1D rocket (always upright). Baro pressure is ISA from altitude; IMU is a simple vertical accel stub for logging/CAN.

| Phase | Boot time | Physics |
|-------|-----------|---------|
| Pad | 0–8 s | 200 m ASL |
| Burn | 8–11 s | +80 m/s² for 3 s |
| Coast / free-fall | after burnout | −9.81 m/s², descent capped at −25 m/s |
| Ground | after touchdown | Back at pad altitude |

Rough outcome: apogee ~3.3 km AGL; full dual flight ~3–4 minutes wall time.

### Ground script (injected VLP)

| Time | Uplink |
|------|--------|
| t ≈ 2 s | `SetTargetApogee(4000 m)` (also persisted to SD config block) |
| t ≈ 3 s | `ChangeMode(Armed)` |

### Replaced vs real tasks

| Real task | HIL replacement |
|-----------|-----------------|
| `imu_baro_task` | `sensor_replay_task` (416 Hz when Armed) |
| `vlp_avionics_daemon_task` | `hil_vlp_bridge_task` + `hil_script_task` |
| `pyro_task` | `hil_pyro_monitor` (logs fires, 3 s continuity pulse, **no GPIO**) |
| `gps_task` | `hil_gps_stub` |
| Boot mode | `LowPower` (skips SelfTest hardware checks) |

CAN / USB / SD / mag / ADC still run on real hardware when present.

## RTT checklist

Watch for panics (`[ERROR]`, `panicked`, `Firmware exited`) while the plot runs.

1. Boot: `HIL-DUAL` or `HIL-SINGLE` warning; enter LowPower
2. ~2 s: `SetTargetApogee` + SD save of target apogee
3. ~3 s: `ChangeMode Armed` → `enter armed mode`
4. ~8.6 s: ascent / `HIL: starting airbrakes` / `Armed -> PoweredAscent`
5. Telemetry `alt_agl` climbs (not stuck near 0); `air_speed` tracks vertical speed
6. Pyro per plot table (`HIL: FIRE PyroDrogue` / `PyroMain`)
7. `terminal state Landed` → `enter landed mode` after ~30 s settle

### Expected timings (current scripted profile)

| Event | Dual | Single |
|-------|------|--------|
| Airbrakes start | ~8.6 s, ~215 m ASL, vy ~38 m/s | same |
| Drogue | ~36.3 s, ~3295 m AGL | ~36.3 s, ~3295 m AGL |
| Main | ~150 s, ~457 m AGL | ~39.3 s (back-to-back at apogee) |
| Landed | ~173 s still + 30 s → mode switch | similar |

Airbrakes: HIL exercises the real start gate and MPC; it does **not** assert a specific extension schedule. Near apogee you may see a small commanded extension (e.g. ~0.1) then retract on deploy.

## SD dump with rocket-cli

Keep USB connected (VID `c0de` / PID `cafe`). After the flight (or while Landed):

```bash
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- list-flight-log
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- download-flight-log hil_flight_log.csv
```

Expect a large tagged log (IMU + SLOW) spanning LowPower → Armed → ascent → deploy → Main/Landed. Optional: `clear-flight-log` before the next plot.

### SD write latency warnings

`SD write took ~55ms (queue depth 0)` means a block write exceeded the 40 ms warn threshold. With queue depth 0 and no “queue full” / offline messages, logging is still healthy; this card is just slower than the warn budget under 416 Hz logging.

## Source layout

| Path | Role |
|------|------|
| `src/hil/mod.rs` | Feature gates + short overview |
| `src/hil/sensor_replay.rs` | `sample_at(t)` + 416 Hz publisher |
| `src/hil/vlp_script.rs` | Timed uplinks + downlink logger |
| `src/hil/pyro_monitor.rs` | Safe pyro fire logging |
| `src/hil/gps_stub.rs` | Fake GPS so mode loops do not stall |
| `Cargo.toml` | `hil-replay` / `hil-dual` / `hil-single` |

Related monorepo pieces: baro-only `RocketStateEstimator`, vertical `AirBrakesMPC`, `VLPAvionics::inject_uplink` / `wait_downlink`, SD config block for target apogee, telemetry altitude packing fix.
