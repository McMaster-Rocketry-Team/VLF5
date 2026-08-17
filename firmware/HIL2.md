# VLF5 HIL — bench facts

What HIL *is* lives in the code: [`src/hil/mod.rs`](src/hil/mod.rs) for the model and the
sensor seams, `Cargo.toml` for the features, [`src/hil/baro_sim.rs`](src/hil/baro_sim.rs) /
[`osiris.rs`](src/hil/osiris.rs) for the two trajectories and their measured noise. This file
holds only what the source cannot tell you: this bench's identity, what has actually been
measured on it, the traps that cost real debugging time, and what HIL still does not prove.

> **Pyro GPIO fires for real in HIL.** The real `pyro_task` energizes the drogue and main
> FETs at apogee. `hil-dual` additionally implies `boot-armed` — it enters Armed straight out
> of reset with no operator and fires ~64 s later, every reset. **No e-matches.**

## 1. Which probe is which board — re-probe, do not trust a cached answer

Two ST-Link V2.1s are attached in the usual setup and **neither firmware's runner pins one**,
so `probe-rs` errors out with >1 probe present unless you pass `--probe 0483:374b:<serial>`.

The serial↔board mapping has been silicon-verified twice, with **contradictory results** —
the probes were physically moved between sessions:

| when | finding |
|---|---|
| 2026-07-09 | `066EFF…3920` reports part `0x4500` (H743) → VLF5. `066BFF…3919` read UID `0x1FFF7A10` = `3d0052000251343039333936`, matching the **GCM's** USB serial (Main's starts `390052…`) → GCM. |
| 2026-08-16 | the probe presenting `066BFF…3919` read the H7 UID at `0x1FF1E800` successfully and **faulted** at the F4 address → that probe was on the **VLF5**; the GCM was not attached at all. |

So the only safe procedure is to identify before flashing:

```bash
probe-rs list                                  # serials currently attached
probe-rs info --probe 0483:374b:<serial>       # part 0x4500 / DPv2 = H743 = VLF5
                                               # Cortex-M4 / DPv1     = F405 = GCM or Main
```

An H743 answer is unambiguous (only the VLF5 is an H7). Between the two F405s, the GCM is the
one with a probe at all — Main has never had one attached.

## 2. Host access

| USB | board | interface | tty |
|---|---|---|---|
| `c0de:cafe` | VLF5 | vendor/WinUSB, bulk-IN `0x81` — **no CDC tty**; rocket-cli flight log only | — |
| `120a:0005` | Endgame GCM | CDC | `/dev/ttyACM2` |
| `120a:0006` | Endgame Main | CDC | `/dev/ttyACM1` |

`/dev/ttyACM0` and `/dev/ttyACM3` are the ST-Links' own virtual COM ports, **not** the defmt
channel — defmt RTT travels over SWD and is selected by probe serial.

`pegasis` is in `plugdev`, not `dialout`; access comes from three udev rules, all installed:
`69-probe-rs.rules` (`0483:374b`), `99-vlf5-flight-log.rules` (`c0de:cafe`), and
`99-endgame-usb.rules` (`120a:0005`/`0006`, incl. `ID_MM_DEVICE_IGNORE`). ModemManager is
active and will transiently grab a fresh `ttyACM` after re-enumeration — the
`ID_MM_DEVICE_IGNORE` lines are what stop it, so if a port is briefly held after replug,
confirm the installed copy of `99-endgame-usb.rules` actually contains them.

## 3. Running it on this bench

There is no `ground-station.toml` here, so every rocket-cli radio command needs the frequency
and key explicitly (SF12 / BW 250 kHz / CR 4/8 are hardcoded in firmware):

```bash
cargo run --release --bin main --features hil-single -- --probe 0483:374b:<vlf5-serial>

rocket-cli clear-flight-log        # append-only log; skip this and downloads grow ~1.1M records
rocket-cli control --frequency 920000000 --vlp-key "$(cat vlp.key)"
#   stdin: arm | mode <low-power|self-test|armed|landed|demo> | target-apogee <m>
#          | fire-pyro <main|drogue> | reset [all|void-lake|amp|icarus] | quit
rocket-cli download-flight-log out.csv
```

`hil_operator.py` drives that sequence unattended and prints PASS/FAIL. Note it **does not
assert on airbrakes** — `saw airbrakes cmd>0` is printed in the summary but never added to
`fails`.

Self-test takes ~35 s when the CAN nodes are offline (it waits out each node's response
timeout), so allow for that before deciding it hung. `--connect-under-reset` sometimes fails to
attach on a freshly replugged probe; retry, or omit it.

## 4. Measured properties of this hardware

**The sample clock is 427.02 Hz, not 416.** `imu_bench` measures this board's LSM6DSM at
2341.8 µs ±1.5 µs — 2.65 % above the nominal `SAMPLES_PER_S = 416`. Every timer that counted
*samples* therefore expired 2.65 % early in wall time; those now read the sample timestamp
instead. `BaroAltitudeKF` still integrates a fixed `DT = 1/416` on purpose — the filter that
fires the pyros must not have its bandwidth moved by a clock — which biases its vertical
velocity 2.6 % low and means anything gated on that filter *settling* moves with the sample
rate (landing detection spreads 2.3 s across 380–480 Hz, all of it after both pyros fire).

## 5. Verified runs

**`hil-dual`, 2026-08-17.** Osiris replay, boot-armed, 20 s pad hold (trajectory time =
wall − 20 s). No panics or faults; ended on the harness timeout. Predates the accelerometer
ignition trigger, so ignition (21.12 s) and the lockout exit (47.12 s) both move ~1 s earlier
on the next run — the lockout *duration* is unchanged, only its anchor.

| wall | event | vs truth |
|---|---|---|
| 38.94 s | vertical filter born, not forced, 6967 m, vv 229.0 m/s | trajectory 18.94 s vs host suite 18.87 s |
| 47.12 s | Mach lockout over | **26.0015 s** for a 26.000 s config |
| 59.64 s | airbrakes estimator retired | true apogee 39.58 s → **+0.06 s** |
| 62.68 s | descent detected, peak 9347.4 m AGL | truth 9329.6 m pressure-AGL → **+17.8 m** |
| 63.68 s | PyroDrogue | apogee call + **1.0026 s** for a 1.000 s config |
| 386.39 s | PyroMain | 457.2 m AGL crossing → **−0.01 s** |
| 497.15 s | landed | 5 s stillness persistence after touchdown |

`PyroMain` is the control here: it is an altitude crossing rather than a timer, and it did not
move across any of the timer fixes.

**`hil-single`, 2026-07-10.** Full radio path end-to-end: `SelfTest → LowPower → Armed →
Ascent → DrogueChute → MainChute → Landed`, apogee 3295 m AGL, airbrakes commanded full
(target 2500 m), both pyros fired via real GPIO (SD `pyro_*_fire` edges), RSSI ~−44, every
uplink acked first try.

## 6. Traps found the hard way

**Host and board computed different arithmetic from the same source line.**
`approximate_air_density` was written `x.powf(4.256)`; the method form resolves to inherent
`f32::powf` under std and to whatever `F32Ext` trait is in scope under `no_std`. With
`micromath` in scope the board's density ran up to 39 % low at altitude, inflating the drag
check's inverted airspeed 28 % and delaying filter birth from 18.9 s to 22.1 s. Fixed by
calling `libm::powf` by name (same for `sqrt`/`tan`/`powi`), after which `micromath` left the
dependency list. **If bench and host suite disagree, suspect trait-resolved float math before
suspecting the sensors** — everything else was ruled out against the SD log first (logged
accel matched the baked table to <1 %, integrating logged sensors reproduced true altitude to
40 m).

**`SD write took ~55ms (queue depth 0)` is benign.** It exceeds the 40 ms warn threshold, but
with queue depth 0 and no "queue full"/offline messages this card is simply slower than the
warn budget under high-rate logging.

## 7. What HIL does not prove

- **Airbrakes are open-loop.** Commands never feed back into the scripted baro/IMU, so the
  MPC is shown to *command*, never to have authority. Servo motion changes nothing downstream.
- **The near-apogee validation deploy does not fire by default** — both trajectories overshoot
  their target, so the MPC saturates full early and `mpc_went_full` latches. To exercise it,
  arm with a target *above* natural apogee (>3.3 km single, >9.4 km dual).
- **Icarus/AMP have historically been offline on the bench**, so `actual_extension` and
  `servo_temp` stayed absent and the servo homing path was never exercised. With Icarus
  attached the servo does move: homing sweeps its full travel on every `servo_active` entry,
  including after a mid-flight re-home, and `INVERT_DIRECTION` in `ICARUS-V2/firmware/src/servo.rs`
  is still marked unverified against real hardware.
- **No supersonic *scripted* profile exists.** `hil-dual` reaches Mach 1.91 and is the only
  bench profile that exercises the Mach lockout at all; `hil-single` never leaves subsonic and
  configures no lockout.
