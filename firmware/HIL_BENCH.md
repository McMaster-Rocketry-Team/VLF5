# VLF5 + Endgame HIL Bench — Current Setup

Physical bench setup for hardware-in-the-loop testing of the **VLF5 avionics** and the
**Endgame GCM** (ground control module). This is the *lab wiring / host access / control-path*
reference. For the **in-process** sensor-replay plots that run entirely on the VLF5 with fake
sensors (no radio), see [HIL.md](HIL.md).

> Probed and written 2026-07-09. Spans three repos:
> `VLF5/firmware` (this dir), `The_Endgame/{gcm_firmware,main_firmware}`, `Rust_Monorepo`.

---

## 0. HIL over USB — the working end-to-end path (no GCM, no radio) ✅

As of 2026-07-09 the `hil-single`/`hil-dual` firmware **bridges the full VLP path over the
VLF5's own USB** (WinUSB `c0de:cafe`), so a bench needs no GCM and no radio link. Sensors,
GPS, and pyro GPIO are faked (bench-safe, no e-matches); the mode machine, baro estimator,
airbrakes MPC, pyro queue, SD logger, and telemetry are all real.

```bash
cd VLF5/firmware
# 1. Flash HIL + watch RTT (keep this running; pick the VLF5 probe)
cargo build --release --bin main --features hil-single
probe-rs run --chip STM32H743VIHx --connect-under-reset \
  --probe 0483:374b:066EFF525086874967123920 target/thumbv7em-none-eabihf/release/main

# 2. In another shell: stream telemetry + drive the flight over USB
rocket-cli control --usb            # JSON telemetry to stdout, commands on stdin
#   commands: arm | mode <low-power|self-test|armed|landed|demo> | target-apogee <m>
#             | fire-pyro <main|drogue> | reset [all|void-lake|amp|icarus] | quit
rocket-cli send-uplink --usb arm    # one-shot

# Or drive a whole single-deploy flight + assert the outcome:
ROCKET_CLI=…/rocket-cli python3 hil_operator.py --usb --target 3000 --rtt-log rtt.log
```

Wire protocol (firmware `usb_handler.rs` + rocket-cli `gs/headless.rs`): VLP **downlinks**
stream on **EP2 bulk-IN** as `[len:u8][payload]` frames; **uplinks** ride a
`CliRequest::Uplink=4` **control-OUT** whose data stage is a serialized `VLPUplinkPacket`,
injected via `inject_uplink` (no VLP signing — USB is a trusted local link, so an uplink is
reported `{"type":"sent"}`, not ack'd). Verified: full single-deploy flight
(`Armed → PoweredAscent → drogue+main at apogee → Landed`, apogee ~3295 m AGL, airbrakes
commanded, both pyros fired in RTT). The GCM/LoRa path (§5c) is the *field* path but is
currently RF-blocked on this bench (GCM hears no packets); prefer USB for bench HIL.

---

## 1. Topology

```
                         ┌───────────────────────── this laptop ─────────────────────────┐
                         │                                                                │
  ST-Link 066EFF ── SWD ─┤ probe-rs (flash + defmt RTT)                                    │
        │                │                                                                │
   [ VLF5 H743 ]         │  rocket-cli:                                                   │
    USB c0de:cafe ───────┤   • flight-log over VLF5 WinUSB (c0de:cafe)                    │
    (WinUSB, flight log)  │   • ground-station TUI over GCM serial (120a:0005)            │
                         │                                                                │
  ST-Link 066BFF ── SWD ─┤ probe-rs (flash + defmt RTT)                                    │
        │                │                                                                │
   [ Endgame GCM F405 ]  │                                                                │
    USB 120a:0005 ───────┤  CDC-ACM /dev/ttyACM2  ◄── LoRa RPC (rkyv/CRC8 @115200)        │
    (CDC serial)         └───────────────────────────────┬────────────────────────────────┘
        │                                                 │
        └── SX1262 / E22 ─── 915 MHz LoRa ────────────────┘
                 ▲
                 │  VLP (signed + Reed-Solomon), downlink-first half-duplex
                 ▼
           [ VLF5 avionics radio ]   ← only in a normal flight build (not in-process HIL)
```

Also physically present but **not a named test target**: **Endgame Main** (F405,
`120a:0006`, `/dev/ttyACM1`) — no debug probe attached, USB-only.

---

## 2. Hardware inventory (as probed)

| Board | MCU | Debug probe (ST-Link V2.1 serial) | Probe VCP | App USB | App tty | Firmware |
|-------|-----|-----------------------------------|-----------|---------|---------|----------|
| **VLF5 avionics** | STM32H743VIHx (M7) | `066EFF525086874967123920` | `/dev/ttyACM0` | `c0de:cafe` WinUSB | — (vendor, no tty) | `VLF5/firmware` bins `main`,`gcm`,`sd_test` |
| **Endgame GCM** | STM32F405RGTx (M4) | `066BFF525086874967123919` | `/dev/ttyACM3` | `120a:0005` CDC | `/dev/ttyACM2` | `The_Endgame/gcm_firmware` bin `firmware` |
| **Endgame Main** | STM32F405RGTx (M4) | *(none attached)* | — | `120a:0006` CDC | `/dev/ttyACM1` | `The_Endgame/main_firmware` |

How the mapping was established (authoritative, not guessed):
- `066EFF…3920` → **SWD reports DPv2 / STMicro / Part `0x4500`** = STM32H743 → VLF5.
- `066BFF…3919` → SWD reports DPv1 / Cortex-M4+ETM = an F405. Read the unique-ID register
  `0x1FFF7A10` through this probe → `3d0052000251343039333936`, an exact match for the
  **GCM's** USB serial (Main's is `390052…`). So this probe is on the **GCM**, and the Main
  has no probe.

Notes:
- The two `/dev/ttyACM0` and `/dev/ttyACM3` are the **ST-Links' own virtual COM ports** (UART
  passthrough), *not* the defmt channel. **defmt RTT travels over SWD** and is selected by
  **probe serial**, not by a tty.
- VLF5 has **no CDC tty** — its `c0de:cafe` USB is a vendor/WinUSB interface (bulk-IN `0x81`),
  used only by rocket-cli for SD flight-log list/download/clear.

---

## 3. Host access / permissions  ✅ resolved

`pegasis` is in `plugdev` (not `dialout`), so access is granted by udev rules (all installed):

| Rule file (`/etc/udev/rules.d/`) | Covers | Effect |
|---|---|---|
| `69-probe-rs.rules` | ST-Link `0483:374b` | probe rw (flash + RTT) — both probes work |
| `99-vlf5-flight-log.rules` | `c0de:cafe` | VLF5 WinUSB rw (rocket-cli flight log) |
| `99-endgame-usb.rules` | `120a:0005`, `120a:0006` | GCM + Main serial **and** raw USB rw; `ID_MM_DEVICE_IGNORE` |

Current verified access: `/dev/ttyACM0..3` all rw; GCM/Main raw USB nodes `0666 plugdev` rw;
both ST-Links rw. Ports open cleanly and are unheld.

**ModemManager is `active`+`enabled`** — it will transiently grab a fresh `ttyACM` after each
re-enumeration. The `ID_MM_DEVICE_IGNORE` lines added to `99-endgame-usb.rules` fix this, but
you installed the file **before** that edit — re-copy and reload to apply:

```bash
sudo cp ../The_Endgame/99-endgame-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
```

---

## 4. Toolchain

- `probe-rs` at `~/.cargo/bin/probe-rs`; `cargo`; Rust **nightly-2025-07-11**, target
  **`thumbv7em-none-eabihf`** (both firmwares pin the same toolchain).
- GCM links with **flip-link**. No `st-info`/`openocd`/`st-flash` installed (probe-rs only).
- `rocket-cli`: `Rust_Monorepo/rocket-cli` (host binary, runs on the laptop).

### ⚠ Multi-probe gotcha (important for this bench)
Two ST-Links are attached, but neither firmware's runner pins a probe
(`.cargo/config.toml` runner is `probe-rs run --chip … --connect-under-reset`; `Embed.toml`
has no probe selector). With >1 probe present, **probe-rs errors unless you pass `--probe`**.

- VLF5: `--probe 0483:374b:066EFF525086874967123920`
- GCM:  `--probe 0483:374b:066BFF525086874967123919`

Either pass `--probe …` on the command line, or add it to the runner line in each repo's
`.cargo/config.toml` so `cargo run` targets the right board unambiguously.

---

## 5. Control paths

### 5a. Flash + live defmt logs (either board) — via probe-rs/SWD
From `VLF5/firmware` (VLF5) or `The_Endgame/gcm_firmware` (GCM):
```bash
# VLF5 (H743) — pick the VLF5 probe explicitly
cargo run --release --bin main -- --probe 0483:374b:066EFF525086874967123920      # may need config edit; see §4
# GCM (F405)
cd ../The_Endgame/gcm_firmware && cargo run --release -- --probe 0483:374b:066BFF525086874967123919
```
DEFMT_LOG levels are preset in each `.cargo/config.toml`.

### 5b. VLF5 SD flight-log — via VLF5 WinUSB (`c0de:cafe`), rocket-cli
```bash
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- list-flight-log
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- download-flight-log out.csv
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- clear-flight-log
```
USB command set is exactly List/Download/Clear (vendor control transfer `wValue`, bulk-IN `0x81`).

### 5c. Commanding the avionics over the real radio — via GCM, rocket-cli ground-station
```bash
cargo run -p rocket-cli --manifest-path ../../Rust_Monorepo/rocket-cli/Cargo.toml -- ground-station
```
- Auto-finds the single GCM (`120a:0005`) on `/dev/ttyACM2`, opens at 115200, does the RPC
  reset handshake, then `configure`s the radio (freq/power from `ground-station.toml`; SF12 /
  BW 250 kHz / CR 4/8 hardcoded).
- Data path: `rocket-cli` VLP daemon (signs + ECC) → RPC (`configure`/`rx`/`tx`/`tx_then_rx`,
  rkyv+CRC8 over CDC) → GCM firmware → SX1262/E22 → 915 MHz → avionics VLP daemon.
- **Uplinks are TUI buttons, not CLI flags** (see §7).

---

## 6. VLP protocol quick catalog

Shared crate `Rust_Monorepo/firmware-common-new/src/vlp/`. Downlink-first half-duplex: the
rocket transmits telemetry, then listens ~500 ms; the ground station piggybacks a signed
uplink into that gap; the rocket replies with a signed `Ack`. Uplinks are
`SHA256(key ‖ last_downlink ‖ uplink)[..16]`-signed; everything is Reed-Solomon ECC. Shared
`vlp_key` is a 32-byte secret that **must match** across avionics + `ground-station.toml`
(`VLF5/firmware/vlp.key` is the base64 form flashed into firmware).

**Uplink (ground→rocket)** — `ChangeMode`, `Reset`, `PayloadEPSOutputOverwrite`,
`AMPOutputOverwrite`, `FirePyro` (**Armed-only**, ignored otherwise), `SetTargetApogee`.

**Downlink (rocket→ground)** — `GPSBeacon`, `Ack`, `LowPowerTelemetry`, `Telemetry`,
`SelfTestResult`, `LandedTelemetry`.

**Avionics modes** (`src/avionics_mode.rs`): `LowPower`, `SelfTest`, `Armed`, `Landed`, `Demo`.
The only external transition trigger is `ChangeMode`; `Armed→Landed` is automatic (terminal
estimator state + 30 s). Telemetry cadence: Armed `Telemetry` 2 s; LowPower/Demo/Landed 5 s;
SelfTest 2 s. Uplink handling is centralized in `src/receive_vlp_task.rs` (mode-independent
except `FirePyro`).

**GCM LoRa RPC** (`firmware-common-new/src/rpc/lora_rpc.rs`): `0 configure(LoraConfig)`,
`1 rx(timeout_ms)`, `2 tx(len,data)`, `3 tx_then_rx(...)`. Per-call `rx` is capped at 254
symbols (~4.2 s at SF12/250 kHz), so the host loops `rx(4000)`.

---

## 7. Two HIL modes — pick the right one

| | **In-process replay** (existing) | **Over-the-air** (real radio) |
|---|---|---|
| Avionics build | `--features hil-dual` / `hil-single` | **normal flight build** (no hil feature) |
| Sensors/GPS/pyro | faked in-process | real (⚠ pyro drives GPIO — no e-matches!) |
| Radio / GCM | **not used** (radio daemon replaced) | GCM + SX1262 carry real VLP |
| Command injection | scripted `inject_uplink` (t≈2s apogee, t≈3s Armed) | rocket-cli `ground-station` TUI |
| Observe | defmt RTT + SD dump | ground-station telemetry + RTT + SD dump |
| Doc | [HIL.md](HIL.md) | this file, §5c |

They are **mutually exclusive**: the in-process HIL build replaces the LoRa daemon, so it
cannot talk to the GCM. To exercise the GCM↔avionics radio link, flash a normal avionics build.

### Automation gap to solve for scripted HIL
The VLP uplink actions (**ChangeMode / SetTargetApogee / FirePyro / Reset / overrides**) are
**interactive buttons inside the `ground-station` TUI** — there is **no** non-interactive
rocket-cli subcommand for them. For scripted over-the-air HIL we'll need one of:
- a small rocket-cli subcommand that sends a single `VLPUplinkPacket` and exits, or
- drive the GCM's LoRa RPC directly (`configure`/`tx_then_rx`) with our own signing, or
- reuse the in-process `hil_script` approach for avionics-only runs (no radio).

Non-interactive helpers that already exist: `testing send-vlp-telemetry`, `testing
mock-ground-station`, and the flight-log commands.

---

## 8. Command cheatsheet

```bash
# Discover
probe-rs list                                   # both ST-Links
lsusb -d c0de:cafe; lsusb -d 120a:0005; lsusb -d 120a:0006
ls -l /dev/serial/by-id/                         # ttyACM ↔ board mapping

# Probe serials (copy/paste)
#   VLF5 : 0483:374b:066EFF525086874967123920  (ttyACM0 = its ST-Link VCP)
#   GCM  : 0483:374b:066BFF525086874967123919  (ttyACM3 = its ST-Link VCP)

# GCM serial : /dev/ttyACM2      Main serial : /dev/ttyACM1
```
