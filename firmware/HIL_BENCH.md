# VLF5 + Endgame HIL Bench — Current Setup

Physical bench setup for hardware-in-the-loop testing of the **VLF5 avionics** and the
**Endgame GCM** (ground control module). This is the *lab wiring / host access / control-path*
reference. For the HIL firmware model (what's faked vs real, the flight timeline, RTT
checklist), see [HIL.md](HIL.md).

> Probed 2026-07-09; §0/§5c/§7 rewritten 2026-07-10 for the baro-only-over-radio HIL model.
> Spans three repos: `VLF5/firmware` (this dir), `The_Endgame/{gcm_firmware,main_firmware}`,
> `Rust_Monorepo`.

---

## 0. HIL over the real GCM/radio — the working end-to-end path ✅

The `hil-single`/`hil-dual` firmware fakes **only the barometer reading**; IMU, GPS, pyro
**GPIO (fires for real!)**, mag, CAN, SD, and the **real LoRa radio** all run. The board
**boots into SelfTest** exactly like a flight build, and you fly it from `rocket-cli` over the
**GCM + LoRa**, same as a real flight. (The old USB-VLP-bridge path and the `hil-radio`
feature are **deleted** — HIL always uses the real radio now.)

> **Real pyro GPIO energizes the drogue/main FETs at apogee — never flash HIL with live
> e-matches connected.**

```bash
cd VLF5/firmware
# 1. Flash HIL + watch RTT (keep this running; pick the VLF5 probe)
cargo run --release --bin main --features hil-single -- \
  --probe 0483:374b:066EFF525086874967123920

# 2. Clean the SD log (it accumulates ~1.1M records across runs), then fly over the radio
rocket-cli clear-flight-log                                       # VLF5 WinUSB c0de:cafe
rocket-cli control --frequency 920000000 --vlp-key "$(cat vlp.key)"
#   Board boots SelfTest. Downlink JSON types: self_test_result / low_power_telemetry /
#   telemetry / landed_telemetry, plus ack|nack|timeout. Commands on stdin:
#   arm | mode <low-power|self-test|armed|landed|demo> | target-apogee <m>
#   | fire-pyro <main|drogue> | reset [all|void-lake|amp|icarus] | quit
#   Sequence: confirm self_test_result -> `mode low-power` -> `arm` -> fly (~3.5 min) -> landed
rocket-cli send-uplink --frequency 920000000 --vlp-key "$(cat vlp.key)" arm   # one-shot

# 3. Pull the flight log (this flight only, if cleared first)
rocket-cli download-flight-log out.csv
```

Verified 2026-07-10 end-to-end over the air: `SelfTest → LowPower → Armed → Ascent →
DrogueChute → MainChute → Landed`, apogee ~3295 m AGL, airbrakes commanded full
extension (target 2500 m < natural apogee), **both pyros fired via the real `pyro_task` GPIO**
(SD `pyro_*_fire` edges), link strong (RSSI ~-44, every uplink acked first try).
Flight-data logging is gated to **Armed→Landed** (`AvionicsMode::should_log`),
so preflight is not logged and downloads stay small.

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
        └── SX1262 / E22 ─── 920 MHz LoRa ────────────────┘
                 ▲
                 │  VLP (signed + Reed-Solomon), downlink-first half-duplex
                 ▼
           [ VLF5 avionics radio ]   ← used in BOTH flight and HIL builds (HIL fakes only baro)
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

### 5c. Commanding the avionics over the real radio — via GCM, rocket-cli
```bash
# Interactive TUI:
rocket-cli ground-station
# Non-interactive (scriptable) — streams downlink JSON on stdout, reads commands on stdin:
rocket-cli control --frequency 920000000 --vlp-key "$(cat vlp.key)"
# One-shot uplink:
rocket-cli send-uplink --frequency 920000000 --vlp-key "$(cat vlp.key)" arm
```
- Auto-finds the single GCM (`120a:0005`) on `/dev/ttyACM2`, opens at 115200, does the RPC
  reset handshake, then `configure`s the radio. There is **no `ground-station.toml`** on this
  bench, so `control`/`send-uplink` need `--frequency 920000000 --vlp-key "$(cat vlp.key)"`
  (SF12 / BW 250 kHz / CR 4/8 are hardcoded).
- Data path: `rocket-cli` VLP daemon (signs + ECC) → RPC (`configure`/`rx`/`tx`/`tx_then_rx`,
  rkyv+CRC8 over CDC) → GCM firmware → SX1262/E22 → 920 MHz → avionics VLP daemon.
- `control` command grammar + downlink JSON schema: `rocket-cli/src/gs/headless.rs`.

---

## 6. VLP protocol quick catalog

Shared crate `Rust_Monorepo/firmware-common-new/src/vlp/`. Downlink-first half-duplex: the
rocket transmits telemetry, then listens ~500 ms; the ground station piggybacks a signed
uplink into that gap; the rocket replies with a signed `Ack`. Uplinks are
`SHA256(key ‖ last_downlink ‖ uplink)[..16]`-signed; everything is Reed-Solomon ECC. Shared
`vlp_key` is a 32-byte secret that **must match** across avionics and the ground station
(passed via `--vlp-key "$(cat vlp.key)"`, or `ground-station.toml` if one exists).
`VLF5/firmware/vlp.key` is the base64 form flashed into firmware.

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

## 7. HIL build vs full flight build

There is **one** HIL model now: `--features hil-single` / `hil-dual` fakes **only the
barometer** and runs everything else for real over the GCM/radio. A HIL build differs from a
real flight in exactly three ways:

| | **HIL build** (`hil-single`/`hil-dual`) | **Full flight build** (no hil feature) |
|---|---|---|
| Barometer | **simulated** scripted trajectory (`baro_sim.rs`) | real MS5607 |
| IMU / GPS / pyro GPIO / mag / CAN / SD / LoRa | **real** (pyro fires for real ⚠ no e-matches) | real |
| Boot mode | SelfTest | SelfTest |
| Deploy profile | compile-time (`hil-single` = both at apogee) | `FlightProfile::Dual` default |
| Command path | rocket-cli `control`/`send-uplink`/`ground-station` over GCM | same |
| Observe | GCM telemetry JSON + RTT + SD dump | same |
| Doc | [HIL.md](HIL.md) | — |

Scripting is fully supported: `rocket-cli control` streams downlink JSON on stdout and reads
`arm` / `mode …` / `target-apogee …` / `fire-pyro …` / `reset …` / `quit` on stdin; `send-uplink`
is the one-shot form. (The old interactive-only "automation gap" is closed, and the old
in-process USB-VLP-bridge / `hil-radio` / `vlp_script` paths are deleted.)

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
