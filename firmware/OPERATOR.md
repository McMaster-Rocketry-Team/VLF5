# VLF5 Operator Guide

Pad guide: listen for self-test beeps, then fly with **rocket-cli**.

---

## What you need

- VLF5 powered on the rocket
- Exactly one GCM plugged into the laptop
- `rocket-cli` installed (needs **Rust nightly** via rustup)

```bash
cargo +nightly install --profile rocket-cli --git https://github.com/McMaster-Rocketry-Team/Rust_Monorepo.git --locked rocket-cli
```

If that install fails, get the team — do not troubleshoot the laptop yourself.

Left side = command buttons. Right side = telemetry from the rocket.

If ground-station fails to open or crashes, check that exactly one GCM is plugged in and try again. If it still fails, get the team (laptop USB permissions).

---

## 1. Power on — self-test beeps

1. Power on VLF5. Right away you get **five beeps**: Low, Mid, High (VLF5 is alive),
   then a short pause, then two more reporting igniter continuity:

| Meaning | Last two beeps |
|--------|--------|
| **Both igniters have continuity** | High, High |
| **One or both do not** | Low, Low |

   Low-Low here is expected before the igniters are wired in. Once they are in,
   Low-Low means a channel is open — fix it before continuing.
2. Wait for self-test to finish (about half a minute or more).
3. When self-test finishes, listen for **four beeps in a row**:

| Meaning | Beeps |
|--------|--------|
| **OK** | Low, Low, High, High |
| **Partial** | Low, Low, High, Low |
| **Failed** | High, High, Low, Low |

- **OK** → continue.
- **Partial** → something is unhealthy; check with the team before flying.
- **Failed** → do not arm until fixed.

---

## 2. Open ground-station

1. After you hear the finish beeps, open ground-station:

```bash
rocket-cli ground-station
```

2. Wait a few seconds. On the right, expect **Self Test Result**.
3. If the right panel is still empty (no Self Test Result), **stop**. Do not continue — get the team to set up rocket-cli for this rocket first.

---

## 3. Before launch

1. Click **Target apogee**, enter height in meters, confirm.
2. Click **Low Power Mode**, confirm.
3. On the right, expect **Low Power Telemetry**.
4. Leave it in Low Power until you are ready to launch.

---

## 4. Just before launch

1. Click **Armed Mode**, confirm.
2. On the right, expect **Telemetry** updating.
3. Keep ground-station open and watch **Telemetry** through the flight.

Reading the panel: a dimmed **`n/a`** means the rocket did not report that
number, not that it is zero. It is normal, not a fault. Two places you will see
it on a good flight:

- **altitude agl** and **vertical velocity** go `n/a` for a few seconds around
  Mach 1 while **state** still says `Ascent`. The altitude filter is locked out
  there and has nothing honest to report. **max altitude agl** keeps working
  straight through, so that is the number to read.
- **actual extension** and **servo temp** are `n/a` until Icarus sends its
  first report, even while **icarus online** is already true. That means "not
  reported yet", not "brakes stowed at 0 C".

On the pad, **altitude agl** should sit near 0 m rather than blank — that is
the barometer and the filter confirming they are alive before launch.

**Armed means the rocket can fire parachute charges on its own.** Only arm when you intend to fly.

**Fire Main Pyro** and **Fire Drogue Pyro** are emergency / manual only — they ask for confirm and only work while Armed.

---

## 5. After the flight

1. Optionally wait for **Landed Telemetry** (or click **Landed Mode** if you need to force it from the ground).
2. Keep VLF5 **powered on**, then plug it into the laptop with **USB-C** (separate from the GCM).
3. Open **another terminal** (or stop ground-station first — that session is busy), then dump the flight log:

```bash
rocket-cli list-flight-log
rocket-cli download-flight-log flight_log.csv
```

If rocket-cli can’t find VLF5 or the dump fails even though it’s powered and plugged in, get the team (laptop USB permissions).

Only clear the log if the team asks:

```bash
rocket-cli clear-flight-log
```

---

## Quick checklist

**Power on**

- [ ] Hear power-on beeps; last two are High, High (both igniters have continuity)
- [ ] Hear finish beeps (OK / partial / failed)

**Before launch**

- [ ] After finish beeps: open `rocket-cli ground-station`; wait a few seconds for Self Test Result (still empty → stop — get the team)
- [ ] **Target apogee**
- [ ] **Low Power Mode**; see Low Power Telemetry

**Just before launch**

- [ ] **Armed Mode**
- [ ] Watch **Telemetry**

**After flight**

- [ ] VLF5 powered on + USB-C to VLF5
- [ ] Another terminal (or stop ground-station): `rocket-cli download-flight-log flight_log.csv`
