# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Storage format v13. Updated 2026-08-17.

## How absence is represented

One rule governs every table below: **absence of data is an `Option`, never a
sentinel value.** There is no NaN meaning "no source", no `0xFFFF` meaning
"unavailable", no zero `unix_time_us` meaning "clock not locked", and no
`valid` bitmask sitting beside the fields it describes. If a value is missing,
the type says so, and a reader cannot pair the wrong validity bit with the
wrong row because there is no validity bit to pair.

Sentinels survive in exactly two places, both of them media that physically
cannot carry an `Option`: the bit-packed LoRa packets and the CAN bus. Even
there they are boundary-only — every packet field with an absence encoding is
built from an `Option` and read back through a getter that returns one, and
every `0xFFFF` on CAN is decoded to `None` by the accessor that reads it. No
code above the wire ever sees the sentinel.

The cost is size: a sentinel is free, an `Option` costs a discriminant plus
padding, which is why the v13 records are half again as large as v11's. That is
the deliberate price of a log that cannot mistake a reading for its own absence.

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding:
`rocket-cli download-flight-log <out.csv>` (one CSV row per fast record, latest
slow snapshot merged in). The storage format is **not** backward compatible at any version:
logs written by other firmware are reported as unsupported by rocket-cli, and
the firmware starts a fresh log over them on the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz — exactly one record per published sensor sample, no filler | 161 B (160 B body + tag), 3 per block | sequence, timestamp_us, **unix_time_us** (GPS-disciplined, absent until the clock locks), `imu` (accel ×3 + gyro ×3, one group — they come from the same read), baro pressure, mag ×3 (100 Hz source, resampled), `deployment` (`kf_altitude_asl` + `kf_vertical_velocity` + `flags`: baro innovation-gate reject + resync, per sample), `airbrakes` (`kf_altitude_asl` + `kf_vertical_velocity` + `kf_tilt_deg` + `flags`: burnout latch, pad calibration complete, two-bit state), flight stage (`RocketState` mirror, Mach lockout folded into `Ascent`, logged only here), **pyro flags** (continuity / fire / short, at ±2.3 ms), `air_brakes` (**commanded + actual extension**, **validation-deploy flag** — here for the edges, not the values) |
| **Slow** (tag 0x02) | 10 Hz | 249 B (248 B body + tag), 2 per block | timestamp_us, **baro temperature** (~13 Hz source), battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), **`launch_pad_altitude_asl`** (the one reference every AGL number is measured from), `air_brakes` (**servo temp**, **MPC predicted apogee ASL**, **MPC target apogee ASL**), **full `NodeStatus` for AMP / Icarus / OZYS / payload SDRM** (uptime, health, mode, custom status), `amp` (**output statuses + shared battery voltage**), `payload` (**EPM battery mV + six rail currents mA + three SEM actuator step counts + three SEM load cells cN + the packed experiment flag word**) |

Throughput ≈ 71 kB/s of record payload (427 Hz × 161 B + 10 Hz × 249 B).
Blocks are what actually reach the card: three fast records per block is
142 blocks/s, two slow records per block is another 5, so ≈ 147 blocks/s
≈ 75 kB/s ≈ 271 MB/hour. Moving the extensions to the fast record in v19 did
not cost a single block write — the fast record still packs three to a block
and the slow one now packs two where it packed one. Capacity remains a
non-issue.

The slow record is the one with no room left: two 249 B records plus the 8 B
block header is 506 of 512, so **6 bytes of slack**. The next field added to it
drops the pack from two records per block to one and doubles the slow block
rate — still affordable, but it is a step, not a slope, and worth knowing
before it is paid by accident.

Absence means "no source is producing this", uniformly, and every one of these
reads as an empty cell in the CSV rather than as a number:
the `deployment` group until the armed-mode estimator produces its first sample
(a few ms after arming); the `airbrakes` group until its piece of the airbrakes
estimator is alive (tilt from ignition, altitude/velocity from the vertical
filter's birth) **and again from apogee onward**, when the airbrakes estimator
is retired — dropped outright, so the whole descent has no `airbrakes_*`
columns at all, flags included; the airbrakes commanded extension until the
firmware commands one; and the airbrakes actual extension and servo temp until
Icarus first reports (both on the same condition, though they now sit on
different records — the extension on the fast one, the temperature on the
slow). The last of those is the one worth remembering when
reading a log: a silent or offline Icarus shows as an empty cell, never as 0.0,
so it cannot be mistaken for stowed brakes at 0 C. The one exception to
absence-encoding is `air_brakes_validation_deploy`, which is a flag rather than
an absence: a commanded 1.0 from the MPC and a commanded 1.0 from the
validation deploy are the same number, so absence-encoding cannot separate
them.

Two levels of absence are worth reading carefully, because the nesting is the
whole point of the record shape:

- `deployment: Some(...)` with `kf_altitude_asl: None` inside is the **Mach
  lockout**: an estimator sample exists for this tick, and the filter has
  nothing to say because it is frozen and holds a stale pre-ignition reading.
  `deployment: None` is different — no estimator sample matched that tick at
  all, so its gate bits are absent too.
- A node's outer `Option` being `None` means **never heard from at all**;
  `NodeStatusRecord::online == false` means the node spoke and then went quiet
  for 5 s, and the rest of its fields describe its last heartbeat. The old
  fabricated `NodeStatusRecord::offline()` is gone, so "never on the bus" no
  longer reads as "booted, then dropped".

`flight_stage` gives stage transitions at ~2.3 ms resolution, and the
per-sample burnout bit and the two-bit airbrakes state reconstruct the
lockout exit post-flight; the pyro fire bits get the same resolution. Through the Mach
lockout the deployment KF columns are **empty**, not frozen — the accessors
that feed the log and the accessors that feed the downlink are the same two
functions, so the two channels describe that window identically by
construction. `flight_stage` reads `Ascent` throughout (the lockout is folded
into `Ascent` on the wire), so the lockout is identified by the `deployment_*`
columns going blank rather than by the stage value; the `airbrakes_*` columns
are live throughout and are what to read there.

## Radio telemetry (VLP downlink, LoRa)

| Packet | Size | When | Contents (summary) |
|---|---|---|---|
| `TelemetryPacket` | 38 B (300/304 bits — 4 spare) | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, deployment altitude AGL + max + vertical velocity (signed, −400..1050 m/s @ ~1.4 m/s), airbrakes tilt, flight stage (3-bit `RocketState` mirror, Mach lockout folded into `Ascent`), **airbrakes enabled**, **MPC predicted apogee AGL**, **target apogee AGL**, amp status + 3 outputs + shared battery, Icarus status + airbrakes ext/temp, OZYS + SDRM status, payload stack status, **EPM battery + six rail currents + three SEM actuator step counts** |
| `LowPowerTelemetryPacket` | 13 B (98 bits — 6 spare) | every 5 s in LowPower + Demo | sats, gps_fixed, **lat/lon**, VL battery, amp online, shared battery, air temp, **payload EPM battery** |
| `LandedTelemetryPacket` | 11 B (88 bits — 0 spare) | every 5 s in Landed | lat/lon, sats, VL battery, amp status + outputs + shared battery |

Six of the `TelemetryPacket`'s bits are validity bits. The packet is a
bit-packed `packed_struct` and cannot carry an `Option` on the wire, so each of
these is the boundary encoding for one specific absence, converted back to an
`Option` by the getter:

| bit | field(s) guarded | means absent when |
|---|---|---|
| `lat_lon_valid` | `lat`, `lon` | no position solution |
| `deployment_kf_valid` | `deployment_kf_altitude_agl` + `deployment_kf_vertical_velocity` (shared) | the filter is unborn, or the Mach lockout has it frozen |
| `max_deployment_kf_altitude_valid` | `max_deployment_kf_altitude_agl` | before the first sample — it is latched, so it stays valid through the lockout and after landing |
| `airbrakes_kf_tilt_valid` | `airbrakes_kf_tilt_deg` | before ignition, and after the estimator is retired at apogee |
| `mpc_predicted_apogee_valid` | `mpc_predicted_apogee_agl` | the MPC is not running, including throughout the validation deploy |
| `icarus_status_valid` | `air_brakes_actual_extension_percentage` + `air_brakes_servo_temp` (shared) | Icarus is heartbeating but has not yet sent an `IcarusStatusMessage` |

That last one is subtler than it looks. `icarus_online` is not a substitute:
the CAN heartbeat starts at boot, before any `IcarusStatusMessage`, and in that
window `icarus_online` is true while the two fields still hold their initial
0.0 — which reads as "brakes fully retracted, servo at 0 C", a measurement
nobody took.

Stage on the downlink: during the deployment estimator's Mach lockout
`deployment_kf_valid` is clear, so `deployment_kf_altitude_agl` and
`deployment_kf_vertical_velocity` are **absent** (the ground station renders
them `n/a`) while the stage still reads `Ascent`. That pairing is the tell — a
blank altitude under an ascending rocket is a frozen filter, not a rocket
sitting at 0 m doing 0 m/s, which is what the old hardcoded zero looked like.
The SD log's `deployment_*` columns are blank over exactly the same rows.
`max_deployment_kf_altitude_agl` has its own bit and stays live through the
lockout, so the latched apogee never disappears from the display; tilt is the
only live instantaneous number on the downlink through that window, and the SD
log's `airbrakes_kf_*` columns are what to read for the rest.
FailedToReachMinApogee is reported as itself. Vertical velocity is signed and
comes from the deployment estimator, so descent rate is live all the way down —
it is the airbrakes estimator that is retired at apogee, and only
`airbrakes_born` and the tilt it feeds are downlinked from it now. The pair
worth watching on ascent is `mpc_predicted_apogee_agl` against
`target_apogee_agl`: while they agree the brakes have authority, and the gap
between them is the overshoot or undershoot the MPC cannot fix.

One knock-on worth knowing on the pad: the packet no longer forces a 0.0 for
`OnPad` / `Landed` / `FailedToReachMinApogee`. The deployment filter is running
and fusing baro in all three, so the ground now sees a **real near-zero AGL
while on the pad** — which is live confirmation that the baro and the filter
are alive before launch, where previously it was an indistinguishable hardcoded
zero.

## Full data map

Columns: **TM** = TelemetryPacket (2 s) · **LP/LD** = low-power / landed
packets (5 s) · **Fast** = SD @ ~427 Hz · **Slow** = SD @ 10 Hz.
A ✓ under **Fast** is about when the column can CHANGE, not how fast its source
runs: the brake extensions are fast-record columns fed by 10 Hz and 100 Hz
sources, and they are there so their edges are timestamped, not resampled.

| Value | TM 2s | LP/LD 5s | Fast | Slow | Derivable post-flight? |
|---|---|---|---|---|---|
| accel, gyro (3-axis) | – | – | ✓ (one `imu` group) | – | primary data |
| mag (3-axis) | – | – | ✓ (100 Hz source) | – | primary data |
| baro pressure | – | – | ✓ (never absent) | – | primary data |
| `temperature` (MS5607 die) | – | – | – | ✓ (never absent) | logged |
| `air_temperature` (same reading, downlinked) | ✓ | LP | – | – | yes — `temperature` in the slow record, unquantized |
| `deployment_kf_altitude_asl` | ✓ as `deployment_kf_altitude_agl` (14-bit, `deployment_kf_valid`) | – | ✓ (absent through the Mach lockout) | – | yes — replay pressure through estimator |
| `deployment_kf_vertical_velocity` | ✓ (signed 10-bit, shares `deployment_kf_valid`) | – | ✓ (absent through the Mach lockout) | – | yes — replay |
| `max_deployment_kf_altitude_agl` | ✓ (14-bit, `max_deployment_kf_altitude_valid`; latched, so live through lockout and landing) | – | – | – | yes — max() over fast records |
| `airbrakes_kf_altitude_asl` | – | – | ✓ (ASL, absent until born / after apogee) | – | yes — replay IMU+baro through estimator |
| `airbrakes_kf_vertical_velocity` | – | – | ✓ (absent until born / after apogee) | – | yes — replay |
| `airbrakes_kf_tilt_deg` | ✓ (8-bit, `airbrakes_kf_tilt_valid`) | – | ✓ (absent before ignition / after apogee) | – | yes — replay / offline attitude |
| `airbrakes_state` (`Armed`/`Stage1`/`DeadReckoning`/`AirbrakesEnabled`, one-way) | ✓ as `airbrakes_enabled` (1 bit, the last state only; 0 after apogee) | – | ✓ (2 bits at the top of `airbrakes.flags`) | – | partly — `Armed` vs `Stage1` is this half's OWN ignition detection and is nowhere else |
| airbrakes drag check | – | – | – | – | **nowhere** — its vote and the birth are the same row, and the birth is the state going to `AirbrakesEnabled` |
| airbrakes burnout latch | – | – | ✓ (`AIRBRAKES_BURNOUT`) | – | SD only; the packet has 5 spare bits, but see the air-time note below |
| airbrakes pad calibration complete | – | – | ✓ (`AIRBRAKES_PAD_CALIBRATED`) | – | SD only — **nowhere on the wire**; RTT line live, candidate for a VLCustomStatus bit |
| `flight_stage` (incl. FailedToReachMinApogee; Mach lockout folded into `Ascent`) | ✓ (3-bit) | – | ✓ full rate | – | logged |
| drogue/main deployed | – | – | ✓ (pyro fire flags) | – | read the stage transition, or the pyro fire bits for the GPIO edge |
| `air_brakes_target_apogee_asl` (what the MPC latched) | ✓ as `target_apogee_agl` (14-bit; the operator's live setting, which can differ) | – | – | ✓ (absent until the MPC is built) | the operator's AGL setting is also in the SD config block |
| deployment baro gate reject, resync | – | – | ✓ (`deployment.flags` bits, exact per sample) | – | logged |
| airbrakes baro gate reject, resync | – | – | – | – | **nowhere** — that filter has no gate; see the bullet below |
| KF innovation magnitude | – | – | – | – | yes — replay pressure |
| `launch_pad_altitude_asl` | – | – | – | ✓ (absent only before the estimator's first sample) | logged — previously had to be eyeballed off `deployment_kf_altitude_asl` while on pad |
| GPS `lat_lon` | ✓ (23-bit lat / 24-bit lon, ~2.4 m, `lat_lon_valid`) | ✓ all three | – | ✓ (f64, exact; absent until a fix) | logged |
| `num_of_fix_satellites` | ✓ (5-bit, **saturates at 31**) | ✓ (full u8) | – | ✓ (full u8; 0 is a real reading, so never absent) | logged |
| `gps_altitude_asl` | – | – | – | ✓ (absent until the fix carries one) | logged |
| `hdop` / `vdop` / `pdop` | – | – | – | ✓ (absent when the GPS did not report one) | logged — see below |
| GPS UTC time | – | – | ✓ (unix µs, full rate; absent until the clock locks) | – | logged — absolute time for video/GCM correlation |
| VL battery voltage | ✓ | ✓ | – | ✓ (absent until the ADC reports) | logged |
| pyro continuity (main/drogue) | ✓ | – | ✓ | – | logged |
| pyro fire outputs, short-circuit | – | – | ✓ (±2.3 ms) | – | logged |
| `air_brakes_commanded_extension` | ✓ (5-bit, always sent) | – | ✓ (absent until commanded; 10 Hz source) | – | logged |
| `air_brakes_actual_extension` | ✓ (5-bit, `icarus_status_valid`) | – | ✓ (absent until Icarus reports; 100 Hz source) | – | logged |
| `air_brakes_validation_deploy` | – | – | ✓ (a flag, not an absence) | – | logged |
| `air_brakes_servo_temp` | ✓ (9-bit, shares `icarus_status_valid`) | – | – | ✓ (absent until Icarus reports) | logged |
| Icarus servo current | – | – | – | – | **nowhere** — the field was dropped from `IcarusStatusMessage`; the servo does not measure current |
| MPC predicted apogee | ✓ (14-bit AGL, `mpc_predicted_apogee_valid`) | – | – | ✓ as `mpc_predicted_apogee_asl` (absent while the MPC is not running) | logged — SD stores ASL, the packet AGL; subtract `launch_pad_altitude_asl` |
| amp: online, outputs ×3, shared battery | ✓ | ✓/LD | – | ✓ (whole `amp` group absent until the first `AmpStatusMessage`) | logged |
| Icarus / OZYS / SDRM online + rebooted | ✓ (2 bits each) | – | – | ✓ (full `NodeStatus`; absent = never heard from) | logged |
| node uptime, health, mode (all four) | – | – | – | ✓ | logged |
| OZYS disk usage, gauge-connected, `sd_ok` | – | – | – | ✓ (`ozys_custom_status`, decode with `OzysCustomStatus`) | logged |
| OZYS strain measurements | – | – | – | – | **nowhere** — OZYS logs to its own SD |
| payload stack flags ×8 (`payload_epm_alive` etc.) | ✓ (1 bit each) | – | – | – | **not derivable** — radio only |
| EPM battery voltage | ✓ (11-bit, 0–17 V @ 8.3 mV; **all-ones code = unavailable**) | ✓/LP (same 11-bit field, same sentinel) | – | ✓ (raw mV, absent = unavailable) | logged |
| EPM rail currents ×6 (sys 3v3/5v, per 3v3/5v/9v/12v) | ✓ (7-bit each, 0–5 A @ 39.4 mA; **all-ones code = unavailable**) | – | – | ✓ (raw mA, absent = unavailable) | logged |
| SEM actuator steps ×3 | ✓ (10-bit each, full u16 @ 64.1 steps; **all-ones code = unavailable**) | – | – | ✓ (raw steps, absent = unavailable) | logged |
| SEM load cells ×3 (fracture load, cN) | – | – | – | ✓ (raw cN, signed, absent = unavailable) | **SD only** — the downlink has no room, see below |
| experiment flags ×7 per channel (fractured, finished, fault, homed, closure confirmed, enabled, monitoring) | – | – | – | ✓ (packed word, expanded to 21 CSV columns) | **SD only** — the downlink has no room, see below |
| payload arm-sequence bits ×3 (`arm_seq_running` / `_complete` / `_fault`) | – | – | – | ✓ (inside `payload_sdrm_custom_status`, bits 8–10) | logged raw; not yet on the downlink |
| VL health flags (imu_ok, sd_ok, …) | – | – | – | – | CAN node status only; not persisted |

Takeaways:

- GPS UTC time is logged at full rate (absolute time for video / GCM /
  redundant-FC correlation), and amp state + servo temp survive an RF-link
  drop via the slow record.
- The payload stack telemetry is on SD as well: the slow record carries the
  raw mV / mA / step / cN values, with the CAN `0xFFFF` already decoded into an
  `Option`, so an RF-link drop does not lose EPM rails or actuator positions.
  The downlink copies are quantized; the SD copies are exact.
- **The payload's experiment data is SD-only.** `CustomPayloadStatusMessage`
  grew from 20 B to 30 B in 2026-08 to carry three fracture load cells and a
  packed 21-bit experiment flag word, because the payload now runs its
  experiments autonomously off the flight stage and nothing on the bus could
  say whether that sequence was working. All of it reaches the slow record;
  none of it reaches the downlink, because the packet has four spare bits and
  the load cells alone would need far more (see the packet's size note below).
  The three `arm_seq_*` bits added in the same revision ride in the SDRM's
  existing 11-bit custom status, so they are already on SD at 10 Hz — they are
  the cheapest of the three to put on the downlink later, at three bits.
  The 30-byte length is a hard cut, not an extension: `FixedLenSerializable` is
  exact-length, so a 20-byte type 35 does not decode as a short 30-byte one.
  Payload and avionics move together or the payload goes dark.
- The downlink packet is 38 B — 48 B on air, after the type byte and
  reed-solomon ecc (`(n+1) + (n+1)/4`). The symbol count steps at 50 / 55 / 60
  bytes on air, so **38 B is the last size that still fits the current symbol
  count**: 39 B is 50 B on air, exactly on the first step. The four spare bits
  are padding inside the last byte, not budget,
  and there is no headroom left to add a field without paying for more air time
  inside the 2 s telemetry period. Time-on-air is lower than the 1642 ms quoted
  for the old 41 B packet and should be re-measured.
  The payload stack owns 93 of the 300 used bits (31%): 8 for the stack flags,
  11 for the EPM battery, 42 for the six rails, 30 for the three actuators,
  2 for SDRM liveness. Every other CAN node together costs 40 — AMP 21, Icarus
  17 (two liveness bits, `icarus_status_valid`, extension and servo temp),
  OZYS 2.
- The EPM battery is the one payload reading that is **also on the low-power
  packet**, which is where it matters most: low power is the mode the stack
  sits in for the hours between power-up and launch, so until now the only way
  to ask "is the payload still alive on the rail" was to arm the rocket. It
  cost the low-power packet two bytes — it had one spare bit and the field is
  11 wide, so 11 B became 13 B (98 bits, 6 spare). At a 5 s cadence that is
  affordable in a way it would not be on the 2 s `TelemetryPacket`, which is
  already at the last size that fits its symbol count. The factory, the
  all-ones sentinel and the codec live in `vlp::packets` rather than in
  `telemetry`, alongside `shared_battery_v`'s, so the voltage cannot decode one
  way on the pad and another way once armed. The six rail currents and three
  actuator step counts stay armed-only: they are flight diagnostics, and the
  battery is the one number that decides whether the stack survives the wait.
  The payload's status stream gets its own staleness receipt in both modes, so
  a payload that browns out mid-wait goes `n/a` within 5 s instead of
  downlinking the healthy voltage it had when it stopped talking.
- Payload readings still have **no validity bit on the downlink**, but they no
  longer read as zero: `epm_batt_v`, the six rail currents and the three
  actuator step counts each spend their **all-ones code** on "the payload could
  not take this reading", and the getters decode that code back to `None`. The
  sentinel moved off 0 deliberately — 0 is a real reading for all three. A
  switched rail that is off genuinely draws ~0 mA (the normal state whenever
  `payload_epm_rails_on` is false), an actuator at its home position genuinely
  sits at step 0, and 0.0 V on the EPM bus is a collapsed or disconnected pack
  the ground needs to see. Giving up the top code instead costs one quantum of
  headroom at full scale, which is headroom nothing real ever reaches: real
  values are clamped one code below full scale so they can never collide with
  the sentinel, capping at 16.992 V, 4961 mA and 65471 steps respectively. This
  is also why the EPM battery factory starts at 0 V rather than 11 V — an 11 V
  floor would have decoded a collapsed bus as a plausible low pack.
- Every CAN node's heartbeat is now on SD in full: uptime, health, mode and
  the 11-bit custom status, at 10 Hz, for AMP / Icarus / OZYS / payload SDRM.
  The downlink still compresses each to two bits because it has no room, so
  the log is the only place a mid-flight reboot or health change is
  recoverable — a reboot is `uptime_s` stepping backwards, which the packet's
  derived `uptime_s < 5` bit misses entirely if it lands between two 2 s
  packets. There is one OZYS on the rocket this year, so it is addressed by
  node type like every other node rather than by a hardcoded node ID.
  A 1 Hz CAN-health snapshot record on SD would close the remainder.
- The commanded extension is not always the MPC's output: if the MPC never
  asks for full extension the whole way up (what an undershoot looks like),
  the firmware forces 100% once vertical velocity drops below Mach 0.1, so
  that every flight ends with in-flight evidence the brakes actuate. The
  forced stretch is roughly the last 3.5 s before apogee, ending when the
  airbrakes estimator is retired and the brakes are commanded to 0. It is
  marked in the log by `air_brakes_validation_deploy`, and confirmed by
  `mpc_predicted_apogee_asl` going empty over the same rows — the command is no
  longer the MPC's, so there is no prediction that describes it. Read the tail
  of the commanded column as a servo test, not as MPC intent. It is not on the
  downlink; the RTT line `forcing 100% for validation` is the live marker.
- The airbrakes **commanded and actual extension are on the fast record**
  (v19), and the rest of the actuation — servo temperature, MPC prediction and
  target — is still on the slow one. Neither extension moves faster than
  100 Hz: the control loop commands at 10 Hz, and Icarus measures the servo
  angle every cycle of its 100 Hz loop. They are logged at 427 Hz for their
  EDGES, not their values, so a commanded step and the extension that follows
  it are rows ~2.3 ms apart and can be read as a step response. On the slow
  record each edge was quantised by ~100 ms and the reported extension was up
  to another ~100 ms stale behind that, which is the same order as the servo
  travel being measured — the pair was never a step response.
  Icarus used to send one report in ten (it was measuring at 100 Hz and
  throwing 9 away); it now sends every one, and reads the servo temperature on
  every tenth cycle, repeating it in the reports between. The `commanded`
  column still steps at 10 Hz — that is the control loop, not the log.
- **The drag check concludes on the sample it votes on.** It had to hold
  continuously for 1 s until v19, on the theory that a sustain rejects
  flicker at the threshold crossing — measured on the LC'25 replay there is
  no flicker: the low-passed airspeed crosses once and the margin grows
  monotonically (+1.03, +1.13, +1.21, +1.44 m/s over the four samples around
  it). What the second actually bought was ~0.04 Mach of margin, and only
  where nothing else was binding; the unsafe direction — a `Cd*A/m` too large,
  which reads the airspeed low and votes early — is held by the inertial Mach
  test at the birth site, which is a pure accelerometer integration and so
  cannot be fooled by a wrong drag model. At Cd +30% on LC'25 the check votes
  at true Mach 0.857 and that test holds the birth at 0.787, with the sustain
  or without it. What it cost was ~1 s of control window on every flight where
  the drag model was right.

  `earliest_subsonic_after_ignition_us` moved 17.50 s → 17.70 s to pay for it:
  that floor is now the whole of the timing guarantee, and the O3400 crosses
  Mach 0.8 at 17.56 s.
- **And the drag check is the whole of the lockout exit.** A second opinion —
  the dead reckoner's own vertical velocity against the same ceiling, tested
  once at the birth site — vetoed a vote the drag model got wrong until v19.
  It shared no input with that model, which is what made it worth having and
  also what made it weak: it is raw integration since ignition, so it reads
  24 m/s low on the clipped-accelerometer replay (the unsafe direction) and
  holds only ~8 m/s of margin on a clean flight. `max_open_mach` is now read
  as an approximation with about ±0.05 Mach either side rather than as a hard
  edge, which is what the removal costs: at the flown 0.8 ceiling the birth
  does not move at all (Mach 0.772), lower ceilings land a hair over instead
  of a hair under (0.610 at 0.6, 0.701 at 0.7), and a `Cd*A/m` a third too
  large births at 0.857 instead of 0.787.
  `airbrakes_subsonic_drag` went with it: with nothing left to defer the vote,
  the vote and the birth are one row, and the birth is already logged.
- The airbrakes estimator is **retired at apogee**, not merely gated: the
  wrapper drops it on the first of (its own vertical velocity <= 0), (its own
  tilt past the horizon), or (the deployment estimator calling apogee). The
  RTT line `retiring airbrakes estimator (...)` names which one fired, which
  is the only place that attribution is recorded — the whole `airbrakes` group
  just goes absent. On the Void Lake replay it is the velocity clause, ~2 s
  before the smoothed baro apogee.
- **The deployment estimator logs its baro innovation gate, per sample; the
  airbrakes estimator has no gate to log.** A **run** of `DEPLOYMENT_BARO_GATE_REJECT`
  is one episode — an ejection transient, or a port the shock front disturbed —
  and the filter rode it out on prediction alone. A `DEPLOYMENT_BARO_RESYNC`
  bit on the last row of a run is the opposite verdict: the run lasted long
  enough that the filter, not the sensor, was judged wrong, and altitude
  snapped to the baro. A run without a resync is the gate doing its job; a run
  ending in one means altitude is discontinuous across that row. Both bits report
  the in-flight KF's gate and nothing else, so they read 0 wherever no KF is
  fusing: through Mach lockout, the same window in which its altitude and
  velocity are absent, and on the pad. (The pad altitude reference had a gate
  of its own until v19 and these bits carried its verdict while on the pad; it
  is a plain mean over a 1 s window now, which rejects nothing.)

  The airbrakes filter had the same pair until v19 and lost both along with the
  gate itself. It is born subsonic and after burnout and retired at apogee, so
  its entire life is the one window of the flight with no shock front ahead of
  the static ports and no charge fired behind them: neither thing a gate exists
  to catch can reach it. What it costs is one bad sample moving altitude by the
  Kalman gain instead of being refused — about half a spike, at birth-time
  covariance. The deployment filter flies pad to landing through both hazards
  and keeps its gate.
- A `record_count` step other than +1 is the **only** signal that samples were
  lost, and every path now consumes a sequence number: the logger falling
  behind the sensor stream, a full SD queue, and an SD card reported offline.
  Before, the last two were invisible — the log read as continuous straight
  through an outage. Timestamps gap too, but only `record_count` distinguishes
  "samples lost" from "the sensor stream itself stalled".
- These bits are **exact, not sampled**. The estimator loop takes them in the
  same critical section as its update and publishes them stamped with that
  sample's `timestamp_us`; the logger only accepts a sample whose stamp equals
  the record's, so a deployment resync — which happens on exactly one sample —
  can be neither missed nor attributed to a neighbouring row. Rows before the
  estimator's first update carry no estimator group at all rather than a
  nearby tick's numbers, and the flag bits go absent with it: a `false` there
  would be a claim about a sample nothing measured.
- **CSV export writes absence as an empty cell** — never `0`, `NaN` or `65535`,
  each of which sits inside the range of values its column legitimately holds.
  Because the data column now says it, the five `*_valid` columns are gone
  (`imu_valid`, `mag_valid`, `gps_fix`, `gps_alt_valid`, `battery_valid`) and
  the header is **80 columns**, down from 85. No column was renamed or
  reordered. Rows logged before the first slow record arrives now have
  genuinely empty slow columns rather than fabricated zeros — including
  `temperature` and `num_sats`, which the slow record itself always carries but
  which no snapshot yet exists to supply.
- There is **one record per sensor sample and nothing else**. The logger blocks
  on the sensor stream with no timeout, so it never writes a filler row: a
  transient bus error costs one skipped tick (~2.4 ms), and anything longer is
  a dead sensor, which ends the log rather than padding it. `sequence` still
  gaps for samples lost between the sensor and the card.
- The packet's satellite count is 5 bits and **saturates at 31** rather than
  wrapping. It has to: packed_struct truncates, so a 32-satellite fix would
  have downlinked as 0 — exactly the reading that means "no fix, do not fly".
  31 on the downlink means "31 or more"; the SD log and the low-power / landed
  packets carry the full count.
- The DOPs no longer need a validity bit to be read safely. They used to log as
  **0.0 when the GPS reported none, which is not a sentinel but the ideal
  value**, so "no DOP reported" read as "geometry as good as it gets". They are
  `Option<f32>` now and blank when the GPS said nothing, so the column can be
  believed on its own. `lat_lon` and `gps_altitude_asl` are separate `Option`s
  for the same reason: a fix can carry a position without an altitude.
- `calibration_complete()` (airbrakes pad calibration, a launch-readiness
  condition) is on the fast record as `AIRBRAKES_PAD_CALIBRATED`, but live it
  is visible only on RTT — it has no CAN/telemetry slot. The
  natural home is a VLCustomStatus bit on the CAN heartbeat, relayed by the
  ground station as a GO/NO-GO light.
