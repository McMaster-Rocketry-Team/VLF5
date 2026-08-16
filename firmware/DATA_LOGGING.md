# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Storage format v12. Updated 2026-08-16.

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
padding, which is why the v12 records are half again as large as v11's. That is
the deliberate price of a log that cannot mistake a reading for its own absence.

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding:
`rocket-cli download-flight-log <out.csv>` (one CSV row per fast record, latest
slow snapshot merged in). Storage format v12 is **not** backward compatible:
logs written by older firmware are reported as unsupported by rocket-cli, and
the firmware starts a fresh log over them on the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz — exactly one record per published sensor sample, no filler | 145 B (144 B body + tag), 3 per block | sequence, timestamp_us, **unix_time_us** (GPS-disciplined, absent until the clock locks), `imu` (accel ×3 + gyro ×3, one group — they come from the same read), baro pressure, mag ×3 (100 Hz source, resampled), `deployment` (`kf_altitude_asl` + `kf_vertical_velocity` + `flags`: baro innovation-gate reject + resync, per sample), `airbrakes` (`kf_altitude_asl` + `kf_vertical_velocity` + `kf_tilt_deg` + `flags`: lockout-exit drag check, burnout latch, filter-born, apogee latch, baro gate reject + resync), flight stage (`RocketState` mirror, Mach lockout folded into `Ascent`, logged only here), **pyro flags** (continuity / fire / short, at ±2.3 ms) |
| **Slow** (tag 0x02) | 10 Hz | 241 B (240 B body + tag), 2 per block | timestamp_us, **baro temperature** (~13 Hz source), battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), `air_brakes` (**commanded + actual extension**, **validation-deploy flag**, **servo temp**, **MPC predicted apogee AGL**), **full `NodeStatus` for AMP / Icarus / OzYS / payload SDRM** (uptime, health, mode, custom status), `amp` (**output statuses + shared battery voltage**), `payload` (**EPM battery mV + six rail currents mA + three SEM actuator step counts**, 2 Hz source) |

Throughput ≈ 64 kB/s of record payload (427 Hz × 145 B + 10 Hz × 241 B),
≈ 70 kB/s of actual block writes once block padding is counted, ≈ 250 MB/hour.
That is up from v11's ≈ 43 kB/s / 154 MB/hour, and capacity remains a
non-issue.

Absence means "no source is producing this", uniformly, and every one of these
reads as an empty cell in the CSV rather than as a number:
the `deployment` group until the armed-mode estimator produces its first sample
(a few ms after arming); the `airbrakes` group until its piece of the airbrakes
estimator is alive (tilt from ignition, altitude/velocity from the vertical
filter's birth) **and again from apogee onward**, when the airbrakes estimator
is retired — dropped outright, so the whole descent has no `airbrakes_*`
columns at all, flags included; the airbrakes commanded extension until the
firmware commands one; and the airbrakes actual extension and servo temp until
Icarus first reports. The last of those is the one worth remembering when
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
  all, so the gate bits are absent too.
- A node's outer `Option` being `None` means **never heard from at all**;
  `NodeStatusRecord::online == false` means the node spoke and then went quiet
  for 5 s, and the rest of its fields describe its last heartbeat. The old
  fabricated `NodeStatusRecord::offline()` is gone, so "never on the bus" no
  longer reads as "booted, then dropped".

`flight_stage` gives stage transitions at ~2.3 ms resolution, and the
per-sample drag-check and burnout bits reconstruct the lockout-exit table
post-flight; the pyro fire bits get the same resolution. Through the Mach
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
| `TelemetryPacket` | 38 B (299/304 bits — 5 spare) | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, deployment altitude AGL + max + vertical velocity (signed, −400..1050 m/s @ ~1.4 m/s), airbrakes tilt, flight stage (3-bit `RocketState` mirror, Mach lockout folded into `Ascent`), **airbrakes born**, **MPC predicted apogee AGL**, **target apogee AGL**, amp status + 3 outputs + shared battery, Icarus status + airbrakes ext/temp, OzYS + SDRM status, payload stack status, **EPM battery + six rail currents + three SEM actuator step counts** |
| `LowPowerTelemetryPacket` | 11 B (87 bits — 1 spare) | every 5 s in LowPower + Demo | sats, gps_fixed, **lat/lon**, VL battery, amp online, shared battery, air temp |
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
| airbrakes born | ✓ (1 bit; 0 after apogee) | – | ✓ (`AIRBRAKES_BARO_TRUSTED`) | – | yes — replay |
| airbrakes drag check, apogee latch | – | – | ✓ (`airbrakes.flags` bits) | – | yes — replay |
| airbrakes burnout latch | – | – | ✓ (`AIRBRAKES_BURNOUT`) | – | SD only; the packet has 5 spare bits, but see the air-time note below |
| airbrakes pad calibration complete | – | – | – | – | **nowhere on the wire** — RTT log line only; candidate for a VLCustomStatus bit |
| `flight_stage` (incl. FailedToReachMinApogee; Mach lockout folded into `Ascent`) | ✓ (3-bit) | – | ✓ full rate | – | logged |
| drogue/main deployed | – | – | ✓ (pyro fire flags) | – | read the stage transition, or the pyro fire bits for the GPIO edge |
| `target_apogee_agl` | ✓ (14-bit) | – | – | – | also in SD config block |
| deployment baro gate reject, resync | – | – | ✓ (`deployment.flags` bits, exact per sample) | – | logged |
| airbrakes baro gate reject, resync | – | – | ✓ (`airbrakes.flags` bits, exact per sample) | – | logged |
| KF innovation magnitude | – | – | – | – | yes — replay pressure |
| launch pad altitude | – | – | – | – | yes — `deployment_kf_altitude_asl` while on pad |
| GPS `lat_lon` | ✓ (23-bit lat / 24-bit lon, ~2.4 m, `lat_lon_valid`) | ✓ all three | – | ✓ (f64, exact; absent until a fix) | logged |
| `num_of_fix_satellites` | ✓ (5-bit, **saturates at 31**) | ✓ (full u8) | – | ✓ (full u8; 0 is a real reading, so never absent) | logged |
| `gps_altitude_asl` | – | – | – | ✓ (absent until the fix carries one) | logged |
| `hdop` / `vdop` / `pdop` | – | – | – | ✓ (absent when the GPS did not report one) | logged — see below |
| GPS UTC time | – | – | ✓ (unix µs, full rate; absent until the clock locks) | – | logged — absolute time for video/GCM correlation |
| VL battery voltage | ✓ | ✓ | – | ✓ (absent until the ADC reports) | logged |
| pyro continuity (main/drogue) | ✓ | – | ✓ | – | logged |
| pyro fire outputs, short-circuit | – | – | ✓ (±2.3 ms) | – | logged |
| `air_brakes_commanded_extension` | ✓ (5-bit, always sent) | – | – | ✓ (absent until commanded) | logged |
| `air_brakes_actual_extension` | ✓ (5-bit, `icarus_status_valid`) | – | – | ✓ (absent until Icarus reports) | logged |
| `air_brakes_validation_deploy` | – | – | – | ✓ (a flag, not an absence) | logged |
| `air_brakes_servo_temp` | ✓ (9-bit, shares `icarus_status_valid`) | – | – | ✓ (absent until Icarus reports) | logged |
| Icarus servo current | – | – | – | – | **nowhere** — the field was dropped from `IcarusStatusMessage`; the servo does not measure current |
| MPC predicted apogee | ✓ (14-bit AGL, `mpc_predicted_apogee_valid`) | – | – | ✓ (absent while the MPC is not running) | logged |
| amp: online, outputs ×3, shared battery | ✓ | ✓/LD | – | ✓ (whole `amp` group absent until the first `AmpStatusMessage`) | logged |
| Icarus / OzYS / SDRM online + rebooted | ✓ (2 bits each) | – | – | ✓ (full `NodeStatus`; absent = never heard from) | logged |
| node uptime, health, mode (all four) | – | – | – | ✓ | logged |
| OzYS disk usage, gauge-connected, `sd_ok` | – | – | – | ✓ (`ozys_custom_status`, decode with `OzysCustomStatus`) | logged |
| OzYS strain measurements | – | – | – | – | **nowhere** — OzYS logs to its own SD |
| payload stack flags ×8 (`payload_epm_alive` etc.) | ✓ (1 bit each) | – | – | – | **not derivable** — radio only |
| EPM battery voltage | ✓ (11-bit, 0–17 V @ 8.3 mV; **all-ones code = unavailable**) | – | – | ✓ (raw mV, absent = unavailable) | logged |
| EPM rail currents ×6 (sys 3v3/5v, per 3v3/5v/9v/12v) | ✓ (7-bit each, 0–5 A @ 39.4 mA; **all-ones code = unavailable**) | – | – | ✓ (raw mA, absent = unavailable) | logged |
| SEM actuator steps ×3 | ✓ (10-bit each, full u16 @ 64.1 steps; **all-ones code = unavailable**) | – | – | ✓ (raw steps, absent = unavailable) | logged |
| VL health flags (imu_ok, sd_ok, …) | – | – | – | – | CAN node status only; not persisted |

Takeaways:

- GPS UTC time is logged at full rate (absolute time for video / GCM /
  redundant-FC correlation), and amp state + servo temp survive an RF-link
  drop via the slow record.
- The payload stack telemetry is on SD as well: the slow record carries the
  raw mV / mA / step values, with the CAN `0xFFFF` already decoded into an
  `Option`, so an RF-link drop does not lose EPM rails or actuator positions.
  The downlink copies are quantized; the SD copies are exact.
- The downlink packet is 38 B — 48 B on air, after the type byte and
  reed-solomon ecc (`(n+1) + (n+1)/4`). The symbol count steps at 50 / 55 / 60
  bytes on air, so **38 B is the last size that still fits the current symbol
  count**: the five spare bits are padding inside the last byte, not budget,
  and there is no headroom left to add a field without paying for more air time
  inside the 2 s telemetry period. Time-on-air is lower than the 1642 ms quoted
  for the old 41 B packet and should be re-measured.
  The payload stack owns 93 of the 299 used bits (31%): 8 for the stack flags,
  11 for the EPM battery, 42 for the six rails, 30 for the three actuators,
  2 for SDRM liveness. Every other CAN node together costs 40 — AMP 21, Icarus
  17 (two liveness bits, `icarus_status_valid`, extension and servo temp),
  OzYS 2.
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
  the 11-bit custom status, at 10 Hz, for AMP / Icarus / OzYS / payload SDRM.
  The downlink still compresses each to two bits because it has no room, so
  the log is the only place a mid-flight reboot or health change is
  recoverable — a reboot is `uptime_s` stepping backwards, which the packet's
  derived `uptime_s < 5` bit misses entirely if it lands between two 2 s
  packets. There is one OzYS on the rocket this year, so it is addressed by
  node type like every other node rather than by a hardcoded node ID.
  A 1 Hz CAN-health snapshot record on SD would close the remainder.
- The commanded extension is not always the MPC's output: if the MPC never
  asks for full extension the whole way up (what an undershoot looks like),
  the firmware forces 100% once vertical velocity drops below Mach 0.1, so
  that every flight ends with in-flight evidence the brakes actuate. The
  forced stretch is roughly the last 3.5 s before apogee, ending when the
  airbrakes estimator is retired and the brakes are commanded to 0. It is
  marked in the log by `air_brakes_validation_deploy`, and confirmed by
  `mpc_predicted_apogee_agl` going empty over the same rows — the command is no
  longer the MPC's, so there is no prediction that describes it. Read the tail
  of the commanded column as a servo test, not as MPC intent. It is not on the
  downlink; the RTT line `forcing 100% for validation` is the live marker.
- Everything about the airbrakes *actuation* lives on the slow record, not
  the fast one: the control loop runs at 10 Hz and Icarus reports at 10 Hz, so
  the fast record was storing each value ~42 times over. It also stops the log
  implying a precision it never had — the Icarus-reported extension is up to
  100 ms older than the row it used to sit on, so a commanded/actual pair was
  never a step response. Resolving servo slew would need Icarus to report
  faster (it measures the angle every cycle at 100 Hz and throws 9 of 10 away),
  not the log to sample faster.
- The airbrakes estimator is **retired at apogee**, not merely gated: the
  wrapper drops it on the first of (its own vertical velocity <= 0), (its own
  tilt past the horizon), or (the deployment estimator calling apogee). The
  RTT line `retiring airbrakes estimator (...)` names which one fired, which
  is the only place that attribution is recorded — the whole `airbrakes` group
  just goes absent. On the Void Lake replay it is the velocity clause, ~2 s
  before the smoothed baro apogee.
- Both estimators log their baro innovation gate, per sample. A **run** of
  reject bits is one episode — an ejection transient, or a port the shock
  front disturbed — and the filter rode it out on prediction alone. A
  **resync** bit (`DEPLOYMENT_BARO_RESYNC` / `AIRBRAKES_BARO_RESYNC`) on the
  last row of a run is the opposite verdict: the run lasted long enough that
  the filter, not the sensor, was judged wrong, and altitude snapped to the
  baro. A run without a resync is the gate doing its job; a run ending in one
  means altitude is discontinuous across that row. Both deployment bits read 0
  through Mach lockout, where the KF is frozen and nothing is fused — the same
  window in which its altitude and velocity are absent.
- A `record_count` step other than +1 is the **only** signal that samples were
  lost, and every path now consumes a sequence number: the logger falling
  behind the sensor stream, a full SD queue, and an SD card reported offline.
  Before, the last two were invisible — the log read as continuous straight
  through an outage. Timestamps gap too, but only `record_count` distinguishes
  "samples lost" from "the sensor stream itself stalled".
- These bits are **exact, not sampled**. The estimator loop takes them in the
  same critical section as its update and publishes them stamped with that
  sample's `timestamp_us`; the logger only accepts a sample whose stamp equals
  the record's, so a resync — which happens on exactly one sample — can be
  neither missed nor attributed to a neighbouring row. Rows before the
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
  condition) is visible only on RTT — it has no CAN/telemetry slot. The
  natural home is a VLCustomStatus bit on the CAN heartbeat, relayed by the
  ground station as a GO/NO-GO light.
