# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Storage format v10. Updated 2026-08-16.

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding: `rocket-cli download-file <out.csv>`
(one CSV row per fast record, latest slow snapshot merged in). Storage format
v10 is **not** backward compatible: logs written by older firmware are reported
as unsupported by rocket-cli, and the firmware starts a fresh log over them on
the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz — exactly one record per published sensor sample, no filler | 97 B | sequence, timestamp_us, **unix_time_us** (GPS-disciplined, 0 until the clock locks), accel ×3, gyro ×3, baro pressure, mag ×3 (100 Hz source, resampled), `deployment_kf_altitude_asl` + `deployment_kf_vertical_velocity` + `deployment_flags` (baro innovation-gate reject + resync, per sample), `airbrakes_kf_altitude_asl` + `airbrakes_kf_vertical_velocity` + `airbrakes_kf_tilt_deg`, `airbrakes_flags` (lockout-exit drag check, burnout latch, filter-born, apogee latch, baro gate reject + resync), flight stage (`RocketState` mirror, Mach lockout folded into `Ascent`, logged only here), **pyro flags** (continuity / fire / short, at ±2.3 ms), valid bitmask |
| **Slow** (tag 0x02) | 10 Hz | 105 B | timestamp_us, **baro temperature** (~13 Hz source), battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), **airbrakes commanded + actual extension** + **validation-deploy flag** + **servo temp**, **MPC predicted apogee AGL**, **amp online + output statuses + shared battery voltage**, **payload EPM battery mV + six rail currents mA + three SEM actuator step counts** (2 Hz source, `0xFFFF` = unavailable), valid bitmask |

Throughput ≈ 42 kB/s ≈ 153 MB/hour; capacity is a non-issue.

NaN means "no source is producing this", uniformly: the `deployment_*` fields
until the armed-mode estimator produces its first sample (a few ms after
arming); the `airbrakes_*` fields until their piece of the airbrakes estimator
is alive (tilt from ignition, altitude/velocity from the vertical filter's
birth) **and again from apogee onward**, when the airbrakes estimator is
retired — dropped outright, so the whole descent has no `airbrakes_*` columns
and `airbrakes_flags` reads 0; the airbrakes
commanded extension until the firmware commands one; and the airbrakes actual
extension and servo temp until Icarus first reports. The last of those is the
one worth remembering when reading a log: a silent or offline Icarus shows as
NaN, never as 0.0, so it cannot be mistaken for stowed brakes at 0 C. There are
no per-field validity bits for any of this — the value says it. The one
exception is `air_brakes_validation_deploy`, which is a flag rather than a NaN:
a commanded 1.0 from the MPC and a commanded 1.0 from the validation deploy are
the same number, so absence-encoding cannot separate them.
`flight_stage` gives stage transitions at ~2.3 ms
resolution, and the per-sample drag-check and burnout bits reconstruct the lockout-exit
table post-flight; the pyro fire bits get the same resolution. The deployment KF columns are frozen
(stale) during the deployment estimator's Mach lockout — the one place the log
intentionally carries numbers the state machine refuses to expose to control
logic. `flight_stage` reads `Ascent` through that window (the lockout is folded
into `Ascent` on the wire), so the frozen stretch is identified by the
`deployment_*` columns going flat rather than by the stage value; the
`airbrakes_*` columns are live throughout and are what to read there.

## Radio telemetry (VLP downlink, LoRa)

| Packet | Size | When | Contents (summary) |
|---|---|---|---|
| `TelemetryPacket` | 41 B (325/328 bits — 3 spare) | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, deployment altitude AGL + max + vertical velocity (signed, −400..1050 m/s @ ~1.4 m/s), airbrakes tilt, flight stage (3-bit `RocketState` mirror, Mach lockout folded into `Ascent`), **airbrakes born**, **MPC predicted apogee AGL**, **target apogee AGL**, amp status + 3 outputs + shared battery, Icarus status + airbrakes ext/temp, OzYS + SDRM status, payload stack status, **EPM battery + six rail currents + three SEM actuator step counts** |
| `LowPowerTelemetryPacket` | 11 B | every 5 s in LowPower + Demo | sats, gps_fixed, **lat/lon**, VL battery, amp online, shared battery, air temp |
| `LandedTelemetryPacket` | 12 B | every 5 s in Landed | lat/lon, sats, VL battery, amp status + outputs + shared battery |

Stage on the downlink: during the deployment estimator's Mach lockout the
packet reports `deployment_kf_altitude_agl` and `deployment_kf_vertical_velocity`
as 0 (the state carries no numbers) under stage `Ascent`. Tilt is the only live
number on the downlink through that window; the SD log's `airbrakes_kf_*`
columns are what to read for the rest.
FailedToReachMinApogee is reported as itself. Vertical velocity is signed and
comes from the deployment estimator, so descent rate is live all the way down —
it is the airbrakes estimator that is retired at apogee, and only `airbrakes_born`
and the tilt it feeds are downlinked from it now. The pair worth watching on
ascent is `mpc_predicted_apogee_agl` against `target_apogee_agl`: while they
agree the brakes have authority, and the gap between them is the overshoot or
undershoot the MPC cannot fix.

## Full data map

Columns: **TM** = TelemetryPacket (2 s) · **LP/LD** = low-power / landed
packets (5 s) · **Fast** = SD @ ~427 Hz · **Slow** = SD @ 10 Hz.

| Value | TM 2s | LP/LD 5s | Fast | Slow | Derivable post-flight? |
|---|---|---|---|---|---|
| accel, gyro (3-axis) | – | – | ✓ | – | primary data |
| mag (3-axis) | – | – | ✓ (100 Hz source) | – | primary data |
| baro pressure | – | – | ✓ | – | primary data |
| `temperature` (MS5607 die) | – | – | – | ✓ | logged |
| `air_temperature` (same reading, downlinked) | ✓ | LP | – | – | yes — `temperature` in the slow record, unquantized |
| `deployment_kf_altitude_asl` | ✓ as `deployment_kf_altitude_agl` (14-bit; 0 during the baro Mach lockout) | – | ✓ | – | yes — replay pressure through estimator |
| `deployment_kf_vertical_velocity` | ✓ (signed 10-bit; 0 during the baro Mach lockout) | – | ✓ | – | yes — replay |
| `max_deployment_kf_altitude_agl` | ✓ (14-bit) | – | – | – | yes — max() over fast records |
| `airbrakes_kf_altitude_asl` | – | – | ✓ (ASL, NaN until born / after apogee) | – | yes — replay IMU+baro through estimator |
| `airbrakes_kf_vertical_velocity` | – | – | ✓ (NaN until born / after apogee) | – | yes — replay |
| `airbrakes_kf_tilt_deg` | ✓ (8-bit; 0 after apogee) | – | ✓ (NaN before ignition / after apogee) | – | yes — replay / offline attitude |
| airbrakes born | ✓ (1 bit; 0 after apogee) | – | ✓ (`AIRBRAKES_BARO_TRUSTED`) | – | yes — replay |
| airbrakes drag check, apogee latch | – | – | ✓ (`airbrakes_flags` bits) | – | yes — replay |
| airbrakes burnout latch | – | – | ✓ (`AIRBRAKES_BURNOUT`) | – | SD only; the packet has 3 spare bits if it's wanted live |
| airbrakes pad calibration complete | – | – | – | – | **nowhere on the wire** — RTT log line only; candidate for a VLCustomStatus bit |
| `flight_stage` (incl. FailedToReachMinApogee; Mach lockout folded into `Ascent`) | ✓ (3-bit) | – | ✓ full rate | – | logged |
| drogue/main deployed | – | – | ✓ (pyro fire flags) | – | read the stage transition, or the pyro fire bits for the GPIO edge |
| `target_apogee_agl` | ✓ (14-bit) | – | – | – | also in SD config block |
| deployment baro gate reject, resync | – | – | ✓ (`deployment_flags` bits, exact per sample) | – | logged |
| airbrakes baro gate reject, resync | – | – | ✓ (`airbrakes_flags` bits, exact per sample) | – | logged |
| KF innovation magnitude | – | – | – | – | yes — replay pressure |
| launch pad altitude | – | – | – | – | yes — `deployment_kf_altitude_asl` while on pad |
| GPS lat/lon, sat count | ✓ | ✓ all three | – | ✓ | logged |
| GPS altitude, DOPs | – | – | – | ✓ | logged |
| GPS UTC time | – | – | ✓ (unix µs, full rate) | – | logged — absolute time for video/GCM correlation |
| VL battery voltage | ✓ | ✓ | – | ✓ | logged |
| pyro continuity (main/drogue) | ✓ | – | ✓ | – | logged |
| pyro fire outputs, short-circuit | – | – | ✓ (±2.3 ms) | – | logged |
| `air_brakes_commanded_extension` | ✓ (5-bit) | – | – | ✓ (NaN until commanded) | logged |
| `air_brakes_actual_extension` | ✓ (5-bit) | – | – | ✓ (NaN until Icarus reports) | logged |
| `air_brakes_validation_deploy` | – | – | – | ✓ | logged |
| `air_brakes_servo_temp` | ✓ (9-bit) | – | – | ✓ (NaN until Icarus reports) | logged |
| Icarus servo current | – | – | – | – | **nowhere** — the field was dropped from `IcarusStatusMessage`; the servo does not measure current |
| MPC predicted apogee | ✓ (14-bit AGL) | – | – | ✓ (NaN while the MPC is not running) | logged |
| amp: online, outputs ×3, shared battery | ✓ | ✓/LD | – | ✓ | logged |
| Icarus / OzYS / SDRM online, uptime | ✓ | – | – | – | **not derivable** — radio only |
| payload stack status | ✓ (11-bit) | – | – | – | **not derivable** — radio only |
| EPM battery voltage | ✓ (1+10 bit, 11–17 V) | – | – | ✓ (raw mV) | logged |
| EPM rail currents ×6 (sys 3v3/5v, per 3v3/5v/9v/12v) | ✓ (1+10 bit each, 0–10.23 A @ 10 mA) | – | – | ✓ (raw mA) | logged |
| SEM actuator steps ×3 | ✓ (1+10 bit each, full u16 @ ~64 steps) | – | – | ✓ (raw steps) | logged |
| VL health flags (imu_ok, sd_ok, …) | – | – | – | – | CAN node status only; not persisted |

Takeaways:

- GPS UTC time is logged at full rate (absolute time for video / GCM /
  redundant-FC correlation), and amp state + servo temp survive an RF-link
  drop via the slow record.
- The payload stack telemetry is on SD as well: the slow record carries the
  raw mV / mA / step values (with the payload's own `0xFFFF` = unavailable
  sentinel), so an RF-link drop does not lose EPM rails or actuator positions.
  The downlink copies are quantized; the SD copies are exact.
- The downlink packet is 41 B (52 B on air after the type byte and
  reed-solomon ecc): 1642 ms time-on-air at SF12 / 250 kHz / CR 4:8, inside
  the 2 s period. Time-on-air steps at 50 / 55 / 60 bytes on air, so anything
  from 37 to 43 B costs exactly this much air time — the next saving is at
  36 B.
- Still radio-only: payload stack status, Icarus/OzYS/SDRM health.
  A 1 Hz CAN-health snapshot record on SD would close the remainder.
- The commanded extension is not always the MPC's output: if the MPC never
  asks for full extension the whole way up (what an undershoot looks like),
  the firmware forces 100% once vertical velocity drops below Mach 0.1, so
  that every flight ends with in-flight evidence the brakes actuate. The
  forced stretch is roughly the last 3.5 s before apogee, ending when the
  airbrakes estimator is retired and the brakes are commanded to 0. It is
  marked in the log by `air_brakes_validation_deploy`, and confirmed by
  `mpc_predicted_apogee_agl` going NaN over the same rows — the command is no
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
  is the only place that attribution is recorded — the `airbrakes_*` columns just go
  NaN. On the Void Lake replay it is the velocity clause, ~2 s before the
  smoothed baro apogee.
- Both estimators log their baro innovation gate, per sample. A **run** of
  reject bits is one episode — an ejection transient, or a port the shock
  front disturbed — and the filter rode it out on prediction alone. A
  **resync** bit (`DEPLOYMENT_BARO_RESYNC` / `AIRBRAKES_BARO_RESYNC`) on the
  last row of a run is the opposite verdict: the run lasted long enough that
  the filter, not the sensor, was judged wrong, and altitude snapped to the
  baro. A run without a resync is the gate doing its job; a run ending in one
  means altitude is discontinuous across that row. Both deployment bits read 0
  through Mach lockout, where the KF is frozen and nothing is fused.
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
  estimator's first update carry NaN estimator values and 0 flags rather than a
  nearby tick's numbers.
- The `valid` bitmask covers only what can genuinely be absent: `VALID_IMU`
  (the IMU is optional per sample), `VALID_MAG` (a 100 Hz source resampled to
  the fast rate), and the GPS/battery bits on the slow record. There is no baro
  bit — every published sample carries a baro reading, so pressure and
  temperature are always present.
- There is **one record per sensor sample and nothing else**. The logger blocks
  on the sensor stream with no timeout, so it never writes a filler row: a
  transient bus error costs one skipped tick (~2.4 ms), and anything longer is
  a dead sensor, which ends the log rather than padding it. `sequence` still
  gaps for samples lost between the sensor and the card.
- `calibration_complete()` (airbrakes pad calibration, a launch-readiness
  condition) is visible only on RTT — it has no CAN/telemetry slot. The
  natural home is a VLCustomStatus bit on the CAN heartbeat, relayed by the
  ground station as a GO/NO-GO light.
