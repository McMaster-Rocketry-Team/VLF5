# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Storage format v8. Updated 2026-08-16.

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding: `rocket-cli download-file <out.csv>`
(one CSV row per fast record, latest slow snapshot merged in). Storage format
v8 is **not** backward compatible: logs written by older firmware are reported
as unsupported by rocket-cli, and the firmware starts a fresh log over them on
the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz (per IMU data-ready edge; placeholder at 50 Hz during sensor-stream stalls) | 105 B | sequence, timestamp_us, **unix_time_us** (GPS-disciplined, 0 until the clock locks), accel ×3, gyro ×3, baro temperature + pressure, mag ×3 (100 Hz source, resampled), deployment KF altitude ASL + vertical velocity, airbrakes estimator altitude ASL + vertical velocity + tilt, ab_flags (lockout-exit drag check, burnout latch, filter-born, apogee latch), flight stage (`RocketState` mirror, Mach lockout folded into `Ascent`), **pyro flags** (continuity / fire / short — fire bits double as the chutes' `deployed`, now at ±2.3 ms), **airbrakes commanded + actual extension**, valid bitmask |
| **Slow** (tag 0x02) | 10 Hz | 97 B | timestamp_us, battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), flight stage (redundant with fast), **airbrakes servo temp**, **amp online + output statuses + shared battery voltage**, **payload EPM battery mV + six rail currents mA + three SEM actuator step counts** (2 Hz source, `0xFFFF` = unavailable), valid bitmask |

Throughput ≈ 46 kB/s ≈ 164 MB/hour; capacity is a non-issue.

NaN means "no source is producing this", uniformly: the KF fields until the
armed-mode estimator produces its first sample (a few ms after arming); the ab
fields until their piece of the airbrakes estimator is alive (tilt from
ignition, altitude/velocity from the vertical filter's birth) **and again from
apogee onward**, when the airbrakes estimator is retired — dropped outright, so
the whole descent has no ab columns and `ab_flags` reads 0; the airbrakes
commanded extension until the firmware commands one; and the airbrakes actual
extension and servo temp until Icarus first reports. The last of those is the
one worth remembering when reading a log: a silent or offline Icarus shows as
NaN, never as 0.0, so it cannot be mistaken for stowed brakes at 0 C. There are
no per-field validity bits for any of this — the value says it.
`flight_stage` gives stage transitions at ~2.3 ms
resolution, and the per-sample drag-check and burnout bits reconstruct the lockout-exit
table post-flight; the pyro fire bits get the same resolution. The deployment KF columns are frozen
(stale) during the deployment estimator's Mach lockout — the one place the log
intentionally carries numbers the state machine refuses to expose to control
logic. `flight_stage` reads `Ascent` through that window (the lockout is folded
into `Ascent` on the wire), so the frozen stretch is identified by the KF
columns going flat rather than by the stage value; the `ab_*` columns are live
throughout and are what to read there.

## Radio telemetry (VLP downlink, LoRa)

| Packet | Size | When | Contents (summary) |
|---|---|---|---|
| `TelemetryPacket` | 43 B (341/344 bits — 3 spare) | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, KF altitude AGL + air speed (+ maxima), tilt, flight stage (4-bit `RocketState` mirror, Mach lockout folded into `Ascent`), drogue/main deployed bits, **ab altitude AGL + ab vertical velocity (signed, ±400 m/s @ ~1.6 m/s) + drag check + born + apogee**, **target apogee AGL**, amp status + 3 outputs + shared battery, Icarus status + airbrakes ext/temp, OzYS + SDRM status, payload stack status, **EPM battery + six rail currents + three SEM actuator step counts** |
| `LowPowerTelemetryPacket` | 11 B | every 5 s in LowPower + Demo | sats, gps_fixed, **lat/lon**, VL battery, amp online, shared battery, air temp |
| `LandedTelemetryPacket` | 12 B | every 5 s in Landed | lat/lon, sats, VL battery, amp status + outputs + shared battery |

Stage on the downlink: during the deployment estimator's Mach lockout the
packet reports the KF altitude and air speed as 0 (the state carries no
numbers) under stage `Ascent`, while the ab fields stay live — so a ground
display should prefer the ab fields during ascent rather than showing the flat
zero. FailedToReachMinApogee is reported as itself. The ground sees the
airbrakes estimator directly:
whether it's born, whether the drag check passes, its altitude/velocity, and the target
apogee it's steering toward.

## Full data map

Columns: **TM** = TelemetryPacket (2 s) · **LP/LD** = low-power / landed
packets (5 s) · **Fast** = SD @ ~427 Hz · **Slow** = SD @ 10 Hz.

| Value | TM 2s | LP/LD 5s | Fast | Slow | Derivable post-flight? |
|---|---|---|---|---|---|
| accel, gyro (3-axis) | – | – | ✓ | – | primary data |
| mag (3-axis) | – | – | ✓ (100 Hz source) | – | primary data |
| baro pressure | – | – | ✓ | – | primary data |
| baro/air temperature | ✓ | LP | ✓ | – | logged |
| deployment KF altitude ASL | ✓ (as AGL, coarse; 0 during the baro Mach lockout) | – | ✓ | – | yes — replay pressure through estimator |
| deployment KF vertical velocity | ✓ (as \|air_speed\|, 8-bit; 0 during the baro Mach lockout) | – | ✓ | – | yes — replay |
| ab estimator altitude | ✓ (as AGL, 13-bit; 0 after apogee) | – | ✓ (ASL, NaN until born / after apogee) | – | yes — replay IMU+baro through estimator |
| ab estimator vertical velocity | ✓ (signed 9-bit; 0 after apogee) | – | ✓ (NaN until born / after apogee) | – | yes — replay |
| ab tilt | ✓ (8-bit; 0 after apogee) | – | ✓ (NaN before ignition / after apogee) | – | yes — replay / offline attitude |
| ab drag check, born, apogee latch | ✓ (3 bits; all 0 after apogee) | – | ✓ (ab_flags bits) | – | yes — replay |
| ab burnout latch | – | – | ✓ (`AB_BURNOUT`) | – | SD only; the packet has 3 spare bits if it's wanted live |
| ab pad calibration complete | – | – | – | – | **nowhere on the wire** — RTT log line only; candidate for a VLCustomStatus bit |
| flight stage (incl. FailedToReachMinApogee; Mach lockout folded into `Ascent`) | ✓ (4-bit) | – | ✓ full rate | ✓ (redundant) | logged |
| drogue/main deployed | ✓ (bits) | – | ✓ (pyro fire flags) | – | logged |
| max altitude / max airspeed | ✓ | – | – | – | yes — max() over fast records |
| target apogee | ✓ (13-bit AGL) | – | – | – | also in SD config block |
| KF innovation / gate rejections | – | – | – | – | yes — replay pressure |
| launch pad altitude | – | – | – | – | yes — KF altitude while on pad |
| GPS lat/lon, sat count | ✓ | ✓ all three | – | ✓ | logged |
| GPS altitude, DOPs | – | – | – | ✓ | logged |
| GPS UTC time | – | – | ✓ (unix µs, full rate) | – | logged — absolute time for video/GCM correlation |
| VL battery voltage | ✓ | ✓ | – | ✓ | logged |
| pyro continuity (main/drogue) | ✓ | – | ✓ | – | logged |
| pyro fire outputs, short-circuit | – | – | ✓ (±2.3 ms) | – | logged |
| airbrakes commanded/actual ext. | ✓ (5-bit) | – | ✓ full rate (NaN until commanded / until Icarus reports) | – | logged — but see the validation-deploy note below |
| airbrakes servo temp | ✓ | – | – | ✓ (NaN until Icarus reports) | logged |
| MPC predicted apogee | – | – | – | – | approx — re-run MPC on logged ab state |
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
- The downlink packet is 43 B (55 B on air after the type byte and
  reed-solomon ecc): 1774 ms time-on-air at SF12 / 250 kHz / CR 4:8, inside
  the 2 s period. Time-on-air steps at 50 / 55 / 60 bytes on air, so trimming
  the packet to 39 B would buy back 132 ms.
- Still radio-only: payload stack status, Icarus/OzYS/SDRM health.
  A 1 Hz CAN-health snapshot record on SD would close the remainder.
- The commanded extension is not always the MPC's output: if the MPC never
  asks for full extension the whole way up (what an undershoot looks like),
  the firmware forces 100% once vertical velocity drops below Mach 0.1, so
  that every flight ends with in-flight evidence the brakes actuate. The
  forced stretch is roughly the last 3.5 s before apogee, ending when the
  airbrakes estimator is retired and the brakes are commanded to 0. It is
  identified in the log by commanded extension stepping straight to 1.0 at
  low `ab_*` velocity; there is no flag for it, and the RTT line
  `forcing 100% for validation` is the only direct marker. Read the tail of
  the commanded column as a servo test, not as MPC intent.
- The airbrakes estimator is **retired at apogee**, not merely gated: the
  wrapper drops it on the first of (its own vertical velocity <= 0), (its own
  tilt past the horizon), or (the deployment estimator calling apogee). The
  RTT line `retiring airbrakes estimator (...)` names which one fired, which
  is the only place that attribution is recorded — the `ab_*` columns just go
  NaN. On the Void Lake replay it is the velocity clause, ~2 s before the
  smoothed baro apogee.
- `calibration_complete()` (airbrakes pad calibration, a launch-readiness
  condition) is visible only on RTT — it has no CAN/telemetry slot. The
  natural home is a VLCustomStatus bit on the CAN heartbeat, relayed by the
  ground station as a GO/NO-GO light.
