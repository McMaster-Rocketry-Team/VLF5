# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Updated 2026-08-15 (storage format v8: payload EPM
rail currents and SEM actuator steps on both the downlink and SD; v7 put
airbrakes state on the downlink, UTC + pyro + extension at full rate, amp/servo
temp on SD, and took bulkheads and the coasting bit off the wire).

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding: `rocket-cli download-file <out.csv>`
(one CSV row per fast record, latest slow snapshot merged in). Storage format
v8 is **not** backward compatible: logs written by older firmware are reported
as unsupported by rocket-cli, and the firmware starts a fresh log over them on
the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz (per IMU data-ready edge; placeholder at 50 Hz during sensor-stream stalls) | 105 B | sequence, timestamp_us, **unix_time_us** (GPS-disciplined, 0 until the clock locks), accel ×3, gyro ×3, baro temperature + pressure, mag ×3 (100 Hz source, resampled), deployment KF altitude ASL + vertical velocity, airbrakes estimator altitude ASL + vertical velocity + tilt, ab_flags (3 lockout-exit vote bits, filter-born, apogee latch), flight stage (honest `RocketState` mirror), **pyro flags** (continuity / fire / short — fire bits double as the chutes' `deployed`, now at ±2.3 ms), **airbrakes commanded + actual extension**, valid bitmask |
| **Slow** (tag 0x02) | 10 Hz | 97 B | timestamp_us, battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), flight stage (redundant with fast), **airbrakes servo temp**, **amp online + output statuses + shared battery voltage**, **payload EPM battery mV + six rail currents mA + three SEM actuator step counts** (2 Hz source, `0xFFFF` = unavailable), valid bitmask |

Throughput ≈ 46 kB/s ≈ 164 MB/hour; capacity is a non-issue.

The KF fields are NaN until the armed-mode estimator produces its first sample
(a few ms after arming); the ab fields are NaN until their piece of the
airbrakes estimator is alive (tilt from ignition, altitude/velocity from the
vertical filter's birth). `flight_stage` gives stage transitions at ~2.3 ms
resolution, and the per-sample vote bits reconstruct the lockout-exit truth
table post-flight; since v7 the pyro fire bits get the same resolution
(previously ±100 ms via the slow record). The deployment KF columns are frozen
(stale) during MachLockout — the one place the log intentionally carries
numbers the state machine refuses to expose to control logic. The coasting
flag is no longer stored: it is a pure timer, reconstructible as
ignition-time + `max_burn_time` from the flight profile.

## Radio telemetry (VLP downlink, LoRa)

| Packet | Size | When | Contents (summary) |
|---|---|---|---|
| `TelemetryPacket` | 43 B (343/344 bits — 1 spare) | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, KF altitude AGL + air speed (+ maxima), tilt, flight stage (4-bit honest mirror), drogue/main deployed bits, **ab altitude AGL + ab vertical velocity (signed, ±400 m/s @ ~1.6 m/s) + 3 vote bits + born + apogee**, **target apogee AGL**, amp status + 3 outputs + shared battery, Icarus status + airbrakes ext/temp, OzYS + SDRM status, payload stack status, **EPM battery + six rail currents + three SEM actuator step counts** |
| `LowPowerTelemetryPacket` | 11 B | every 5 s in LowPower + Demo | sats, gps_fixed, **lat/lon**, VL battery, amp online, shared battery, air temp |
| `LandedTelemetryPacket` | 12 B | every 5 s in Landed | lat/lon, sats, VL battery, amp status + outputs + shared battery |

Stage-honesty on the downlink: during MachLockout the packet reports the KF
altitude and air speed as 0 (the state carries no numbers — the stage value
explains why) while the ab fields stay live. FailedToReachMinApogee is
reported as itself. The ground now sees the airbrakes estimator directly:
whether it's born, which votes pass, its altitude/velocity, and the target
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
| deployment KF altitude ASL | ✓ (as AGL, coarse; 0 in MachLockout) | – | ✓ | – | yes — replay pressure through estimator |
| deployment KF vertical velocity | ✓ (as \|air_speed\|, 8-bit; 0 in MachLockout) | – | ✓ | – | yes — replay |
| ab estimator altitude | ✓ (as AGL, 13-bit) | – | ✓ (ASL, NaN until born) | – | yes — replay IMU+baro through estimator |
| ab estimator vertical velocity | ✓ (signed 9-bit) | – | ✓ (NaN until born) | – | yes — replay |
| ab tilt | ✓ (8-bit) | – | ✓ (NaN before ignition) | – | yes — replay / offline attitude |
| ab lockout-exit votes, born, apogee latch | ✓ (5 bits) | – | ✓ (ab_flags bits) | – | yes — replay |
| ab pad calibration complete | – | – | – | – | **nowhere on the wire** — RTT log line only; candidate for a VLCustomStatus bit |
| flight stage (honest, incl. MachLockout / FailedToReachMinApogee) | ✓ (4-bit) | – | ✓ full rate | ✓ (redundant) | logged |
| coasting (burn-timer flag) | – | – | – | – | yes — ignition time (stage change) + `max_burn_time` config; internal to the airbrakes gate |
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
| airbrakes commanded/actual ext. | ✓ (5-bit) | – | ✓ full rate | – | logged |
| airbrakes servo temp | ✓ | – | – | ✓ | logged |
| MPC predicted apogee | – | – | – | – | approx — re-run MPC on logged ab state |
| amp: online, outputs ×3, shared battery | ✓ | ✓/LD | – | ✓ | logged |
| bulkheads: online, brightness | – | – | – | – | **nowhere** — dropped from TM in v7; CAN-local only |
| Icarus / OzYS / SDRM online, uptime | ✓ | – | – | – | **not derivable** — radio only |
| payload stack status | ✓ (11-bit) | – | – | – | **not derivable** — radio only |
| EPM battery voltage | ✓ (1+10 bit, 11–17 V) | – | – | ✓ (raw mV) | logged |
| EPM rail currents ×6 (sys 3v3/5v, per 3v3/5v/9v/12v) | ✓ (1+10 bit each, 0–10.23 A @ 10 mA) | – | – | ✓ (raw mA) | logged |
| SEM actuator steps ×3 | ✓ (1+10 bit each, full u16 @ ~64 steps) | – | – | ✓ (raw steps) | logged |
| VL health flags (imu_ok, sd_ok, …) | – | – | – | – | CAN node status only; not persisted |

Takeaways:

- v7 closed the two old gaps: GPS UTC time is now logged at full rate (absolute
  time for video / GCM / redundant-FC correlation), and amp state + servo temp
  survive an RF-link drop via the slow record.
- v8 put the payload stack telemetry on SD as well: the slow record carries the
  raw mV / mA / step values (with the payload's own `0xFFFF` = unavailable
  sentinel), so an RF-link drop no longer loses EPM rails or actuator positions.
  The downlink copies are quantized; the SD copies are exact.
- The downlink packet grew 36 B -> 43 B (55 B on air after the type byte and
  reed-solomon ecc), which costs one extra symbol block: 1774 ms time-on-air
  instead of 1642 ms at SF12 / 250 kHz / CR 4:8, still inside the 2 s period.
  Time-on-air only steps at 50 / 55 / 60 bytes on air, so trimming the packet
  back to 39 B would recover the old 1642 ms.
- Still radio-only: payload stack status, Icarus/OzYS/SDRM health.
  A 1 Hz CAN-health snapshot record on SD would close the remainder.
- Bulkhead status is now on no channel at all (deliberate v7 drop) — it exists
  only as live CAN traffic.
- `calibration_complete()` (airbrakes pad calibration, a launch-readiness
  condition) is visible only on RTT — it has no CAN/telemetry slot. The
  natural home is a VLCustomStatus bit on the CAN heartbeat, relayed by the
  ground station as a GO/NO-GO light.
