# VLF5 Data Map — SD Log & Radio Telemetry

Reference for what data leaves the flight computer, on which channel, at what
rate, and whether a value lost from one channel can be reconstructed from
another after the flight. Updated 2026-08-13 (storage format v4).

## SD flight log (armed mode only)

Two tagged record types interleaved into 512 B blocks (508 usable + CRC32),
superblock flushed every 250 ms. Decoding: `rocket-cli download-file <out.csv>`
(one CSV row per fast record, latest slow snapshot merged in). Storage format
v4 is **not** backward compatible: logs written by older firmware are reported
as unsupported by rocket-cli, and the firmware starts a fresh log over them on
the next arm.

| Record | Rate | Wire size | Contents |
|---|---|---|---|
| **Fast** (tag 0x01) | ~427 Hz (per IMU data-ready edge; placeholder at 50 Hz during sensor-stream stalls) | 73 B | sequence, timestamp_us, accel ×3, gyro ×3, baro temperature + pressure, mag ×3 (100 Hz source, resampled), **KF altitude ASL**, **KF vertical velocity**, **flight stage**, valid bitmask |
| **Slow** (tag 0x02) | 10 Hz | 73 B | timestamp_us, battery voltage, GPS lat/lon/alt/sats/DOPs (~1 Hz source), flight stage (redundant with fast), pyro flags (continuity / fire / short), airbrakes commanded + actual extension, valid bitmask |

Throughput ≈ 32 kB/s ≈ 113 MB/hour; capacity is a non-issue.

The KF fields are NaN until the armed-mode estimator produces its first sample
(a few ms after arming). `flight_stage` in the fast record gives stage
transitions at ~2.3 ms resolution (previously only 100 ms via the slow record).

## Radio telemetry (VLP downlink, LoRa)

| Packet | Size | When | Contents (summary) |
|---|---|---|---|
| `TelemetryPacket` | 34 B | every 2 s in Armed + SelfTest | GPS fix, VL battery, air temp, pyro continuity, KF altitude AGL + air speed (+ maxima), flight stage, tilt (unimplemented, 0), amp status + outputs + shared battery, bulkhead status + brightness, Icarus status + airbrakes ext/temp, OzYS + SDRM status, payload stack status, EPM rails |
| `LowPowerTelemetryPacket` | 5 B | every 5 s in LowPower + Demo | sats, gps_fixed, VL battery, amp online, shared battery, air temp |
| `LandedTelemetryPacket` | 12 B | every 5 s in Landed | lat/lon, sats, VL battery, amp status + outputs + shared battery |
| `GPSBeaconPacket` | 12 B | **never sent by VLF5** (defined in VLP only) | — |

## Full data map

Columns: **TM** = TelemetryPacket (2 s) · **LP/LD** = low-power / landed
packets (5 s) · **Fast** = SD @ ~427 Hz · **Slow** = SD @ 10 Hz.

| Value | TM 2s | LP/LD 5s | Fast | Slow | Derivable post-flight? |
|---|---|---|---|---|---|
| accel, gyro (3-axis) | – | – | ✓ | – | primary data |
| mag (3-axis) | – | – | ✓ (100 Hz source) | – | primary data |
| baro pressure | – | – | ✓ | – | primary data |
| baro/air temperature | ✓ | LP | ✓ | – | logged |
| KF altitude ASL | ✓ (as AGL, coarse) | – | ✓ | – | yes — replay pressure through estimator |
| KF vertical velocity | ✓ (as \|air_speed\|, 8-bit) | – | ✓ | – | yes — replay |
| flight stage | ✓ (3-bit) | – | ✓ full rate | ✓ (redundant) | logged |
| max altitude / max airspeed | ✓ | – | – | – | yes — max() over fast records |
| tilt_deg | ✓ (always 0, unimplemented) | – | – | – | yes — offline attitude from logged IMU + mag |
| KF innovation / gate rejections | – | – | – | – | yes — replay pressure |
| launch pad altitude | – | – | – | – | yes — KF altitude while on pad |
| GPS lat/lon, sat count | ✓ | LD (lat/lon), both (sats) | – | ✓ | logged |
| GPS altitude, DOPs | – | – | – | ✓ | logged |
| GPS UTC time | – | – | – | – | **nowhere** — parsed then dropped; log has only boot-time µs |
| VL battery voltage | ✓ | ✓ | – | ✓ | logged |
| pyro continuity (main/drogue) | ✓ | – | – | ✓ | logged |
| pyro fire outputs, short-circuit | – | – | – | ✓ (10 Hz) | logged, ±100 ms |
| airbrakes commanded/actual ext. | ✓ (5-bit) | – | – | ✓ | logged |
| airbrakes servo temp | ✓ | – | – | – | **not derivable** — radio only |
| MPC predicted apogee | – | – | – | – | approx — re-run MPC on logged KF state |
| target apogee | – | – | – | – | SD config block (separate from log stream) |
| amp: online, outputs, shared battery | ✓ | ✓/LD | – | – | **not derivable** — radio only |
| bulkheads: online, brightness | ✓ | – | – | – | **not derivable** — radio only |
| Icarus / OzYS / SDRM online, uptime | ✓ | – | – | – | **not derivable** — radio only |
| payload stack status | ✓ (11-bit) | – | – | – | **not derivable** — radio only |
| EPM rails (batt, 3v3, 5v, per-5v, per-9v) | ✓ | – | – | – | **not derivable** — radio only |
| VL health flags (imu_ok, sd_ok, …) | – | – | – | – | CAN node status only; not persisted |

Takeaways:

- Everything marked "radio only" (CAN node health, EPM rails, stack status,
  servo temp) is lost forever if the RF link drops — a 1 Hz CAN-health snapshot
  record on SD would close that gap.
- GPS UTC time is the only value that is parsed and then discarded with no way
  to reconstruct it; adding it to the slow record would give the log absolute
  time for correlating with video / GCM logs / the redundant flight computer.
