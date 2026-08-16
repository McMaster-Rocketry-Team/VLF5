use crate::{
    AirBrakesWatch, AmpStateWatch, ContinuityWatch, FlightEstimatorsMutex, FlightStageMutex,
    GPSReadingWatch, PayloadStateWatch, VLStatusMutex,
    can_central::CanCentral,
    tasks::{
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
        unix_clock::UnixClock,
    },
    utils::drain_latest,
};

use air_brakes_controller_core::FlightEstimators;
use defmt::warn;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::{
    can_bus::node_types::AMP_NODE_TYPE,
    flight_data_record::{
        AB_APOGEE, AB_BARO_TRUSTED, AB_VOTE_BARO_RATE, AB_VOTE_DEPLOYMENT, AB_VOTE_INERTIAL,
        FlightDataFastRecord, FlightDataSlowRecord, LogRecord, VALID_AIRBRAKES_ACTUAL,
        VALID_AIRBRAKES_COMMANDED, VALID_BARO, VALID_BATTERY, VALID_GPS_ALT, VALID_GPS_FIX,
        VALID_IMU, VALID_MAG,
    },
};

/// Queue between the IMU-clocked logger and the SD writer.
pub const FLIGHT_DATA_CHANNEL_DEPTH: usize = 512;
pub type FlightDataChannel = Channel<NoopRawMutex, LogRecord, FLIGHT_DATA_CHANNEL_DEPTH>;

const IMU_TIMEOUT: Duration = Duration::from_millis(20);
const SLOW_INTERVAL: Duration = Duration::from_millis(100);

/// The estimator numbers a fast record carries, read straight off the live
/// estimators rather than a cached copy:
///
/// * `(kf_altitude_asl, kf_vertical_velocity)` — the deployment KF's raw,
///   possibly lockout-frozen output. Logging only.
/// * `(ab_altitude_asl, ab_vertical_velocity, ab_tilt_deg, ab_flags)` — the
///   airbrakes estimator, with `ab_flags` packing the `AB_*` bits (the three
///   lockout-exit votes, baro-trusted/born, apogee latch).
///
/// NaN for any value this session's estimators have not produced yet.
fn estimator_log_state(est: &FlightEstimators) -> ((f32, f32), (f32, f32, f32, u8)) {
    let dep = est.deployment_estimator();
    let kf = (dep.kf_altitude_asl(), dep.kf_vertical_velocity());

    let ab = est.airbrakes_estimator();
    let (v1, v2, v3) = ab.lockout_votes().unwrap_or((false, false, false));
    let mut flags = 0u8;
    if v1 {
        flags |= AB_VOTE_INERTIAL;
    }
    if v2 {
        flags |= AB_VOTE_DEPLOYMENT;
    }
    if v3 {
        flags |= AB_VOTE_BARO_RATE;
    }
    if ab.baro_trusted() {
        flags |= AB_BARO_TRUSTED;
    }
    if ab.is_apogee() {
        flags |= AB_APOGEE;
    }

    (
        kf,
        (
            ab.altitude_asl().unwrap_or(f32::NAN),
            ab.velocity().map(|v| v.y).unwrap_or(f32::NAN),
            ab.tilt().map(|t| t.to_degrees()).unwrap_or(f32::NAN),
            flags,
        ),
    )
}

/// Flight-data logging, run as one of armed mode's joined futures rather than a
/// standalone task — logging exists exactly as long as an armed session does.
///
/// That is also the logging policy, now expressed by construction instead of by
/// a mode predicate: `Armed` covers the whole ascent/coast/deploy/descent until
/// the auto-switch to `Landed`, and no other mode logs, so pre-flight checks and
/// time on the ground after landing don't fill the SD card. To log in every mode
/// for bench work, spawn this as its own task from `low_prio_main` instead —
/// every input below is already `&'static` except `estimators`, which only
/// exists while armed.
///
/// That containment is what makes cross-session leakage unrepresentable. Every
/// subscriber and counter below is created on entry, and an embassy pubsub
/// subscriber only receives messages published after its creation, so a re-arm
/// (Armed -> LowPower -> Armed) cannot carry pre-arm samples into the new
/// session's first records — no low-power baro backlog, and no mag readings left
/// over from the *previous* armed session (`mag_task` is powered down outside
/// Armed, so its queue would otherwise be minutes stale). The estimator numbers
/// are read live off `estimators`, the same instance armed mode rebuilds per
/// session, so a latched `AB_APOGEE` from a previous flight cannot be logged
/// against a rocket back on the pad either.
///
/// Cancellation-safe: records reach the SD writer through a non-blocking
/// `try_send`, so being dropped at a mode change cannot tear one.
pub async fn log_flight_data(
    estimators: &FlightEstimatorsMutex,
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mag_pubsub: &'static MagReadingPubSub,
    gps_watch: &'static GPSReadingWatch,
    battery_watch: &'static BatteryVWatch,
    continuity_watch: &'static ContinuityWatch,
    air_brakes_watch: &'static AirBrakesWatch,
    amp_state_watch: &'static AmpStateWatch,
    payload_state_watch: &'static PayloadStateWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    unix_clock: &'static UnixClock,
    flight_stage: &'static FlightStageMutex,
    vl_status: &'static VLStatusMutex,
    channel: &'static FlightDataChannel,
) {
    let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();
    let mut mag_sub = mag_pubsub.subscriber().unwrap();

    let mut sequence: u32 = 0;
    let mut backlog_warned = false;
    let mut last_slow_emit = Instant::now();

    loop {
        let (timestamp_us, imu_opt, baro_opt) =
            match select(imu_baro_sub.next_message_pure(), Timer::after(IMU_TIMEOUT)).await {
                Either::First(reading) => {
                    let (imu, baro) = reading.data;
                    (reading.timestamp_us, imu, Some(baro))
                }
                Either::Second(_) => (Instant::now().as_micros(), None, None),
            };

        let mag_opt = drain_latest(&mut mag_sub);
        // Read straight off the watches: this loop only ever samples their
        // latest value and never awaits a change, so it needs no receiver slot
        // (they are a scarce, `unwrap`-ed resource — `GPSReadingWatch` in
        // particular sits at its capacity while armed).
        let gps_opt = gps_watch.try_get();
        let battery_opt = battery_watch.try_get();
        let continuity_opt = continuity_watch.try_get();
        let airbrakes_opt = air_brakes_watch.try_get();
        let amp_opt = amp_state_watch.try_get();
        // Defaults to the 0xFFFF sentinel until the payload's first status message.
        let payload = payload_state_watch.try_get().unwrap_or_default();
        let stage = flight_stage.lock(|r| *r.borrow());
        // NaN until this session's estimators have produced their first sample.
        let ((kf_altitude_asl, kf_vertical_velocity), (
            ab_altitude_asl,
            ab_vertical_velocity,
            ab_tilt_deg,
            ab_flags,
        )) = estimators.lock(|s| estimator_log_state(&s.borrow()));

        let mut fast_valid = 0u8;
        let (acc, gyro) = match imu_opt {
            Some(imu) => {
                fast_valid |= VALID_IMU;
                (
                    [imu.acc.x, imu.acc.y, imu.acc.z],
                    [imu.gyro.x, imu.gyro.y, imu.gyro.z],
                )
            }
            None => ([0.0; 3], [0.0; 3]),
        };
        let (temperature, pressure) = match baro_opt {
            Some(b) => {
                fast_valid |= VALID_BARO;
                (b.temperature, b.pressure)
            }
            None => (0.0, 0.0),
        };
        let mag = match mag_opt {
            Some(r) => {
                fast_valid |= VALID_MAG;
                let m = r.data.mag;
                [m.x, m.y, m.z]
            }
            None => [0.0; 3],
        };

        let mut slow_valid = 0u8;
        let battery_voltage = match battery_opt {
            Some(r) => {
                slow_valid |= VALID_BATTERY;
                r.data
            }
            None => 0.0,
        };
        let (lat_lon, gps_altitude_asl, num_sats, hdop, vdop, pdop) = match gps_opt {
            Some(r) => {
                let g = r.data;
                if g.lat_lon.is_some() {
                    slow_valid |= VALID_GPS_FIX;
                }
                if g.altitude_asl.is_some() {
                    slow_valid |= VALID_GPS_ALT;
                }
                (
                    g.lat_lon.unwrap_or((0.0, 0.0)),
                    g.altitude_asl.unwrap_or(0.0),
                    g.num_of_fix_satellites,
                    g.hdop.unwrap_or(0.0),
                    g.vdop.unwrap_or(0.0),
                    g.pdop.unwrap_or(0.0),
                )
            }
            None => ((0.0, 0.0), 0.0, 0, 0.0, 0.0, 0.0),
        };
        let pyro_flags = match continuity_opt {
            Some(c) => {
                (c.pyro_main_continuity as u8)
                    | ((c.pyro_main_fire as u8) << 1)
                    | ((c.pyro_drogue_continuity as u8) << 2)
                    | ((c.pyro_drogue_fire as u8) << 3)
                    | ((c.short_circuit as u8) << 4)
            }
            None => 0,
        };
        let (air_brakes_commanded_extension, air_brakes_actual_extension, air_brakes_servo_temp) =
            match airbrakes_opt {
                Some(ab) => {
                    if ab.commanded_valid {
                        fast_valid |= VALID_AIRBRAKES_COMMANDED;
                    }
                    if ab.actual_valid {
                        fast_valid |= VALID_AIRBRAKES_ACTUAL;
                    }
                    (ab.commanded_extension, ab.actual_extension, ab.servo_temp)
                }
                None => (0.0, 0.0, 0.0),
            };
        let (amp_shared_battery_v, amp_out_status) = match amp_opt {
            Some(a) => (a.shared_battery_v, a.out_status),
            None => (0.0, 0),
        };
        let amp_online = can_central
            .get_nodes::<1>(AMP_NODE_TYPE)
            .first()
            .map(|node| node.is_online())
            .unwrap_or(false);

        let sd_ok = vl_status.lock(|s| s.borrow().sd_ok);
        if !sd_ok {
            continue;
        }

        let fast_record = LogRecord::Fast(FlightDataFastRecord {
            sequence,
            timestamp_us,
            // 0 until the GPS PPS has disciplined the unix clock.
            unix_time_us: unix_clock.convert_to_unix_us(timestamp_us).unwrap_or(0),
            acc,
            gyro,
            temperature,
            pressure,
            mag,
            kf_altitude_asl,
            kf_vertical_velocity,
            ab_altitude_asl,
            ab_vertical_velocity,
            ab_tilt_deg,
            ab_flags,
            pyro_flags,
            air_brakes_commanded_extension,
            air_brakes_actual_extension,
            flight_stage: stage,
            valid: fast_valid,
        });
        sequence = sequence.wrapping_add(1);

        let slow_record = FlightDataSlowRecord {
            timestamp_us,
            battery_voltage,
            lat_lon,
            gps_altitude_asl,
            num_of_fixed_satalites: num_sats,
            hdop,
            vdop,
            pdop,
            flight_stage: stage,
            air_brakes_servo_temp,
            amp_online,
            amp_out_status,
            amp_shared_battery_v,
            payload_epm_batt_mv: payload.epm_batt_mv,
            payload_rail_ma: payload.rail_ma,
            payload_actuator_steps: payload.actuator_steps,
            valid: slow_valid,
        };

        let emit_slow = last_slow_emit.elapsed() >= SLOW_INTERVAL;

        let mut send_failed = false;
        if channel.try_send(fast_record).is_err() {
            send_failed = true;
        } else if emit_slow {
            if channel.try_send(LogRecord::Slow(slow_record)).is_err() {
                send_failed = true;
            } else {
                last_slow_emit = Instant::now();
            }
        }

        if send_failed {
            if !backlog_warned {
                warn!("data_logger: SD path busy, dropping flight record");
                backlog_warned = true;
            }
        } else if backlog_warned {
            backlog_warned = false;
        }
    }
}
