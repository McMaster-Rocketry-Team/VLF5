use crate::{
    AirBrakesWatch, AirbrakesStateWatch, AmpStateWatch, AvionicsModeWatch, ContinuityWatch,
    FlightStageMutex, GPSReadingWatch, KfStateWatch, PayloadStateWatch, VLStatusMutex,
    can_central::CanCentral,
    tasks::{
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
        unix_clock::UnixClock,
    },
    utils::drain_latest,
};

use defmt::warn;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::{
    can_bus::node_types::AMP_NODE_TYPE,
    flight_data_record::{
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

#[embassy_executor::task]
pub async fn data_logger(
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
    kf_state_watch: &'static KfStateWatch,
    airbrakes_state_watch: &'static AirbrakesStateWatch,
    avionics_mode_watch: &'static AvionicsModeWatch,
    vl_status: &'static VLStatusMutex,
    channel: &'static FlightDataChannel,
) {
    let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();
    let mut mag_sub = mag_pubsub.subscriber().unwrap();
    let mut gps_receiver = gps_watch.receiver().unwrap();
    let mut battery_receiver = battery_watch.receiver().unwrap();
    let mut continuity_receiver = continuity_watch.receiver().unwrap();
    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();

    let mut sequence: u32 = 0;
    let mut backlog_warned = false;
    let mut last_slow_emit = Instant::now();

    loop {
        if !avionics_mode
            .try_get()
            .map(|m| m.should_log())
            .unwrap_or(false)
        {
            avionics_mode.changed().await;
            backlog_warned = false;
            continue;
        }

        let (timestamp_us, imu_opt, baro_opt) =
            match select(imu_baro_sub.next_message_pure(), Timer::after(IMU_TIMEOUT)).await {
                Either::First(reading) => {
                    let (imu, baro) = reading.data;
                    (reading.timestamp_us, imu, Some(baro))
                }
                Either::Second(_) => (Instant::now().as_micros(), None, None),
            };

        let mag_opt = drain_latest(&mut mag_sub);
        let gps_opt = gps_receiver.try_get();
        let battery_opt = battery_receiver.try_get();
        let continuity_opt = continuity_receiver.try_get();
        let airbrakes_opt = air_brakes_watch.try_get();
        let amp_opt = amp_state_watch.try_get();
        // Defaults to the 0xFFFF sentinel until the payload's first status message.
        let payload = payload_state_watch.try_get().unwrap_or_default();
        let stage = flight_stage.lock(|r| *r.borrow());
        // NaN until the armed-mode estimators have produced their first sample.
        let (kf_altitude_asl, kf_vertical_velocity) =
            kf_state_watch.try_get().unwrap_or((f32::NAN, f32::NAN));
        let (ab_altitude_asl, ab_vertical_velocity, ab_tilt_deg, ab_flags) = airbrakes_state_watch
            .try_get()
            .unwrap_or((f32::NAN, f32::NAN, f32::NAN, 0));

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
