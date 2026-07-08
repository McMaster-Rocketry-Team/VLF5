use crate::{
    AvionicsModeWatch, ContinuityWatch, FlightStageMutex, GPSReadingWatch, VLStatusMutex,
    tasks::sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
    utils::drain_latest,
};

use defmt::warn;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::flight_data_record::FlightDataRecord;

/// Queue between the IMU-clocked logger and the SD writer. Sized for ~1 s of
/// backlog at 416 Hz and worst-case ~40 ms SDIO stalls (see `sd_bench`).
pub const FLIGHT_DATA_CHANNEL_DEPTH: usize = 512;
pub type FlightDataChannel = Channel<NoopRawMutex, FlightDataRecord, FLIGHT_DATA_CHANNEL_DEPTH>;

pub const VALID_IMU: u8 = 1 << 0;
pub const VALID_BARO: u8 = 1 << 1;
pub const VALID_MAG: u8 = 1 << 2;
pub const VALID_GPS_FIX: u8 = 1 << 3;
pub const VALID_GPS_ALT: u8 = 1 << 4;
pub const VALID_BATTERY: u8 = 1 << 5;

const IMU_TIMEOUT: Duration = Duration::from_millis(20);

#[embassy_executor::task]
pub async fn data_logger(
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mag_pubsub: &'static MagReadingPubSub,
    gps_watch: &'static GPSReadingWatch,
    battery_watch: &'static BatteryVWatch,
    continuity_watch: &'static ContinuityWatch,
    flight_stage: &'static FlightStageMutex,
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

    let mut record_count: u32 = 0;
    let mut backlog_warned = false;

    loop {
        if !avionics_mode
            .try_get()
            .map(|m| m.sensors_active())
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
        let stage = flight_stage.lock(|r| *r.borrow());

        let mut valid = 0u8;
        let (acc, gyro) = match imu_opt {
            Some(imu) => {
                valid |= VALID_IMU;
                (
                    [imu.acc.x, imu.acc.y, imu.acc.z],
                    [imu.gyro.x, imu.gyro.y, imu.gyro.z],
                )
            }
            None => ([0.0; 3], [0.0; 3]),
        };
        let (temperature, pressure) = match baro_opt {
            Some(b) => {
                valid |= VALID_BARO;
                (b.temperature, b.pressure)
            }
            None => (0.0, 0.0),
        };
        let mag = match mag_opt {
            Some(r) => {
                valid |= VALID_MAG;
                let m = r.data.mag;
                [m.x, m.y, m.z]
            }
            None => [0.0; 3],
        };
        let battery_voltage = match battery_opt {
            Some(r) => {
                valid |= VALID_BATTERY;
                r.data
            }
            None => 0.0,
        };
        let (lat_lon, altitude, num_sats, hdop, vdop, pdop) = match gps_opt {
            Some(r) => {
                let g = r.data;
                if g.lat_lon.is_some() {
                    valid |= VALID_GPS_FIX;
                }
                if g.altitude.is_some() {
                    valid |= VALID_GPS_ALT;
                }
                (
                    g.lat_lon.unwrap_or((0.0, 0.0)),
                    g.altitude.unwrap_or(0.0),
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

        let record = FlightDataRecord {
            record_count,
            timestamp_us,
            acc,
            gyro,
            temperature,
            pressure,
            mag,
            battery_voltage,
            valid,
            lat_lon,
            altitude,
            num_of_fixed_satalites: num_sats,
            hdop,
            vdop,
            pdop,
            flight_stage: stage,
            pyro_flags,
        };
        record_count = record_count.wrapping_add(1);

        let sd_ok = vl_status.lock(|s| s.borrow().sd_ok);
        if !sd_ok {
            continue;
        }

        if channel.try_send(record).is_err() {
            if !backlog_warned {
                warn!("data_logger: SD path busy, dropping flight record");
                backlog_warned = true;
            }
        } else if backlog_warned {
            backlog_warned = false;
        }
    }
}
