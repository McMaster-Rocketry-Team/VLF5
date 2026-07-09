//! Time-based IMU/baro replay publisher for HIL.

use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::DynPublisher;
use embassy_time::{Duration, Instant, Ticker};
use firmware_common_new::{
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};
use icao_isa::calculate_isa_pressure;
use icao_units::si::Metres;
use nalgebra::Vector3;

use crate::{
    avionics_mode::AvionicsMode, tasks::sensor_tasks::IMUBaroReadingPubSub, AvionicsModeWatch,
    VLStatusMutex,
};

/// Pad altitude ASL used by the scripted flight profile (m).
pub const PAD_ALTITUDE_ASL: f32 = 200.0;
/// Seconds of pad sit after boot before ignition (must be after VLP arm at ~3 s).
const PAD_DURATION_S: f32 = 8.0;
const BURN_ACCEL_MS2: f32 = 80.0;
const BURN_TIME_S: f32 = 3.0;
const GRAVITY: f32 = 9.81;
const DESCENT_TERMINAL_MS: f32 = -25.0;
const ISA_TEMP_C: f32 = 15.0;

/// Scripted vertical flight sample at time `t_s` since boot.
///
/// Timeline: pad → burn → coast/free-fall → ground (same shape as baro estimator unit tests).
pub fn sample_at(t_s: f32) -> (Option<IMUData>, BaroData) {
    let (altitude_asl, vertical_accel_body) = profile_altitude_accel(t_s);
    let pressure = calculate_isa_pressure(Metres(altitude_asl as f64)).0 as f32;
    let baro = BaroData {
        temperature: ISA_TEMP_C,
        pressure,
    };
    let imu = Some(IMUData {
        // Body-frame vertical accel including gravity (approx upright rocket).
        acc: Vector3::new(0.0, 0.0, vertical_accel_body),
        gyro: Vector3::zeros(),
    });
    (imu, baro)
}

/// Returns (altitude_asl, body-frame vertical accel including gravity).
fn profile_altitude_accel(t_s: f32) -> (f32, f32) {
    if t_s < PAD_DURATION_S {
        return (PAD_ALTITUDE_ASL, GRAVITY);
    }

    let t = t_s - PAD_DURATION_S;

    // Burn: a = BURN_ACCEL, v0 = 0, h0 = pad
    if t < BURN_TIME_S {
        let altitude = PAD_ALTITUDE_ASL + 0.5 * BURN_ACCEL_MS2 * t * t;
        return (altitude, BURN_ACCEL_MS2 + GRAVITY);
    }

    let v_burnout = BURN_ACCEL_MS2 * BURN_TIME_S;
    let h_burnout = PAD_ALTITUDE_ASL + 0.5 * BURN_ACCEL_MS2 * BURN_TIME_S * BURN_TIME_S;
    let t_coast = t - BURN_TIME_S;

    // Coast / free-fall with terminal velocity clamp on descent.
    // Without clamp: v = v_bo - g*t, h = h_bo + v_bo*t - 0.5*g*t^2
    let v_uncapped = v_burnout - GRAVITY * t_coast;
    if v_uncapped >= DESCENT_TERMINAL_MS {
        let altitude = h_burnout + v_burnout * t_coast - 0.5 * GRAVITY * t_coast * t_coast;
        if altitude <= PAD_ALTITUDE_ASL {
            return (PAD_ALTITUDE_ASL, GRAVITY);
        }
        return (altitude, 0.0); // free-fall sensed as ~0 in body if we model specific force; use 0 for coast
    }

    // Time when terminal velocity is reached
    let t_term = (v_burnout - DESCENT_TERMINAL_MS) / GRAVITY;
    let h_term = h_burnout + v_burnout * t_term - 0.5 * GRAVITY * t_term * t_term;
    let t_after = t_coast - t_term;
    let altitude = h_term + DESCENT_TERMINAL_MS * t_after;
    if altitude <= PAD_ALTITUDE_ASL {
        return (PAD_ALTITUDE_ASL, GRAVITY);
    }
    (altitude, 0.0)
}

#[embassy_executor::task]
pub async fn sensor_replay_task(
    pubsub: &'static IMUBaroReadingPubSub,
    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    info!("HIL: sensor_replay_task started (416 Hz when Armed/SelfTest)");
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
        s.baro_ok = true;
    });

    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();
    let publisher = pubsub.dyn_publisher().unwrap();
    let t0 = Instant::now();

    loop {
        match avionics_mode.get().await {
            AvionicsMode::Armed | AvionicsMode::SelfTest => {
                match select(
                    publish_loop(&publisher, t0, 416),
                    avionics_mode.changed_and(|m| {
                        *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest
                    }),
                )
                .await
                {
                    Either::First(never) => match never {},
                    Either::Second(_) => {}
                }
            }
            AvionicsMode::LowPower | AvionicsMode::Demo => {
                match select(
                    publish_loop(&publisher, t0, 5),
                    avionics_mode.changed(),
                )
                .await
                {
                    Either::First(never) => match never {},
                    Either::Second(_) => {}
                }
            }
            AvionicsMode::Landed => {
                avionics_mode.changed().await;
            }
        }
    }
}

async fn publish_loop(
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
    t0: Instant,
    hz: u64,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_hz(hz));
    loop {
        ticker.next().await;
        let t_s = (Instant::now() - t0).as_millis() as f32 / 1000.0;
        let (imu, baro) = sample_at(t_s);
        let timestamp_us = Instant::now().as_micros();
        publisher.publish_immediate(SensorReading::new(timestamp_us, (imu, baro)));
    }
}
