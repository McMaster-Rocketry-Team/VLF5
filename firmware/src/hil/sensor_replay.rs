//! Time-based IMU/baro replay publisher for HIL.
//!
//! The barometer is the only sensor that drives flight decisions (launch detect,
//! apogee, deploy, landing, airbrakes MPC), so it is generated from a scripted
//! single-deploy vertical trajectory plus per-sample sensor noise. The flight clock
//! is relative to **entering Armed**, not boot: the operator arms over the radio and
//! the flight plays out from that instant, so re-arming replays the flight cleanly.
//! IMU is a constant stub (telemetry/logging only).

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
/// Pad-sit duration after Arm before ignition (s).
///
/// Must exceed the baro estimator's pad-altitude reference settle window (~10 s
/// low-pass). Too short → wrong pad reference → wrong AGL → wrong deploy altitude
/// or a spurious `FailedToReachMinApogee`. Prime adversarial-loop tuning target.
const PAD_DURATION_S: f32 = 15.0;
/// Constant body-frame acceleration during motor burn (m/s^2).
const BURN_ACCEL_MS2: f32 = 80.0;
/// Motor burn duration (s).
const BURN_TIME_S: f32 = 3.0;
const GRAVITY: f32 = 9.81;
/// Terminal (clamped) descent velocity (m/s, negative = down).
const DESCENT_TERMINAL_MS: f32 = -25.0;
const ISA_TEMP_C: f32 = 15.0;

/// 1-sigma barometric-altitude sensor noise (m). `baro_noise` has unit variance, so
/// per-sample altitude error has std `BARO_NOISE_M`. Measured on the real MS5607 on
/// this VLF5 (stationary, drift-removed first-difference): sigma ~= 0.36 m, peaks
/// ~+/-1.25 m over 10k samples. Crank up to stress the KF.
const BARO_NOISE_M: f32 = 0.36;
/// 1-sigma temperature sensor noise (deg C). Measured MS5607 per-sample temp noise is
/// tiny (~0.02 C, quantization-limited); temperature is decision-irrelevant regardless.
const TEMP_NOISE_C: f32 = 0.05;

/// Scripted vertical altitude (m ASL) at flight time `t_s` since ignition arm.
///
/// Timeline: pad hold → constant-accel burn → ballistic coast → terminal-velocity
/// descent → ground (clamped at pad). Open-loop: airbrake commands do not feed back
/// into the trajectory, so natural apogee is fixed regardless of the MPC output.
pub fn trajectory_altitude_asl(t_s: f32) -> f32 {
    if t_s < PAD_DURATION_S {
        return PAD_ALTITUDE_ASL;
    }
    let t = t_s - PAD_DURATION_S;

    // Powered ascent: constant accel from rest on the pad.
    if t < BURN_TIME_S {
        return PAD_ALTITUDE_ASL + 0.5 * BURN_ACCEL_MS2 * t * t;
    }

    let v_burnout = BURN_ACCEL_MS2 * BURN_TIME_S;
    let h_burnout = PAD_ALTITUDE_ASL + 0.5 * BURN_ACCEL_MS2 * BURN_TIME_S * BURN_TIME_S;
    let t_coast = t - BURN_TIME_S;

    // Ballistic coast (up then down) until descent reaches terminal velocity.
    let v_uncapped = v_burnout - GRAVITY * t_coast;
    if v_uncapped >= DESCENT_TERMINAL_MS {
        let altitude = h_burnout + v_burnout * t_coast - 0.5 * GRAVITY * t_coast * t_coast;
        return altitude.max(PAD_ALTITUDE_ASL);
    }

    // Terminal-velocity descent to the ground.
    let t_term = (v_burnout - DESCENT_TERMINAL_MS) / GRAVITY;
    let h_term = h_burnout + v_burnout * t_term - 0.5 * GRAVITY * t_term * t_term;
    let altitude = h_term + DESCENT_TERMINAL_MS * (t_coast - t_term);
    altitude.max(PAD_ALTITUDE_ASL)
}

/// Deterministic per-sample pseudo-noise with ~unit variance, roughly Gaussian in
/// [-3, 3].
///
/// Pure function of the sample index — no RNG state, no `rand` crate — so a given
/// build replays the exact same noise sequence, keeping HIL runs reproducible. Sums
/// three decorrelated uniform[-1,1] hashed draws: by the CLT this is Gaussian-ish
/// with std = sqrt(3) * (1/sqrt(3)) = 1.0, so `BARO_NOISE_M` reads directly as sigma.
pub fn baro_noise(seed: u32) -> f32 {
    let a = hash_unit(seed.wrapping_mul(0x9E37_79B1));
    let b = hash_unit(seed.wrapping_mul(0x85EB_CA77).wrapping_add(0x1656_67B1));
    let c = hash_unit(seed.wrapping_mul(0xC2B2_AE3D).wrapping_add(0x27D4_EB2F));
    a + b + c
}

/// Integer avalanche hash → f32 in [-1, 1].
fn hash_unit(mut x: u32) -> f32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7FEB_352D);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846C_A68B);
    x ^= x >> 16;
    (x as f32 / u32::MAX as f32) * 2.0 - 1.0
}

/// Generate a noisy barometer reading for flight time `t_s`, sample `sample_idx`.
pub fn generate_baro(t_s: f32, sample_idx: u32) -> BaroData {
    let altitude_asl = trajectory_altitude_asl(t_s) + BARO_NOISE_M * baro_noise(sample_idx);
    let pressure = calculate_isa_pressure(Metres(altitude_asl as f64)).0 as f32;
    let temperature = ISA_TEMP_C + TEMP_NOISE_C * baro_noise(!sample_idx);
    BaroData {
        temperature,
        pressure,
    }
}

/// One replay sample: noisy baro + constant IMU stub.
fn sample_at(t_s: f32, sample_idx: u32) -> (Option<IMUData>, BaroData) {
    let baro = generate_baro(t_s, sample_idx);
    // IMU is decision-irrelevant here (flight logic is baro-only); a constant upright
    // resting reading keeps CAN/telemetry/logging fields sane.
    let imu = Some(IMUData {
        acc: Vector3::new(0.0, 0.0, GRAVITY),
        gyro: Vector3::zeros(),
    });
    (imu, baro)
}

#[embassy_executor::task]
pub async fn sensor_replay_task(
    pubsub: &'static IMUBaroReadingPubSub,
    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    info!("HIL: sensor_replay_task started (flight clock starts on Arm, 416 Hz)");
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
        s.baro_ok = true;
    });

    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();
    let publisher = pubsub.dyn_publisher().unwrap();

    loop {
        match avionics_mode.get().await {
            AvionicsMode::Armed => {
                // Flight clock is latched on entry to Armed; leaving Armed drops it so a
                // re-arm restarts the flight deterministically from the pad.
                let armed_t0 = Instant::now();
                match select(
                    publish_loop(&publisher, Some(armed_t0), 416),
                    avionics_mode.changed_and(|m| *m != AvionicsMode::Armed),
                )
                .await
                {
                    Either::First(never) => match never {},
                    Either::Second(_) => {}
                }
            }
            AvionicsMode::SelfTest => {
                // Stable pad reading (flight clock held at 0) so self-test baro check passes.
                match select(
                    publish_loop(&publisher, None, 416),
                    avionics_mode.changed_and(|m| *m != AvionicsMode::SelfTest),
                )
                .await
                {
                    Either::First(never) => match never {},
                    Either::Second(_) => {}
                }
            }
            AvionicsMode::LowPower | AvionicsMode::Demo => {
                match select(publish_loop(&publisher, None, 5), avionics_mode.changed()).await {
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

/// Publish samples at `hz`. When `armed_t0` is set, the flight clock is `now - armed_t0`;
/// otherwise the clock is held at 0 (pad hold), still applying per-sample noise.
async fn publish_loop(
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
    armed_t0: Option<Instant>,
    hz: u64,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_hz(hz));
    let mut idx: u32 = 0;
    loop {
        ticker.next().await;
        let t_s = match armed_t0 {
            Some(t0) => (Instant::now() - t0).as_micros() as f32 / 1_000_000.0,
            None => 0.0,
        };
        let (imu, baro) = sample_at(t_s, idx);
        idx = idx.wrapping_add(1);
        let timestamp_us = Instant::now().as_micros();
        publisher.publish_immediate(SensorReading::new(timestamp_us, (imu, baro)));
    }
}
