//! HIL IMU simulation — the inertial half of the scripted flight.
//!
//! The bench IMU is real but stationary: it never feels ignition, so on a
//! real-IMU bench the airbrakes estimator would sit in `OnPad` for the whole
//! simulated flight and the workarounds would have to live downstream at the
//! application layer — exactly what HIL's "substitute at the sensor boundary
//! only" rule forbids. So HIL synthesizes the accel/gyro *values* too, from
//! the same arm-relative script clock the baro uses, and the entire airbrakes
//! estimator (pad bias calibration → ignition → thrust-vector alignment →
//! dead reckoning → filter birth → tracking → apogee) flies the production
//! path on the bench.
//!
//! Only the values are synthetic. The real LSM6DSM is still read on its real
//! data-ready interrupt and the published timestamps are the genuine read
//! timestamps, so the estimators' measured-dt integration is exercised
//! against real sample timing — jitter, stalls and all. See
//! `imu_values_or_sim` in `sensor_tasks.rs` for the seam.
//!
//! Units and frame are exactly what the LSM6DSM driver produces at that seam
//! (`IMUData`: accel is specific force in m/s^2, gyro in deg/s, one
//! right-handed device frame; armed mode converts gyro to rad/s at the
//! estimator edge). The script points the rocket axis along device +Z. The
//! estimator self-calibrates the mounting from pad gravity and thrust
//! direction, so the axis choice is arbitrary — what matters is that pad
//! gravity and burn thrust agree on one axis, i.e. a vertical rail.

use firmware_common_new::readings::IMUData;
use nalgebra::Vector3;

use super::baro_sim::{BURN_ACCEL_MS2, BURN_TIME_S, DESCENT_TERMINAL_MS, GRAVITY, PAD_DURATION_S};
use super::noise::hash_noise;

/// 1-sigma accelerometer noise (m/s^2), per axis.
///
/// Measured on THIS board's LSM6DSM by `cargo run --bin imu_bench`, 57
/// stationary 2 s windows over 114 s, aggregated by
/// `scripts/imu_bench_stats.py` (median window sigma). The axes are not
/// equal and the earlier scalar 0.07 — taken from the Void Lake flight pad,
/// which had rail sway and a live motor in it — was 2-5x pessimistic.
const ACCEL_NOISE_MS2: Vector3<f32> = Vector3::new(0.0147, 0.0190, 0.0359);
/// 1-sigma gyro noise (deg/s), per axis, same capture. Note X and Y are
/// several times noisier than the old scalar 0.1 guess, and Z is quieter.
const GYRO_NOISE_DPS: Vector3<f32> = Vector3::new(0.448, 0.181, 0.048);

/// Constant injected gyro bias (deg/s, device frame).
///
/// Deliberately nonzero so the pad bias calibration is exercised for real,
/// and on a lateral axis so a broken calibration is *visible*: uncorrected,
/// 0.1 deg/s about X integrates to ~2.7 deg of phantom tilt by apogee
/// (~27 s), and the gravity misprojection turns that into dead-reckoned
/// velocity error. A roll (Z) bias would only spin the rocket about its own
/// axis and hide. Realistic magnitude for the LSM6DSM's in-run bias.
const GYRO_BIAS_DPS: Vector3<f32> = Vector3::new(0.1, 0.0, 0.0);

/// Salt keeping the six IMU noise streams clear of the baro's seed stream
/// (`generate_baro` hashes the raw sample index).
const IMU_SEED_SALT: u32 = 0x494D_5501;

/// Scripted specific force along device +Z (m/s^2) at flight time `t_s`.
///
/// The trajectory in [`super::baro_sim`] is analytic, so the specific force —
/// what an accelerometer actually measures, f = a − g — is exact per phase,
/// derived from the same constants:
///
/// * pad hold, and later the terminal descent and the ground: supported
///   (rail / drag / dirt) at 1 g → `+GRAVITY`
/// * burn: net upward acceleration `BURN_ACCEL_MS2` plus the 1 g the thrust
///   also has to supply → `BURN_ACCEL_MS2 + GRAVITY`
/// * ballistic coast (up, over the top, and down until the descent caps at
///   terminal velocity): gravity is the only force → `0`
pub fn specific_force_up(t_s: f32) -> f32 {
    if t_s < PAD_DURATION_S {
        return GRAVITY;
    }
    let t = t_s - PAD_DURATION_S;

    if t < BURN_TIME_S {
        return BURN_ACCEL_MS2 + GRAVITY;
    }

    // Same phase split as `trajectory_altitude_asl`: ballistic until the
    // descent reaches terminal velocity (well above ground in this profile),
    // then constant-velocity descent — aerodynamically supported at 1 g —
    // which also covers sitting on the ground after touchdown.
    let v_burnout = BURN_ACCEL_MS2 * BURN_TIME_S;
    let t_coast = t - BURN_TIME_S;
    if v_burnout - GRAVITY * t_coast >= DESCENT_TERMINAL_MS {
        0.0
    } else {
        GRAVITY
    }
}

/// Generate a noisy IMU reading for flight time `t_s`, sample `sample_idx`.
///
/// Same units/frame as the real driver output: accel specific force in
/// m/s^2, gyro in deg/s. Six per-sample hash-noise channels, decorrelated
/// from each other and from the baro's.
pub fn generate_imu(t_s: f32, sample_idx: u32) -> IMUData {
    let noise = |channel: u32| {
        hash_noise(
            sample_idx
                .wrapping_mul(8)
                .wrapping_add(channel)
                .wrapping_add(IMU_SEED_SALT),
        )
    };
    IMUData {
        acc: Vector3::new(
            ACCEL_NOISE_MS2.x * noise(0),
            ACCEL_NOISE_MS2.y * noise(1),
            specific_force_up(t_s) + ACCEL_NOISE_MS2.z * noise(2),
        ),
        gyro: GYRO_BIAS_DPS
            + Vector3::new(
                GYRO_NOISE_DPS.x * noise(3),
                GYRO_NOISE_DPS.y * noise(4),
                GYRO_NOISE_DPS.z * noise(5),
            ),
    }
}
