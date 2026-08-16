//! HIL barometer simulation — the baro half of the scripted flight.
//!
//! A static bench can't feel altitude, so in HIL builds the barometer's
//! *value* is replaced by a scripted single-deploy vertical trajectory plus
//! measured sensor noise. The trajectory constants below are the single
//! source of truth for the whole script: [`super::imu_sim`] derives its
//! per-phase specific force from these same numbers, and both sensors run off
//! the same arm-relative flight clock ([`super::HilSimState`]), so the baro
//! and the IMU always tell one consistent story.

use firmware_common_new::readings::BaroData;
use icao_isa::calculate_isa_pressure;
use icao_units::si::Metres;

use super::noise::hash_noise;

/// Pad altitude ASL used by the scripted flight profile (m).
pub const PAD_ALTITUDE_ASL: f32 = 200.0;
/// Pad-sit duration after Arm before ignition (s).
///
/// Must exceed the baro estimator's pad-altitude reference settle window (~10 s
/// low-pass). Too short → wrong pad reference → wrong AGL → wrong deploy
/// altitude or a spurious `FailedToReachMinApogee`. Also sized for the
/// airbrakes estimator's pad gyro-bias calibration: 15 s of pad hold completes
/// ~7 of its 2 s bias windows before ignition (bump this if
/// calibration-complete ever requires more). Prime adversarial-loop tuning
/// target.
pub(crate) const PAD_DURATION_S: f32 = 15.0;
/// Constant net upward acceleration during motor burn (m/s^2).
pub(crate) const BURN_ACCEL_MS2: f32 = 80.0;
/// Motor burn duration (s).
pub(crate) const BURN_TIME_S: f32 = 3.0;
pub(crate) const GRAVITY: f32 = 9.81;
/// Terminal (clamped) descent velocity (m/s, negative = down).
pub(crate) const DESCENT_TERMINAL_MS: f32 = -25.0;
const ISA_TEMP_C: f32 = 15.0;

/// 1-sigma barometric-altitude sensor noise (m). `hash_noise` has unit
/// variance, so per-sample altitude error has std `BARO_NOISE_M`. Measured on
/// the real MS5607 on this VLF5 (stationary, drift-removed first-difference):
/// sigma ~= 0.36 m, peaks ~+/-1.25 m over 10k samples. Crank up to stress the
/// KF.
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

/// Generate a noisy barometer reading for flight time `t_s`, sample `sample_idx`.
pub fn generate_baro(t_s: f32, sample_idx: u32) -> BaroData {
    let altitude_asl = trajectory_altitude_asl(t_s) + BARO_NOISE_M * hash_noise(sample_idx);
    let pressure = calculate_isa_pressure(Metres(altitude_asl as f64)).0 as f32;
    let temperature = ISA_TEMP_C + TEMP_NOISE_C * hash_noise(!sample_idx);
    BaroData {
        temperature,
        pressure,
    }
}
