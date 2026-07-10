//! HIL barometer simulation.
//!
//! The barometer is the one sensor that cannot be exercised on a static bench — it
//! can't feel altitude — so in HIL builds its *value* is replaced by a scripted
//! single-deploy vertical trajectory plus measured sensor noise. Every other sensor,
//! task, and GPIO stays real (IMU, GPS, pyro, mag, CAN, SD, LoRa): HIL affects only
//! the baro reading. The flight clock is relative to **entering Armed**, not boot —
//! the operator arms over the radio and the flight plays out from that instant, and
//! leaving Armed resets the clock so a re-arm replays cleanly from the pad.

use embassy_time::Instant;
use firmware_common_new::readings::BaroData;
use icao_isa::calculate_isa_pressure;
use icao_units::si::Metres;

use crate::avionics_mode::AvionicsMode;

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

/// Per-task HIL baro state: a monotonic sample counter plus the Arm-relative
/// flight-clock origin.
pub struct HilBaroState {
    idx: u32,
    armed_t0: Option<Instant>,
}

impl HilBaroState {
    pub const fn new() -> Self {
        Self {
            idx: 0,
            armed_t0: None,
        }
    }

    /// Next synthetic baro sample for the current avionics `mode`.
    ///
    /// The flight clock latches on the first `Armed` sample and resets whenever the
    /// mode leaves `Armed`, so pre-arm modes (SelfTest / LowPower / Demo) always read
    /// the noisy pad altitude and re-arming replays the flight cleanly from the pad.
    pub fn next(&mut self, mode: AvionicsMode) -> BaroData {
        let now = Instant::now();
        if mode == AvionicsMode::Armed {
            if self.armed_t0.is_none() {
                self.armed_t0 = Some(now);
            }
        } else {
            self.armed_t0 = None;
        }
        let t_s = match self.armed_t0 {
            Some(t0) => (now - t0).as_micros() as f32 / 1_000_000.0,
            None => 0.0,
        };
        let baro = generate_baro(t_s, self.idx);
        self.idx = self.idx.wrapping_add(1);
        baro
    }
}
