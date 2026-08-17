//! Osiris replay — the `hil-dual` flight (feature `hil-dual`).
//!
//! Where [`super::baro_sim`] flies an analytic stand-in trajectory good
//! enough to walk the mode machine, this flies the *actual* rocket: the
//! OpenRocket run behind `FLIGHT_CONFIG`, baked into
//! [`super::osiris_table`], replayed through the same two sensor seams.
//!
//! That is the difference between "the state machine advances" and "the
//! flight config works". Osiris goes to Mach 1.91 and 9.5 km, so this is
//! the only bench profile that exercises the Mach lockout at all: the
//! barometer really does go quiet for 26 s, the drag check really does have
//! to find the subsonic crossing, and the airbrakes filter really is born
//! at ignition+18 s rather than a few seconds after burnout. Deployment is
//! the real dual profile too — drogue at apogee plus the configured 1 s,
//! main at 1500 ft AGL — which on this trajectory is six minutes of
//! descent, not thirty seconds.
//!
//! # Assembling sensors from the table
//!
//! The table stores physics, not sensor vectors (see
//! `scripts/gen_osiris_hil_table.py`), and this module turns it back into
//! what the two chips would have reported:
//!
//! * **Barometer** — the interpolated static pressure, plus the same
//!   measured MS5607 noise [`super::baro_sim`] uses.
//! * **Accelerometer** — the airframe axis is device +Z (same convention as
//!   [`super::imu_sim`]; the estimator self-calibrates the mounting, so the
//!   choice is free as long as pad gravity and burn thrust agree on one
//!   axis). Specific force is therefore `(0, 0, axial)`.
//! * **Gyro** — the table's non-rolling body rates rotated into the rolling
//!   frame, plus the roll rate itself on Z. Roll is integrated here rather
//!   than tabulated, so it stays exact between rows: at 1 rev/s a 100 Hz
//!   table would have to interpolate 3.6 deg per step.
//!
//! Noise and the injected gyro bias are drawn from the same helpers as the
//! scripted profile, so a HIL run's sensor character does not change with
//! the trajectory.

use core::f32::consts::PI;

use firmware_common_new::readings::{BaroData, IMUData};
use nalgebra::Vector3;

use super::noise::hash_noise;
use super::osiris_table::{END_S, TABLE};

/// Pad hold after Arm before the trajectory starts (s).
///
/// Two independent floors, and this clears both. The deployment estimator
/// low-passes its pad altitude reference with a 10 s time constant, and the
/// airbrakes estimator needs at least three surviving 2 s calibration
/// windows before it will admit an ignition at all. 20 s gives the first
/// two time constants and the second ten windows.
pub const PAD_DURATION_S: f32 = 20.0;

/// Measured 1-sigma sensor noise, from `imu_bench` on this board
/// (`scripts/imu_bench_stats.py`, 57 windows / 114 s stationary).
/// Per-axis, because they are genuinely not equal.
const ACCEL_NOISE_MS2: Vector3<f32> = Vector3::new(0.0147, 0.0190, 0.0359);
const GYRO_NOISE_DPS: Vector3<f32> = Vector3::new(0.448, 0.181, 0.048);

/// Injected constant gyro bias (deg/s, device frame), on the same order as
/// the real part's measured bias (x=+0.83, y=-2.28, z=+0.33 deg/s on this
/// board). Deliberately nonzero and mostly lateral so a broken pad
/// calibration is visible as phantom tilt rather than hidden in roll.
const GYRO_BIAS_DPS: Vector3<f32> = Vector3::new(0.5, -1.2, 0.2);

/// 1-sigma barometric noise expressed as pressure (Pa). The MS5607 measures
/// pressure, so the noise belongs here and not on an altitude: 0.36 m near
/// the pad is ~4.3 Pa, and the same 4.3 Pa is worth ~1.1 m at apogee, which
/// is the honest way round.
const BARO_NOISE_PA: f32 = 4.3;
const ISA_TEMP_C: f32 = 15.0;
const TEMP_NOISE_C: f32 = 0.05;

const IMU_SEED_SALT: u32 = 0x4F53_4901;

/// Peak roll rate (rad/s). Entirely invented — OpenRocket reports zero roll
/// for this design, and a rocket that does not roll never exercises the
/// gyro integration the tilt estimate depends on.
const ROLL_PEAK_RAD_S: f32 = 2.0 * PI;
/// Motor burn time (s), used only to shape the roll decay.
const BURN_S: f32 = 6.32;

/// Roll rate at flight time `t` (rad/s): spin up over the burn, then decay.
pub fn roll_rate(t: f32) -> f32 {
    if t <= 0.0 {
        return 0.0;
    }
    let spin_up = 1.0 - expf(-t / 2.0);
    let decay = expf(-(t - BURN_S).max(0.0) / 25.0);
    ROLL_PEAK_RAD_S * spin_up * decay
}

fn expf(x: f32) -> f32 {
    libm::expf(x)
}

/// Interpolate the table at flight time `t` (s since ignition), clamped at
/// both ends. Returns `[pressure_pa, axial_sf, wx, wy, wz]`.
fn sample(t: f32) -> [f32; 5] {
    let pick = |r: &[f32; 6]| [r[1], r[2], r[3], r[4], r[5]];

    if t <= TABLE[0][0] {
        return pick(&TABLE[0]);
    }
    if t >= END_S {
        return pick(&TABLE[TABLE.len() - 1]);
    }

    // Binary search for the bracketing pair. The table is two uniform
    // stretches rather than one, so an index computation would need to know
    // where the rate changes; a search does not.
    let (mut lo, mut hi) = (0usize, TABLE.len() - 1);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if TABLE[mid][0] <= t {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (a, b) = (&TABLE[lo], &TABLE[hi]);
    let span = b[0] - a[0];
    let s = if span > 0.0 { (t - a[0]) / span } else { 0.0 };
    let (pa, pb) = (pick(a), pick(b));
    let mut out = [0.0f32; 5];
    for i in 0..5 {
        out[i] = pa[i] + (pb[i] - pa[i]) * s;
    }
    out
}

/// Replay state that has to persist between samples: the roll angle, which
/// is integrated rather than tabulated.
pub struct OsirisReplay {
    roll_rad: f32,
    last_t_s: f32,
}

impl OsirisReplay {
    pub const fn new() -> Self {
        Self {
            roll_rad: 0.0,
            last_t_s: 0.0,
        }
    }

    /// Advance the roll integrator to flight time `t_s` and return the roll
    /// angle there. Resets cleanly when the clock goes backwards, which is
    /// what a re-arm looks like.
    fn roll_at(&mut self, t_s: f32) -> f32 {
        if t_s < self.last_t_s {
            self.roll_rad = 0.0;
        }
        let dt = (t_s - self.last_t_s).clamp(0.0, 0.25);
        self.roll_rad += roll_rate(t_s - PAD_DURATION_S) * dt;
        self.last_t_s = t_s;
        // keep the accumulator small so f32 resolution stays good across a
        // 500 s flight at 1 rev/s
        if self.roll_rad > 2.0 * PI {
            self.roll_rad -= 2.0 * PI;
        }
        self.roll_rad
    }

    /// Barometer reading at flight time `t_s` (since Arm), sample `idx`.
    pub fn baro(&mut self, t_s: f32, idx: u32) -> BaroData {
        let [pressure, ..] = sample(t_s - PAD_DURATION_S);
        BaroData {
            pressure: pressure + BARO_NOISE_PA * hash_noise(idx),
            temperature: ISA_TEMP_C + TEMP_NOISE_C * hash_noise(!idx),
        }
    }

    /// IMU reading at flight time `t_s` (since Arm), sample `idx`.
    ///
    /// Units and frame match the LSM6DSM driver's output exactly: specific
    /// force in m/s^2, angular rate in deg/s, one right-handed device frame.
    pub fn imu(&mut self, t_s: f32, idx: u32) -> IMUData {
        let t = t_s - PAD_DURATION_S;
        let [_, axial, wx0, wy0, wz0] = sample(t);
        let roll = self.roll_at(t_s);
        let (sr, cr) = (sinf(roll), cosf(roll));

        // Non-rolling body rates into the rolling device frame, plus the
        // roll itself about the axis.
        let gyro = Vector3::new(
            wx0 * cr + wy0 * sr,
            -wx0 * sr + wy0 * cr,
            wz0 + roll_rate(t).to_degrees(),
        );

        let noise = |channel: u32| {
            hash_noise(
                idx.wrapping_mul(8)
                    .wrapping_add(channel)
                    .wrapping_add(IMU_SEED_SALT),
            )
        };

        IMUData {
            // airframe axis is device +Z; off-axis specific force is the
            // normal force, which at Osiris's <= 1 deg angle of attack is
            // small enough that noise dominates it
            acc: Vector3::new(
                ACCEL_NOISE_MS2.x * noise(0),
                ACCEL_NOISE_MS2.y * noise(1),
                axial + ACCEL_NOISE_MS2.z * noise(2),
            ),
            gyro: gyro
                + GYRO_BIAS_DPS
                + Vector3::new(
                    GYRO_NOISE_DPS.x * noise(3),
                    GYRO_NOISE_DPS.y * noise(4),
                    GYRO_NOISE_DPS.z * noise(5),
                ),
        }
    }
}

fn sinf(x: f32) -> f32 {
    libm::sinf(x)
}

fn cosf(x: f32) -> f32 {
    libm::cosf(x)
}
