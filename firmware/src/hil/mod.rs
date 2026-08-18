//! HIL sensor simulation (features `hil-dual` / `hil-single`).
//!
//! HIL's rule: **substitute at the sensor boundary only.** Every task, bus,
//! and GPIO runs the production path on real hardware — GPS, pyro GPIO, mag,
//! CAN, SD, and the LoRa radio — and the only things replaced are the sensor
//! *values* a static bench cannot produce: the barometer reading
//! ([`baro_sim`]) and the IMU's accel/gyro values ([`imu_sim`]), both
//! synthesized from one flight. `hil-single` flies a short analytic script
//! ([`baro_sim`] / [`imu_sim`]) that walks the mode machine in about a
//! minute; `hil-dual` replays the real Osiris OpenRocket trajectory
//! ([`osiris`]) against the real `FLIGHT_CONFIG` — Mach 1.9, 9.5 km, and
//! eight minutes of wall clock. The IMU chip itself is
//! still read on its real data-ready interrupt, so loop pacing and sample
//! timestamps stay genuine; only the values are swapped. The board boots into
//! SelfTest exactly like a flight build and is flown from rocket-cli over the
//! radio — except under `boot-armed`, which is opt-in. The crate-root
//! [`HIL2.md`](../../HIL2.md) carries the bench specifics: probe identification,
//! measured hardware properties, verified runs, and what HIL does not prove.
//!
//! Because the real pyro task drives real GPIO in HIL,
//! **never flash a HIL build with live e-matches connected.**

#[cfg(all(feature = "hil-dual", feature = "hil-single"))]
compile_error!("enable only one of hil-dual or hil-single, not both");

#[cfg(all(
    feature = "hil-replay",
    not(feature = "hil-dual"),
    not(feature = "hil-single")
))]
compile_error!("hil-replay requires hil-dual or hil-single");

pub mod baro_sim;
pub mod imu_sim;
pub mod noise;
#[cfg(feature = "hil-dual")]
pub mod osiris;
#[cfg(feature = "hil-dual")]
#[rustfmt::skip]
pub mod osiris_table;

use embassy_time::Instant;
use firmware_common_new::readings::{BaroData, IMUData};

use crate::avionics_mode::AvionicsMode;

/// Per-task HIL sim state: the Arm-relative script clock plus one monotonic
/// sample counter per synthesized sensor.
///
/// One struct on purpose: the baro and the IMU must key off the **same**
/// flight clock, or the two sensors tell different stories (a baro that
/// climbs while the accelerometer still reads a quiet 1 g would — correctly —
/// never get past the estimators). The clock latches on the first sample seen
/// in `Armed` and resets whenever the mode leaves `Armed`, so pre-arm modes
/// (SelfTest / LowPower / Demo) always read the stationary pad and a re-arm
/// replays the flight cleanly from the pad.
pub struct HilSimState {
    armed_t0: Option<Instant>,
    baro_idx: u32,
    imu_idx: u32,
    /// `hil-dual` replays the real Osiris trajectory from a baked table and
    /// has to carry its roll integrator across samples; `hil-single` flies
    /// the analytic script, which is stateless.
    #[cfg(feature = "hil-dual")]
    osiris: osiris::OsirisReplay,
}

impl HilSimState {
    pub const fn new() -> Self {
        Self {
            armed_t0: None,
            baro_idx: 0,
            imu_idx: 0,
            #[cfg(feature = "hil-dual")]
            osiris: osiris::OsirisReplay::new(),
        }
    }

    /// Flight time (s) since entering `Armed`, latching/resetting the shared
    /// script clock for the current avionics `mode`. Idempotent within a
    /// sample: whichever sensor asks first latches the clock, the other reads
    /// the same origin.
    fn flight_time_s(&mut self, mode: AvionicsMode) -> f32 {
        let now = Instant::now();
        if mode == AvionicsMode::Armed {
            if self.armed_t0.is_none() {
                self.armed_t0 = Some(now);
            }
        } else {
            self.armed_t0 = None;
        }
        match self.armed_t0 {
            Some(t0) => (now - t0).as_micros() as f32 / 1_000_000.0,
            None => 0.0,
        }
    }

    /// Next synthetic baro sample for the current avionics `mode`.
    pub fn next_baro(&mut self, mode: AvionicsMode) -> BaroData {
        let t_s = self.flight_time_s(mode);
        #[cfg(feature = "hil-dual")]
        let baro = self.osiris.baro(t_s, self.baro_idx);
        #[cfg(not(feature = "hil-dual"))]
        let baro = baro_sim::generate_baro(t_s, self.baro_idx);
        self.baro_idx = self.baro_idx.wrapping_add(1);
        baro
    }

    /// Next synthetic IMU values for the current avionics `mode`. The caller
    /// keeps the real reading's DRDY timestamp — only the values are
    /// replaced.
    pub fn next_imu(&mut self, mode: AvionicsMode) -> IMUData {
        let t_s = self.flight_time_s(mode);
        #[cfg(feature = "hil-dual")]
        let imu = self.osiris.imu(t_s, self.imu_idx);
        #[cfg(not(feature = "hil-dual"))]
        let imu = imu_sim::generate_imu(t_s, self.imu_idx);
        self.imu_idx = self.imu_idx.wrapping_add(1);
        imu
    }
}
