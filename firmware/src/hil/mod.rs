//! HIL barometer simulation (features `hil-dual` / `hil-single`).
//!
//! In HIL builds every task and sensor runs on real hardware — IMU, GPS, pyro GPIO,
//! mag, CAN, SD, and the LoRa radio — and **only the barometer reading is
//! synthesized** from a scripted flight ([`baro_sim`]). The board boots into SelfTest
//! exactly like a flight build and is flown from rocket-cli over the radio (see the
//! crate-root [`HIL.md`](../../HIL.md) for the plot guide).
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
