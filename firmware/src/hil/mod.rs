//! In-process HIL replay (features `hil-dual` / `hil-single`).
//!
//! See crate-root [`HIL.md`](../../HIL.md) for the full plot guide (run commands,
//! timeline, RTT checklist, rocket-cli SD dump).
//!
//! **Never flash a HIL build with live e-matches connected.**

#[cfg(all(feature = "hil-dual", feature = "hil-single"))]
compile_error!("enable only one of hil-dual or hil-single, not both");

#[cfg(all(
    feature = "hil-replay",
    not(feature = "hil-dual"),
    not(feature = "hil-single")
))]
compile_error!("hil-replay requires hil-dual or hil-single");

pub mod gps_stub;
pub mod pyro_monitor;
pub mod sensor_replay;
