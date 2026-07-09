//! Log pyro fire requests without driving GPIO.

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;

use crate::{
    tasks::pyro_task::ContinuityUpdate, ContinuityWatch, FireSignal,
};

fn continuity(
    pyro_main_fire: bool,
    pyro_drogue_fire: bool,
) -> ContinuityUpdate {
    ContinuityUpdate {
        pyro_main_continuity: true,
        pyro_main_fire,
        pyro_drogue_continuity: true,
        pyro_drogue_fire,
        short_circuit: false,
    }
}

#[embassy_executor::task]
pub async fn hil_pyro_monitor(
    continuity_watch: &'static ContinuityWatch,
    fire_signal: &'static FireSignal,
) {
    info!("HIL: pyro_monitor started (NO GPIO — safe for desk use)");

    // Fake healthy continuity so telemetry fields stay sane.
    continuity_watch.sender().send(continuity(false, false));

    loop {
        let pyro = fire_signal.receive().await;
        let t_ms = Instant::now().as_millis();
        match pyro {
            PyroSelect::PyroDrogue => {
                info!("HIL: FIRE PyroDrogue at t={} ms", t_ms);
                continuity_watch.sender().send(continuity(false, true));
                Timer::after(Duration::from_millis(3000)).await;
                continuity_watch.sender().send(continuity(false, false));
            }
            PyroSelect::PyroMain => {
                info!("HIL: FIRE PyroMain at t={} ms", t_ms);
                continuity_watch.sender().send(continuity(true, false));
                Timer::after(Duration::from_millis(3000)).await;
                continuity_watch.sender().send(continuity(false, false));
            }
        }
    }
}
