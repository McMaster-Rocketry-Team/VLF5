use core::cell::RefCell;
use embassy_futures::join::join4;
use embassy_stm32::{
    Peri,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    peripherals::{EXTI12, EXTI13, PD8, PD9, PD13, PE9, PE12, PE13},
};
use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;

use crate::{ContinuityWatch, FireSignal};

#[derive(Clone, defmt::Format, PartialEq, Eq)]
pub struct ContinuityUpdate {
    pub pyro_main_continuity: bool,
    pub pyro_main_fire: bool,
    pub pyro_drogue_continuity: bool,
    pub pyro_drogue_fire: bool,
    pub short_circuit: bool,
}

#[embassy_executor::task]
pub async fn pyro_task(
    pyro_n_en: Peri<'static, PE9>,
    pyro_pg: Peri<'static, PE13>,
    pyro_pg_exti: Peri<'static, EXTI13>,

    pyro1_ctrl: Peri<'static, PD8>,
    pyro1_cont: Peri<'static, PD13>,
    pyro2_ctrl: Peri<'static, PD9>,
    pyro2_cont: Peri<'static, PE12>,
    pyro2_cont_exti: Peri<'static, EXTI12>,

    continuity_watch: &'static ContinuityWatch,
    fire_signal: &'static FireSignal,
) {
    // https://www.notion.so/mcmasterrocketry/VLF5-1c0d3a029ea580f882dfee3f98b0b897?pvs=4#1ebd3a029ea5807e8651fe9f530ff869
    let mut pyro_n_en = Output::new(pyro_n_en, Level::High, Speed::Low);
    let mut pyro_pg = ExtiInput::new(pyro_pg, pyro_pg_exti, Pull::Up);

    let mut pyro1_ctrl = Output::new(pyro1_ctrl, Level::Low, Speed::Low);
    let pyro1_cont = Input::new(pyro1_cont, Pull::Up);

    let mut pyro2_ctrl = Output::new(pyro2_ctrl, Level::Low, Speed::Low);
    let mut pyro2_cont = ExtiInput::new(pyro2_cont, pyro2_cont_exti, Pull::Up);

    pyro_n_en.set_low();
    Timer::after(Duration::from_millis(10)).await;

    let state =
        Mutex::<NoopRawMutex, RefCell<ContinuityUpdate>>::new(RefCell::new(ContinuityUpdate {
            pyro_main_continuity: pyro1_cont.is_low(),
            pyro_main_fire: false,
            pyro_drogue_continuity: pyro2_cont.is_low(),
            pyro_drogue_fire: false,
            short_circuit: false,
        }));

    let continuity_sender = continuity_watch.sender();
    let pyro1_cont_fut = async {
        let mut ticker = Ticker::every(Duration::from_millis(250));
        loop {
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.pyro_main_continuity = pyro1_cont.is_low();
                continuity_sender.send(state.clone());
            });
            ticker.next().await;
        }
    };

    let pyro2_cont_fut = async {
        loop {
            pyro2_cont.wait_for_any_edge().await;
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.pyro_drogue_continuity = pyro2_cont.is_low();
                continuity_sender.send(state.clone());
            });
        }
    };

    let pyro_pg_fut = async {
        loop {
            pyro_pg.wait_for_any_edge().await;
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.short_circuit = pyro_pg.is_high();
                continuity_sender.send(state.clone());
            });
        }
    };

    let fire_fut = async {
        loop {
            let pyro = fire_signal.wait().await;
            match pyro {
                PyroSelect::PyroMain => {
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro_main_fire = true;
                        continuity_sender.send(state.clone());
                    });
                    pyro1_ctrl.set_high();
                    Timer::after(Duration::from_millis(3000)).await;
                    pyro1_ctrl.set_low();
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro_main_fire = false;
                        continuity_sender.send(state.clone());
                    });
                }
                PyroSelect::PyroDrogue => {
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro_drogue_fire = true;
                        continuity_sender.send(state.clone());
                    });
                    pyro2_ctrl.set_high();
                    Timer::after(Duration::from_millis(3000)).await;
                    pyro2_ctrl.set_low();
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro_drogue_fire = false;
                        continuity_sender.send(state.clone());
                    });
                }
            }
        }
    };

    join4(pyro1_cont_fut, pyro2_cont_fut, pyro_pg_fut, fire_fut).await;
}
