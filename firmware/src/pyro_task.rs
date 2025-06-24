use core::cell::RefCell;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    peripherals::{EXTI12, EXTI13, PD13, PD8, PD9, PE12, PE13, PE9},
    Peri,
};
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, Mutex},
    signal, watch,
};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::vlp::packets::fire_pyro::PyroSelect;
use futures::join;

#[derive(Clone, defmt::Format, PartialEq, Eq)]
pub struct ContinuityUpdate {
    pub pyro1_continuity: bool,
    pub pyro1_fire: bool,
    pub pyro2_continuity: bool,
    pub pyro2_fire: bool,
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

    continuity_update: watch::DynSender<'static, ContinuityUpdate>,
    fire_signal: &'static signal::Signal<NoopRawMutex, PyroSelect>,
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
            pyro1_continuity: pyro1_cont.is_low(),
            pyro1_fire: false,
            pyro2_continuity: pyro2_cont.is_low(),
            pyro2_fire: false,
            short_circuit: false,
        }));

    let pyro1_cont_fut = async {
        let mut ticker = Ticker::every(Duration::from_millis(250));
        loop {
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.pyro1_continuity = pyro1_cont.is_low();
                continuity_update.send(state.clone());
            });
            ticker.next().await;
        }
    };

    let pyro2_cont_fut = async {
        loop {
            pyro2_cont.wait_for_any_edge().await;
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.pyro2_continuity = pyro2_cont.is_low();
                continuity_update.send(state.clone());
            });
        }
    };

    let pyro_pg_fut = async {
        loop {
            pyro_pg.wait_for_any_edge().await;
            state.lock(|state| {
                let mut state = state.borrow_mut();
                state.short_circuit = pyro_pg.is_high();
                continuity_update.send(state.clone());
            });
        }
    };

    let fire_fut = async {
        loop {
            let pyro = fire_signal.wait().await;
            match pyro {
                PyroSelect::Pyro1 => {
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro1_fire = true;
                        continuity_update.send(state.clone());
                    });
                    pyro1_ctrl.set_high();
                    Timer::after(Duration::from_millis(3000)).await;
                    pyro1_ctrl.set_low();
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro1_fire = false;
                        continuity_update.send(state.clone());
                    });
                }
                PyroSelect::Pyro2 => {
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro2_fire = true;
                        continuity_update.send(state.clone());
                    });
                    pyro2_ctrl.set_high();
                    Timer::after(Duration::from_millis(3000)).await;
                    pyro2_ctrl.set_low();
                    state.lock(|state| {
                        let mut state = state.borrow_mut();
                        state.pyro2_fire = false;
                        continuity_update.send(state.clone());
                    });
                }
            }
        }
    };

    join!(pyro1_cont_fut, pyro2_cont_fut, pyro_pg_fut, fire_fut);
}
