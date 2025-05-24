// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod time;
mod utils;

use {defmt_rtt_pipe as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::time::mhz;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;
        let mut config = embassy_stm32::Config::default();
        let rcc = &mut config.rcc;

        rcc.hsi = None;
        rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });
        rcc.csi = false;

        rcc.hsi48 = None;
        rcc.sys = Sysclk::PLL1_P;

        rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL60,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV20),
            divr: None,
        });
        rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: Some(PllDiv::DIV4),
            divq: Some(PllDiv::DIV4),
            divr: Some(PllDiv::DIV4),
        });
        rcc.pll3 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllDiv::DIV8),
        });

        rcc.d1c_pre = AHBPrescaler::DIV1;
        rcc.ahb_pre = AHBPrescaler::DIV2;
        rcc.apb1_pre = APBPrescaler::DIV2;
        rcc.apb2_pre = APBPrescaler::DIV2;
        rcc.apb3_pre = APBPrescaler::DIV2;
        rcc.apb4_pre = APBPrescaler::DIV2;

        rcc.timer_prescaler = TimerPrescaler::DefaultX2;
        rcc.voltage_scale = VoltageScale::Scale0;

        rcc.ls = LsConfig::default_lsi();
        rcc.mux.spi123sel = Saisel::PLL2_P;
        rcc.mux.usart16910sel = Usart16910sel::PLL2_Q;
        rcc.mux.rngsel = Rngsel::PLL1_Q;
        rcc.mux.i2c1235sel = I2c1235sel::PLL3_R;
        rcc.mux.spi45sel = Spi45sel::PLL2_Q;
        rcc.mux.adcsel = Adcsel::PLL2_P;
        rcc.mux.usbsel = Usbsel::PLL1_Q;
        rcc.mux.fdcansel = Fdcansel::PLL2_Q;
        rcc.mux.sdmmcsel = Sdmmcsel::PLL2_R;

        config
    };
    let p = embassy_stm32::init(config);
    info!("Hello VLF5!");
    // PS: PA3 (low: force pwm)
    // red_led: PB1
    // green_led: PA7
    // blue_led: PA2
    let mut red_led = Output::new(p.PB1, Level::Low, Speed::Low);

    // https://www.notion.so/mcmasterrocketry/VLF5-1c0d3a029ea580f882dfee3f98b0b897?pvs=4#1ebd3a029ea5807e8651fe9f530ff869
    let mut pyro_n_en = Output::new(p.PE9, Level::Low, Speed::Low);
    let mut pyro_pg = ExtiInput::new(p.PE13, p.EXTI13, Pull::Up);

    // high: enable pyro
    let mut pyro1_ctrl = Output::new(p.PD8, Level::High, Speed::Low);
    let mut pyro1_cont = Input::new(p.PD13, Pull::Up);

    let mut pyro2_ctrl = Output::new(p.PD9, Level::High, Speed::Low);
    let mut pyro2_cont = ExtiInput::new(p.PE12, p.EXTI12, Pull::Up);

    loop {
        pyro_pg.wait_for_any_edge().await;
        // power good: low
        red_led.set_level(pyro_pg.get_level());
        // info!("pyro1_cont: {}", pyro1_cont.is_low());
        // info!("pyro2_cont: {}", pyro2_cont.is_low());
    }
}
