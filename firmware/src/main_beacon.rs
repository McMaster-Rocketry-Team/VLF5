// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod fmt;
mod time;
mod utils;

use {defmt_rtt as _, panic_probe as _};

use cortex_m::singleton;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    time::mhz,
    usart::{self, BufferedUart, Config as UartConfig},
};
use firmware_common_new::run_gps_uart_receiver;
use time::Clock;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    spawner.must_spawn(gps_task(p.USART1, p.PA10, p.PB14));
}

#[embassy_executor::task]
async fn gps_task(usart1: USART1, pa10: PA10, pb14: PB14) {
    bind_interrupts!(struct Irqs {
        USART1 => usart::BufferedInterruptHandler<USART1>;
    });

    let tx_buffer = singleton!(: [u8; 32] = [0; 32]).unwrap();
    let rx_buffer = singleton!(: [u8; 32] = [0; 32]).unwrap();
    let mut config = UartConfig::default();
    config.baudrate = 9600;

    let mut uart1 =
        BufferedUart::new(usart1, Irqs, pa10, pb14, tx_buffer, rx_buffer, config).unwrap();

    run_gps_uart_receiver(&mut uart1, Clock, |gps_data| {
        log_info!("GPS Data: {:?}", gps_data);
    })
    .await;
}
