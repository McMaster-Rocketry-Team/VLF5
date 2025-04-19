#![no_std]
#![no_main]


mod utils;

use core::{fmt::Debug, marker::PhantomData};

use {defmt_rtt as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use dspower_servo::DSPowerServo;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Pull, Speed},
    mode::Async,
    peripherals,
    spi::{Config as SpiConfig, Spi},
    time::{mhz, Hertz},
    usart::{
        self, BufferedUart, Config as UartConfig, Error as UartError, HalfDuplexConfig,
        HalfDuplexReadback, Uart,
    },
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_alloc::Heap;
use embedded_io_async::{ErrorType, Read, Write};
use firmware_common::driver::{barometer::Barometer, imu::IMU};


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


}
