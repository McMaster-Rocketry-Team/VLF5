// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod ms5607;
mod time;
mod utils;

use {defmt_rtt_pipe as _, panic_probe as _};

use biquad::{Biquad, Coefficients, DirectForm2Transposed, ToHertz as _, Type, Q_BUTTERWORTH_F32};
use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::PA2;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::Peri;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use ms5607::MS5607;

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

    // PS: PA3 (low: force pwm)
    let mut _ps = Output::new(p.PA3, Level::Low, Speed::Low);
    let mut green_led = Output::new(p.PA7, Level::High, Speed::Low);
    let mut red_led = Output::new(p.PB1, Level::High, Speed::Low);

    // baro
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(250_000);
    let spi1 = Mutex::<NoopRawMutex, _>::new(Spi::new(
        p.SPI1, p.PA5, p.PD7, p.PA6, p.DMA1_CH4, p.DMA1_CH5, spi_config,
    ));
    let baro_spi_device = SpiDeviceWithConfig::new(
        &spi1,
        Output::new(p.PC6, Level::High, Speed::High),
        spi_config,
    );
    let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
    let mut baro = MS5607::new(baro_spi_device, baro_buffer);
    baro.reset().await.unwrap();
    info!("Barometer initialized");

    spawner.must_spawn(power_led_task(p.PA2));

    let sampling_hz = 200;
    let cut_off_freq = (1).hz();

    let mut lowpass = DirectForm2Transposed::<f32>::new(
        Coefficients::<f32>::from_params(
            Type::LowPass,
            (sampling_hz as f32).hz(),
            cut_off_freq,
            Q_BUTTERWORTH_F32,
        )
        .unwrap(),
    );

    let mut ticker = Ticker::every(Duration::from_hz(sampling_hz));
    let min_drogue_agl_m = 3000f32;
    let drogue_agl_m = 426f32;

    enum State {
        Init,
        Ascent {
            ground_altitude_m: f32,
            max_altitude_m: f32,
        },
        DrogueDescent {
            ground_altitude_m: f32,
        },
        MainDescent {},
    }
    let mut state = State::Init;
    loop {
        let altitude = baro.read().await.unwrap().data.altitude();
        let altitude = lowpass.run(altitude);

        match &mut state {
            State::Init => {
                state = State::Ascent {
                    ground_altitude_m: altitude,
                    max_altitude_m: altitude,
                }
            }
            State::Ascent {
                ground_altitude_m,
                max_altitude_m,
            } => {
                *max_altitude_m = max_altitude_m.max(altitude);
                if altitude > min_drogue_agl_m + *ground_altitude_m && altitude < *max_altitude_m {
                    state = State::DrogueDescent {
                        ground_altitude_m: *ground_altitude_m,
                    };
                    green_led.set_low();
                }
            }
            State::DrogueDescent { ground_altitude_m } => {
                if altitude < drogue_agl_m + *ground_altitude_m {
                    state = State::MainDescent {};
                    red_led.set_low();
                }
            }
            State::MainDescent {} => {}
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn power_led_task(blue_led: Peri<'static, PA2>) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        blue_led.set_low();
        Timer::after_millis(50).await;
        blue_led.set_high();
        ticker.next().await;
    }
}
