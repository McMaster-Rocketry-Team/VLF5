// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod clock_config;
mod ms5607;
mod time;
mod utils;

use crate::clock_config::vlf5_clock_config;

use {defmt_rtt_pipe as _, panic_probe as _};

use biquad::{Biquad, Coefficients, DirectForm2Transposed, ToHertz as _, Type, Q_BUTTERWORTH_F32};
use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::PA2;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use ms5607::MS5607;

/// drogue must deploy above this altitude
const DROGUE_CHUTE_MIN_AGL_M: f32 = 3000f32;

// main chute will deploy once the rocket descents to this altitude
const MAIN_CHUTE_AGL_M: f32 = 457.2f32; // 1500ft

/// This program is only intended for altimeter test. The logic here for deploying main and drogue
/// chutes is different from the actual logic used in the rocket. This is because the actual logic
/// uses imu for more accurate measurements, which we can't recreate under the test conditions of
/// the altimeter test (vacuum chamber only)
///
/// Blue led blinks means powered on
/// Green led on means drogue deployed
/// Red led on means main deployed
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

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

    enum State {
        Init,
        Ascent {
            ground_altitude_m: f32,
            max_altitude_m: f32,
        },
        DrogueDescent {
            ground_altitude_m: f32,
        },
        MainDescent,
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
                if altitude > DROGUE_CHUTE_MIN_AGL_M + *ground_altitude_m && altitude < *max_altitude_m {
                    state = State::DrogueDescent {
                        ground_altitude_m: *ground_altitude_m,
                    };
                    green_led.set_low();
                }
            }
            State::DrogueDescent { ground_altitude_m } => {
                if altitude < MAIN_CHUTE_AGL_M + *ground_altitude_m {
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
