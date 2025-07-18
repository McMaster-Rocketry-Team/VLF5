// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod clock_config;
mod ms5607;
mod time;
mod utils;

use core::mem;

use crate::{clock_config::vlf5_clock_config};

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{
        PA7, PB1,
    };
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH4, DMA1_CH5, PA2, PA5, PA6, PC6, PD7, SPI1};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex, watch};
use embassy_time::{Duration, Ticker, Timer};
use ms5607::MS5607;

/// drogue must deploy above this altitude
const DROGUE_CHUTE_MIN_AGL_M: f32 = 3000f32;

/// main chute will deploy once the rocket descents to this altitude
const MAIN_CHUTE_AGL_M: f32 = 457.2f32; // 1500ft

/// This program is only intended for altimeter test. The logic here for deploying main and drogue
/// chutes is different from the actual logic used in the rocket. This is because the actual logic
/// uses imu for more accurate measurements, which we can't recreate under the test conditions of
/// the altimeter test (vacuum chamber only)
///
/// This program sends current continuity, altitude and temperature to GCM over lora using
/// `AltimeterTelemetryPacket` every second
///
/// Blue led blinks means powered on
/// Green led on means drogue deployed
/// Red led on means main deployed
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

    // PS: PA3 (low: force pwm)
    let ps = Output::new(p.PA3, Level::Low, Speed::Low);
    mem::forget(ps); // forget ps pin so it does not get reset to Hi-Z when main function finishes

    // (temperature, asl altitude)
    let baro_data =
        singleton!(: watch::Watch<NoopRawMutex, (f32, f32), 1> = watch::Watch::new()).unwrap();

    spawner.must_spawn(power_led_task(p.PA2));

    spawner.must_spawn(altimeter_task(
        p.SPI1,
        p.PA5,
        p.PD7,
        p.PA6,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        p.PA7,
        p.PB1,
        baro_data.sender(),
    ));
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

#[embassy_executor::task]
async fn altimeter_task(
    spi1: Peri<'static, SPI1>,
    sck: Peri<'static, PA5>,
    mosi: Peri<'static, PD7>,
    miso: Peri<'static, PA6>,
    cs: Peri<'static, PC6>,
    tx_dma: Peri<'static, DMA1_CH4>,
    rx_dma: Peri<'static, DMA1_CH5>,
    green_led: Peri<'static, PA7>,
    red_led: Peri<'static, PB1>,
    baro_data: watch::Sender<'static, NoopRawMutex, (f32, f32), 1>,
) {
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);
    let mut red_led = Output::new(red_led, Level::High, Speed::Low);

    // baro
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(250_000);
    let spi1 =
        Mutex::<NoopRawMutex, _>::new(Spi::new(spi1, sck, mosi, miso, tx_dma, rx_dma, spi_config));
    let baro_spi_device =
        SpiDeviceWithConfig::new(&spi1, Output::new(cs, Level::High, Speed::High), spi_config);
    let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
    let mut baro = MS5607::new(baro_spi_device, baro_buffer);
    baro.reset().await.unwrap();
    info!("Barometer initialized");

    let sampling_hz = 200;
    // let cut_off_freq = (1).hz();

    // let mut lowpass = DirectForm2Transposed::<f32>::new(
    //     Coefficients::<f32>::from_params(
    //         Type::LowPass,
    //         (sampling_hz as f32).hz(),
    //         cut_off_freq,
    //         Q_BUTTERWORTH_F32,
    //     )
    //     .unwrap(),
    // );

    let mut ticker = Ticker::every(Duration::from_hz(sampling_hz));

    enum State {
        Init {
            count: usize,
        },
        Ascent {
            ground_altitude_m: f32,
            max_altitude_m: f32,
        },
        DrogueDescent {
            ground_altitude_m: f32,
        },
        MainDescent {
            ground_altitude_m: f32,
        },
    }
    let mut state = State::Init { count: 0 };
    loop {
        let baro_measurement = baro.read().await.unwrap().data;
        let altitude = baro_measurement.altitude();
        // let altitude = lowpass.run(altitude);

        match &mut state {
            State::Init { count } => {
                if *count < 200 {
                    // let low pass filter do its thing
                    *count += 1;
                } else {
                    info!("ground altitude: {}m", altitude);
                    state = State::Ascent {
                        ground_altitude_m: altitude,
                        max_altitude_m: 0.0,
                    }
                }
            }
            State::Ascent {
                ground_altitude_m,
                max_altitude_m,
            } => {
                let altitude_agl = altitude - *ground_altitude_m;
                // info!(
                //     "altitude agl: {}m, ground altitude: {}m",
                //     altitude_agl, *ground_altitude_m
                // );

                baro_data.send((baro_measurement.temperature, altitude_agl));

                *max_altitude_m = max_altitude_m.max(altitude_agl);
                if altitude_agl > DROGUE_CHUTE_MIN_AGL_M && altitude_agl < *max_altitude_m - 10.0 {
                    state = State::DrogueDescent {
                        ground_altitude_m: *ground_altitude_m,
                    };
                    green_led.set_low();
                }
            }
            State::DrogueDescent { ground_altitude_m } => {
                let altitude_agl = altitude - *ground_altitude_m;

                baro_data.send((baro_measurement.temperature, altitude_agl));

                if altitude_agl < MAIN_CHUTE_AGL_M {
                    state = State::MainDescent {
                        ground_altitude_m: *ground_altitude_m,
                    };
                    red_led.set_low();
                }
            }
            State::MainDescent { ground_altitude_m } => {
                let altitude_agl = altitude - *ground_altitude_m;

                baro_data.send((baro_measurement.temperature, altitude_agl));
            }
        }

        ticker.next().await;
    }
}
