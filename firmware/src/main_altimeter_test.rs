// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod clock_config;
mod e22;
mod ms5607;
mod time;
mod utils;

use core::mem;

use crate::{clock_config::vlf5_clock_config, e22::E22};

use {defmt_rtt_pipe as _, panic_probe as _};

use biquad::{Coefficients, DirectForm2Transposed, ToHertz as _, Type, Q_BUTTERWORTH_F32};
use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_stm32::{
    adc::{self, Adc, AdcChannel as _},
    peripherals::{
        ADC1, DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA7, PA8, PB0, PB1, PB3, PB4, PC7, PD0, PD1, PD4,
        PD5, PD6,
    },
};
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    peripherals::SPI3,
};
use embassy_stm32::{
    gpio::Input,
    peripherals::{DMA1_CH4, DMA1_CH5, PA2, PA5, PA6, PC6, PD13, PD7, PD8, PD9, PE12, PE9, SPI1},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex, watch};
use embassy_time::{Delay, Duration, Ticker, Timer};
use firmware_common_new::vlp::{
    client::VLPAvionics,
    lora::LoraPhy,
    lora_config::LoraConfig,
    packets::{altimeter_telemetry::AltimeterTelemetryPacket, VLPDownlinkPacket},
};
use lora_phy::{
    iv::GenericSx126xInterfaceVariant,
    sx126x::{self, Sx126x},
    LoRa,
};
use ms5607::MS5607;

const VLP_KEY: [u8; 32] = [42u8; 32];

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

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    // (temperature, agl altitude)
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

    spawner.must_spawn(send_altimeter_packet_task(
        p.ADC1,
        p.PB0,
        p.PE9,
        p.PD8,
        p.PD13,
        p.PD9,
        p.PE12,
        vlp_avionics_client,
        baro_data.receiver().unwrap(),
    ));

    spawner.must_spawn(lora_daemon_task(
        vlp_avionics_client,
        p.SPI3,
        p.PB3,
        p.PD6,
        p.PB4,
        p.PC7,
        p.PD5,
        p.PD4,
        p.EXTI4,
        p.PD1,
        p.EXTI1,
        p.PD0,
        p.PA8,
        p.DMA1_CH3,
        p.DMA1_CH2,
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
            _ => {}
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn send_altimeter_packet_task(
    adc1: Peri<'static, ADC1>,
    bat_v_m: Peri<'static, PB0>,

    pyro_n_en: Peri<'static, PE9>,
    pyro1_ctrl: Peri<'static, PD8>,
    pyro1_cont: Peri<'static, PD13>,
    pyro2_ctrl: Peri<'static, PD9>,
    pyro2_cont: Peri<'static, PE12>,

    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    mut baro_data: watch::Receiver<'static, NoopRawMutex, (f32, f32), 1>,
) {
    let mut adc = Adc::new(adc1);
    adc.set_resolution(adc::Resolution::BITS12V);
    adc.set_sample_time(adc::SampleTime::CYCLES810_5);
    let mut bat_v_m = bat_v_m.degrade_adc();
    // let mut vrefint_channel = adc.enable_vrefint();

    let mut read_battery_voltage = || {
        let vrefint = 1.21f32;
        // for some reason reading vrefint gives lower than expected value,
        // using a hard coded value instead
        // let vrefint_raw = adc.blocking_read(&mut vrefint_channel);
        let vrefint_raw = 1502u16;
        let ratio = vrefint / vrefint_raw as f32;

        let bat_v_raw = adc.blocking_read(&mut bat_v_m);
        bat_v_raw as f32 * ratio / 0.161
    };

    // https://www.notion.so/mcmasterrocketry/VLF5-1c0d3a029ea580f882dfee3f98b0b897?pvs=4#1ebd3a029ea5807e8651fe9f530ff869
    let _pyro_n_en = Output::new(pyro_n_en, Level::Low, Speed::Low);

    // high: enable pyro
    let _pyro1_ctrl = Output::new(pyro1_ctrl, Level::Low, Speed::Low);
    let pyro1_cont = Input::new(pyro1_cont, Pull::Up);

    let _pyro2_ctrl = Output::new(pyro2_ctrl, Level::Low, Speed::Low);
    let pyro2_cont = Input::new(pyro2_cont, Pull::Up);

    baro_data.get().await;

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let main_cont = pyro1_cont.is_low();
        let drogue_cont = pyro2_cont.is_low();
        let (temperature, altitude) = baro_data.try_get().unwrap();
        let battery_voltage = read_battery_voltage();
        info!(
            "main cont: {}, drogue cont: {}, temp: {}C, agl altitude: {}m, batt v: {}V",
            main_cont, drogue_cont, temperature, altitude, battery_voltage
        );

        vlp_avionics_client.send(VLPDownlinkPacket::AltimeterTelemetry(
            AltimeterTelemetryPacket::new(
                main_cont,
                drogue_cont,
                battery_voltage,
                temperature,
                altitude,
            ),
        ));
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn lora_daemon_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    spi3: Peri<'static, SPI3>,
    sck: Peri<'static, PB3>,
    mosi: Peri<'static, PD6>,
    miso: Peri<'static, PB4>,
    cs: Peri<'static, PC7>,
    reset: Peri<'static, PD5>,
    dio1: Peri<'static, PD4>,
    dio1_exti: Peri<'static, EXTI4>,
    busy: Peri<'static, PD1>,
    busy_exti: Peri<'static, EXTI1>,
    txen: Peri<'static, PD0>,
    rxen: Peri<'static, PA8>,
    tx_dma: Peri<'static, DMA1_CH3>,
    rx_dma: Peri<'static, DMA1_CH2>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(250_000);
    let spi3 =
        Mutex::<NoopRawMutex, _>::new(Spi::new(spi3, sck, mosi, miso, tx_dma, rx_dma, spi_config));
    let lora_spi_device =
        SpiDeviceWithConfig::new(&spi3, Output::new(cs, Level::High, Speed::High), spi_config);

    let config = sx126x::Config {
        chip: E22,
        tcxo_ctrl: None,
        use_dcdc: false,
        rx_boost: true,
    };
    let iv = GenericSx126xInterfaceVariant::new(
        Output::new(reset, Level::High, Speed::Low),
        ExtiInput::new(dio1, dio1_exti, Pull::Down),
        ExtiInput::new(busy, busy_exti, Pull::Down),
        Some(Output::new(rxen, Level::High, Speed::High)),
        Some(Output::new(txen, Level::High, Speed::High)),
    )
    .unwrap();
    let sx1262 = Sx126x::new(lora_spi_device, iv, config);
    let mut lora = LoRa::new(sx1262, false, Delay).await.unwrap();
    info!("LoRa initialized");
    let mut lora = LoraPhy::new(
        &mut lora,
        LoraConfig {
            frequency: 915_100_000,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        },
    );
    let mut daemon = vlp_avionics_client.daemon(&mut lora, &VLP_KEY);
    daemon.run().await;
}
