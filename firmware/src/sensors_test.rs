// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod e22;
mod lsm6dsm;
mod ms5607;
mod time;
mod utils;

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{mhz, Hertz};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use firmware_common_new::variance::VarianceEstimator;
use lsm6dsm::LSM6DSM;
use ms5607::MS5607;

// bind_interrupts!(struct Irqs {
//     ADC1 => adc::InterruptHandler<peripherals::ADC1>;
// });

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

    // PS: PA3 (low: force pwm)
    let mut _ps = Output::new(p.PA3, Level::Low, Speed::Low);
    // red_led: PB1
    // green_led: PA7
    // blue_led: PA2

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

    // Low G IMU
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi4 = Mutex::<NoopRawMutex, _>::new(Spi::new(
        p.SPI4, p.PE2, p.PE6, p.PE5, p.DMA2_CH1, p.DMA2_CH0, spi_config,
    ));
    let low_g_imu_spi_device = SpiDeviceWithConfig::new(
        &spi4,
        Output::new(p.PC13, Level::High, Speed::High),
        spi_config,
    );
    let mut low_g_imu = LSM6DSM::new(low_g_imu_spi_device);
    low_g_imu.reset().await.unwrap();
    info!("Low G IMU initialized");

    // let mut adc = Adc::new(p.ADC1);
    // adc.set_resolution(adc::Resolution::BITS12V);
    // let mut bat_v_m = p.PB0.degrade_adc();

    // adc.set_sample_time(SampleTime::CYCLES810_5);

    // let mut vrefint_channel = adc.enable_vrefint();

    let hz = 5000u32;
    let mut ticker = Ticker::every(Duration::from_hz(hz as u64));
    let mut variance_estimator = VarianceEstimator::<6>::new();
    let mut count = 0u32;
    loop {
        // let VREFINT = 1.21f32;
        // let vrefint = adc.blocking_read(&mut vrefint_channel);
        // info!("vrefint: {}", vrefint);
        // let ratio = VREFINT / vrefint as f32;

        // let measured = adc.blocking_read(&mut bat_v_m);
        // info!("measured: {}V", measured as f32 * ratio);

        // let baro_measurements = baro.read().await.unwrap();
        // info!(
        //     "Barometer measurements: {:?}, altitude: {}",
        //     baro_measurements,
        //     baro_measurements.data.altitude()
        // );
        let low_g_imu_measurements = low_g_imu.read().await.unwrap();
        // info!("Low G IMU measurements: {:?}", low_g_imu_measurements);
        let acc = low_g_imu_measurements.data.acc;
        let gyro = low_g_imu_measurements.data.gyro;
        variance_estimator.update([acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]]);

        count += 1;
        if count > 50000 {
            info!("mean: {:?}", variance_estimator.mean());
            info!("noise density: {:?}", variance_estimator.noise_density(hz as f32));
            break;
        }

        ticker.next().await;
    }
}
