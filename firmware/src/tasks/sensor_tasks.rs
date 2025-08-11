use core::cell::RefCell;

use crate::{avionics_mode::AvionicsMode, lsm6dsm::LSM6DSM, ms5607::MS5607};
use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_futures::{join::join, select::select};
use embassy_stm32::{
    Peri,
    adc::{self, Adc, AdcChannel as _},
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    peripherals::{
        ADC1, DMA1_CH4, DMA1_CH5, DMA2_CH0, DMA2_CH1, EXTI14, PA5, PA6, PB0, PC6, PC13, PC14, PD7,
        PE2, PE5, PE6, SPI1, SPI4,
    },
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::{
        Mutex as BlockingMutex,
        raw::{CriticalSectionRawMutex, NoopRawMutex},
    },
    mutex::Mutex,
    watch,
};
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_async::spi::SpiDevice;
use firmware_common_new::{
    can_bus::custom_status::vl_custom_status::VLCustomStatus,
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

#[embassy_executor::task]
pub async fn imu_baro_task(
    imu_spi4: Peri<'static, SPI4>,
    imu_sck: Peri<'static, PE2>,
    imu_mosi: Peri<'static, PE6>,
    imu_miso: Peri<'static, PE5>,
    imu_cs: Peri<'static, PC13>,
    imu_tx_dma: Peri<'static, DMA2_CH1>,
    imu_rx_dma: Peri<'static, DMA2_CH0>,
    imu_int1: Peri<'static, PC14>,
    imu_int1_exti: Peri<'static, EXTI14>,
    imu_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,

    baro_spi1: Peri<'static, SPI1>,
    baro_sck: Peri<'static, PA5>,
    baro_mosi: Peri<'static, PD7>,
    baro_miso: Peri<'static, PA6>,
    baro_cs: Peri<'static, PC6>,
    baro_tx_dma: Peri<'static, DMA1_CH4>,
    baro_rx_dma: Peri<'static, DMA1_CH5>,
    baro_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,

    vl_status: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>,
    mut avionics_mode: watch::Receiver<'static, CriticalSectionRawMutex, AvionicsMode, 10>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi4 = Mutex::<NoopRawMutex, _>::new(Spi::new(
        imu_spi4, imu_sck, imu_mosi, imu_miso, imu_tx_dma, imu_rx_dma, spi_config,
    ));
    let imu_spi_device = SpiDeviceWithConfig::new(
        &spi4,
        Output::new(imu_cs, Level::High, Speed::High),
        spi_config,
    );
    let mut imu = LSM6DSM::new(imu_spi_device);
    imu.reset().await.unwrap();
    let mut imu_int1 = ExtiInput::new(imu_int1, imu_int1_exti, Pull::None);
    info!("IMU initialized");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi1 = Mutex::<NoopRawMutex, _>::new(Spi::new(
        baro_spi1,
        baro_sck,
        baro_mosi,
        baro_miso,
        baro_tx_dma,
        baro_rx_dma,
        spi_config,
    ));
    let baro_spi_device = SpiDeviceWithConfig::new(
        &spi1,
        Output::new(baro_cs, Level::High, Speed::High),
        spi_config,
    );
    let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
    let mut baro = MS5607::new(baro_spi_device, baro_buffer);
    baro.reset().await.unwrap();
    info!("Barometer initialized");

    loop {
        match avionics_mode.get().await {
            AvionicsMode::Armed | AvionicsMode::SelfTest => {
                imu.power_up().await.unwrap();
                select(
                    read_imu_baro_loop(
                        &mut imu_int1,
                        &mut imu,
                        &imu_reading_sender,
                        &mut baro,
                        &baro_reading_sender,
                        vl_status,
                    ),
                    avionics_mode
                        .changed_and(|m| *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest),
                )
                .await;
                imu.power_down().await.unwrap();
            }
            AvionicsMode::LowPower => {
                select(
                    read_baro_low_power_loop(&mut baro, &baro_reading_sender, vl_status),
                    avionics_mode.changed(),
                )
                .await;
            }
            AvionicsMode::Landed => {
                avionics_mode.changed().await;
            }
        }
    }
}

async fn read_baro_low_power_loop(
    baro: &mut MS5607<'static, impl SpiDevice>,
    baro_reading_sender: &watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    vl_status: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>,
) {
    let mut baro_ok = true;
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
    });

    let mut ticker = Ticker::every(Duration::from_hz(5));

    loop {
        match baro.read().await {
            Ok(reading) => {
                baro_reading_sender.send(reading);
                if !baro_ok {
                    baro_ok = true;
                    vl_status.lock(|s| {
                        let mut s = s.borrow_mut();
                        s.baro_ok = true;
                    });
                }
            }
            Err(_) => {
                if baro_ok {
                    baro_ok = false;
                    vl_status.lock(|s| {
                        let mut s = s.borrow_mut();
                        s.baro_ok = false;
                    });
                }
            }
        }
        ticker.next().await;
    }
}

async fn read_imu_baro_loop(
    imu_int1: &mut ExtiInput<'static>,
    imu: &mut LSM6DSM<impl SpiDevice>,
    imu_reading_sender: &watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    baro: &mut MS5607<'static, impl SpiDevice>,
    baro_reading_sender: &watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    vl_status: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>,
) {
    let mut imu_ok = true;
    let mut baro_ok = true;
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
        s.baro_ok = true;
    });

    loop {
        imu_int1.wait_for_rising_edge().await;
        let read_imu_fut = async {
            match imu.read().await {
                Ok(reading) => {
                    imu_reading_sender.send(reading);
                    if !imu_ok {
                        imu_ok = true;
                        vl_status.lock(|s| {
                            let mut s = s.borrow_mut();
                            s.imu_ok = true;
                        });
                    }
                }
                Err(_) => {
                    if imu_ok {
                        imu_ok = false;
                        vl_status.lock(|s| {
                            let mut s = s.borrow_mut();
                            s.imu_ok = false;
                        });
                    }
                }
            }
        };

        let read_baro_but = async {
            match baro.read().await {
                Ok(reading) => {
                    baro_reading_sender.send(reading);
                    if !baro_ok {
                        baro_ok = true;
                        vl_status.lock(|s| {
                            let mut s = s.borrow_mut();
                            s.baro_ok = true;
                        });
                    }
                }
                Err(_) => {
                    if baro_ok {
                        baro_ok = false;
                        vl_status.lock(|s| {
                            let mut s = s.borrow_mut();
                            s.baro_ok = false;
                        });
                    }
                }
            }
        };

        join(read_imu_fut, read_baro_but).await;
    }
}

#[embassy_executor::task]
pub async fn adc_task(
    adc1: Peri<'static, ADC1>,
    battery_v_pin: Peri<'static, PB0>,
    battery_v_reading_sender: watch::Sender<
        'static,
        NoopRawMutex,
        SensorReading<BootTimestamp, f32>,
        1,
    >,
) {
    let mut adc = Adc::new(adc1);
    adc.set_resolution(adc::Resolution::BITS12V);
    adc.set_sample_time(adc::SampleTime::CYCLES810_5);
    let mut bat_v_m = battery_v_pin.degrade_adc();

    let mut read_battery_voltage = || {
        let vrefint = 1.21f32;
        // for some reason reading vrefint gives lower than expected value,
        // using a hard coded value instead
        // let vrefint_raw = adc.blocking_read(&mut vrefint_channel);
        let vrefint_raw = 1502u16;
        let ratio = vrefint / vrefint_raw as f32;

        // TODO: move to async?
        let pb0_raw = adc.blocking_read(&mut bat_v_m);
        let pb0 = pb0_raw as f32 * ratio;
        let batt_v = pb0 / 0.161;
        batt_v
    };

    let mut ticker = Ticker::every(Duration::from_hz(10));

    loop {
        ticker.next().await;
        battery_v_reading_sender.send(SensorReading::new(
            Instant::now().as_micros(),
            read_battery_voltage(),
        ));
    }
}
