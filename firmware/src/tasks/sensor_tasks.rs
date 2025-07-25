use crate::{avionics_mode::AvionicsMode, lsm6dsm::LSM6DSM, ms5607::MS5607};
use cortex_m::singleton;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_stm32::{
    Peri,
    adc::{self, Adc, AdcChannel as _},
    gpio::{Level, Output, Speed},
    peripherals::{
        ADC1, DMA1_CH4, DMA1_CH5, DMA2_CH0, DMA2_CH1, PA5, PA6, PB0, PC6, PC13, PD7, PE2, PE5, PE6,
        SPI1, SPI4,
    },
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    watch,
};
use embassy_time::{Duration, Instant, Ticker};
use firmware_common_new::{
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

#[embassy_executor::task]
pub async fn imu_task(
    spi4: Peri<'static, SPI4>,
    sck: Peri<'static, PE2>,
    mosi: Peri<'static, PE6>,
    miso: Peri<'static, PE5>,
    cs: Peri<'static, PC13>,
    tx_dma: Peri<'static, DMA2_CH1>,
    rx_dma: Peri<'static, DMA2_CH0>,
    imu_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    mut avionics_mode: watch::Receiver<'static, CriticalSectionRawMutex, AvionicsMode, 10>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi4 =
        Mutex::<NoopRawMutex, _>::new(Spi::new(spi4, sck, mosi, miso, tx_dma, rx_dma, spi_config));
    let low_g_imu_spi_device =
        SpiDeviceWithConfig::new(&spi4, Output::new(cs, Level::High, Speed::High), spi_config);
    let mut low_g_imu = LSM6DSM::new(low_g_imu_spi_device);
    low_g_imu.reset().await.unwrap();
    info!("IMU initialized");

    avionics_mode.get().await;
    let mut ticker = Ticker::every(Duration::from_hz(200));
    loop {
        let mode = avionics_mode.try_get().unwrap();
        if mode.sensors_active() {
            imu_reading_sender.send(low_g_imu.read().await.unwrap());
            ticker.next().await;
        } else {
            // TODO: put sensor to sleep

            // wait until sensors active
            while !avionics_mode.get().await.sensors_active() {}

            // TODO: enable sensor
        }
    }
}

#[embassy_executor::task]
pub async fn baro_task(
    spi1: Peri<'static, SPI1>,
    sck: Peri<'static, PA5>,
    mosi: Peri<'static, PD7>,
    miso: Peri<'static, PA6>,
    cs: Peri<'static, PC6>,
    tx_dma: Peri<'static, DMA1_CH4>,
    rx_dma: Peri<'static, DMA1_CH5>,
    baro_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    mut avionics_mode: watch::Receiver<'static, CriticalSectionRawMutex, AvionicsMode, 10>,
) {
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

    avionics_mode.get().await;
    let mut ticker = Ticker::every(Duration::from_hz(200));
    loop {
        let mode = avionics_mode.try_get().unwrap();
        // TODO: also read sensor during low power mode
        if mode.sensors_active() {
            baro_reading_sender.send(baro.read().await.unwrap());
            ticker.next().await;
        } else {
            // TODO: put sensor to sleep

            // wait until sensors active
            while !avionics_mode.get().await.sensors_active() {}

            // TODO: enable sensor
        }
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
