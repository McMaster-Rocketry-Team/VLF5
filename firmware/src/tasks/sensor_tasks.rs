use crate::{
    AvionicsModeWatch, VLStatusMutex,
    avionics_mode::AvionicsMode,
    drivers::{lis2mdl::LIS2MDL, lsm6dsm::LSM6DSM, ms5607::MS5607},
};
use cortex_m::singleton;
use defmt::{error, info};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_stm32::{
    Peri,
    adc::{self, Adc, AdcChannel as _},
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    i2c::{self, Config as I2cConfig, Error as I2cError, I2c},
    peripherals::{
        ADC1, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, DMA2_CH0, DMA2_CH1, EXTI14, I2C2, PA5, PA6,
        PB0, PB10, PB11, PC6, PC13, PC14, PD7, PE2, PE5, PE6, SPI1, SPI4,
    },
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{DynPublisher, PubSubChannel},
    watch::Watch,
};
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_async::i2c::I2c as HalI2c;
use embedded_hal_async::spi::SpiDevice;
use firmware_common_new::{
    readings::{BaroData, IMUData, MagData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

pub type IMUBaroReadingPubSub = PubSubChannel<
    CriticalSectionRawMutex,
    SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>,
    10,
    4,
    1,
>;

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

    baro_spi1: Peri<'static, SPI1>,
    baro_sck: Peri<'static, PA5>,
    baro_mosi: Peri<'static, PD7>,
    baro_miso: Peri<'static, PA6>,
    baro_cs: Peri<'static, PC6>,
    baro_tx_dma: Peri<'static, DMA1_CH4>,
    baro_rx_dma: Peri<'static, DMA1_CH5>,

    pubsub: &'static IMUBaroReadingPubSub,

    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
        s.baro_ok = true;
    });

    let result = try {
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = Hertz(1_000_000);

        let spi4 = singleton!(: Mutex<NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>> = Mutex::new(Spi::new(
            imu_spi4,
            imu_sck,
            imu_mosi,
            imu_miso,
            imu_tx_dma,
            imu_rx_dma,
            spi_config,
        ))).unwrap();
        let imu_spi_device = SpiDeviceWithConfig::new(
            spi4,
            Output::new(imu_cs, Level::High, Speed::High),
            spi_config,
        );
        let mut imu = LSM6DSM::new(imu_spi_device);
        imu.reset().await.map_err(IMUOrBaroError::IMU)?;
        let mut imu_int1 = ExtiInput::new(imu_int1, imu_int1_exti, Pull::None);
        info!("IMU initialized");

        let mut spi_config = SpiConfig::default();
        spi_config.frequency = Hertz(1_000_000);
        let spi1 = singleton!(: Mutex<NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>> = Mutex::new(Spi::new(
            baro_spi1,
            baro_sck,
            baro_mosi,
            baro_miso,
            baro_tx_dma,
            baro_rx_dma,
            spi_config,
        ))).unwrap();
        let baro_spi_device = SpiDeviceWithConfig::new(
            spi1,
            Output::new(baro_cs, Level::High, Speed::High),
            spi_config,
        );
        let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
        let mut baro = MS5607::new(baro_spi_device, baro_buffer);
        baro.reset().await.map_err(IMUOrBaroError::Baro)?;
        info!("Barometer initialized");

        let publisher = pubsub.dyn_publisher().unwrap();
        loop {
            match avionics_mode.get().await {
                AvionicsMode::Armed | AvionicsMode::SelfTest => {
                    imu.power_up().await.map_err(IMUOrBaroError::IMU)?;
                    match select(
                        read_imu_baro_loop(&mut imu_int1, &mut imu, &mut baro, &publisher),
                        avionics_mode.changed_and(|m| {
                            *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest
                        }),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(e)?,
                        Either::Second(_) => {}
                    };
                    imu.power_down().await.map_err(IMUOrBaroError::IMU)?;
                }
                AvionicsMode::LowPower | AvionicsMode::Demo => {
                    match select(
                        read_baro_low_power_loop(&mut baro, &publisher),
                        avionics_mode.changed(),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(IMUOrBaroError::Baro(e))?,
                        Either::Second(_) => {},
                    };
                }
                AvionicsMode::Landed => {
                    avionics_mode.changed().await;
                }
            }
        }
    };

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();

        match result {
            Err(IMUOrBaroError::Baro(baro_error)) => {
                error!("baro error: {}", baro_error);
                s.baro_ok = false;
            }
            Err(IMUOrBaroError::IMU(imu_error)) => {
                error!("imu error: {}", imu_error);
                s.imu_ok = false;
            }
            Err(IMUOrBaroError::IMUAndBaro(imu_error, baro_error)) => {
                error!("baro error: {}", baro_error);
                s.baro_ok = false;
                error!("imu error: {}", imu_error);
                s.imu_ok = false;
            }
        }
    });
}

enum IMUOrBaroError<I: SpiDevice, B: SpiDevice> {
    IMU(I::Error),
    Baro(B::Error),
    IMUAndBaro(I::Error, B::Error),
}

async fn read_baro_low_power_loop<B: SpiDevice>(
    baro: &mut MS5607<'static, B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
) -> Result<!, B::Error> {
    let mut ticker = Ticker::every(Duration::from_hz(5));

    loop {
        let reading = baro.read().await?;
        publisher.publish_immediate(SensorReading::new(
            reading.timestamp_us,
            (None, reading.data),
        ));
        ticker.next().await;
    }
}

async fn read_imu_baro_loop<I: SpiDevice, B: SpiDevice>(
    imu_int1: &mut ExtiInput<'static>,
    imu: &mut LSM6DSM<I>,
    baro: &mut MS5607<'static, B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
) -> Result<!, IMUOrBaroError<I, B>> {
    loop {
        imu_int1.wait_for_rising_edge().await;
        match join(imu.read(), baro.read()).await {
            (Ok(imu_reading), Ok(baro_reading)) => publisher.publish_immediate(SensorReading::new(
                imu_reading.timestamp_us,
                (Some(imu_reading.data), baro_reading.data),
            )),
            (Ok(_), Err(baro_error)) => Err(IMUOrBaroError::Baro(baro_error))?,
            (Err(imu_error), Ok(_)) => Err(IMUOrBaroError::IMU(imu_error))?,
            (Err(imu_error), Err(baro_error)) => {
                Err(IMUOrBaroError::IMUAndBaro(imu_error, baro_error))?
            }
        };
    }
}

pub type MagReadingPubSub =
    PubSubChannel<CriticalSectionRawMutex, SensorReading<BootTimestamp, MagData>, 10, 2, 1>;

#[embassy_executor::task]
pub async fn mag_task(
    i2c: Peri<'static, I2C2>,
    scl: Peri<'static, PB10>,
    sda: Peri<'static, PB11>,
    tx_dma: Peri<'static, DMA1_CH7>,
    rx_dma: Peri<'static, DMA1_CH6>,

    pubsub: &'static MagReadingPubSub,

    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.mag_ok = true;
    });

    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();

    let result: Result<!, I2cError> = try {
        bind_interrupts!(struct Irqs {
            I2C2_EV => i2c::EventInterruptHandler<I2C2>;
            I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
        });

        let mut config = I2cConfig::default();
        config.sda_pullup = true;
        config.scl_pullup = true;
        config.frequency = Hertz(400_000);
        let i2c: I2c<'_, embassy_stm32::mode::Async, i2c::Master> =
            I2c::new(i2c, scl, sda, Irqs, tx_dma, rx_dma, config);
        let mut mag = LIS2MDL::new(i2c);
        mag.reset().await?;
        info!("Magnetometer initialized");

        let publisher = pubsub.dyn_publisher().unwrap();
        loop {
            match avionics_mode.get().await {
                AvionicsMode::Armed | AvionicsMode::SelfTest => {
                    mag.power_up().await?;
                    match select(
                        read_mag_loop(&mut mag, &publisher),
                        avionics_mode.changed_and(|m| {
                            *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest
                        }),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(e)?,
                        Either::Second(_) => {}
                    };
                    mag.power_down().await?;
                }
                AvionicsMode::LowPower | AvionicsMode::Demo | AvionicsMode::Landed => {
                    avionics_mode.changed().await;
                }
            }
        }
    };

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();

        let error = result.unwrap_err();
        error!("mag error: {}", error);
        s.mag_ok = false;
    });
}

async fn read_mag_loop<B: HalI2c>(
    mag: &mut LIS2MDL<B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, MagData>>,
) -> Result<!, B::Error> {
    let mut ticker = Ticker::every(Duration::from_hz(100));

    loop {
        let reading = mag.read().await?;
        publisher.publish_immediate(reading);
        ticker.next().await;
    }
}

pub type BatteryVWatch = Watch<NoopRawMutex, SensorReading<BootTimestamp, f32>, 3>;

#[embassy_executor::task]
pub async fn adc_task(
    adc1: Peri<'static, ADC1>,
    battery_v_pin: Peri<'static, PB0>,
    battery_v_watch: &'static BatteryVWatch,
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

    let sender = battery_v_watch.sender();
    loop {
        ticker.next().await;
        sender.send(SensorReading::new(
            Instant::now().as_micros(),
            read_battery_voltage(),
        ));
    }
}
