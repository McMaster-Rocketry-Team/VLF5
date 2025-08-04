// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod clock_config;

use crate::{bootloader::watchdog_task, clock_config::vlf5_clock_config, lsm6dsm::LSM6DSM};

use {defmt_rtt_pipe as _, panic_probe as _};

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};

mod bootloader;
mod lsm6dsm;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

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

    // spawner.must_spawn(watchdog_task(p.IWDG1));

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        let reading = low_g_imu.read().await.unwrap();
        info!("gyro z: {}", reading.data.gyro.z);
        ticker.next().await;
    }
}
