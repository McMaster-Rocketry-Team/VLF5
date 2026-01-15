use crate::{
    AvionicsModeWatch,
    tasks::sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
};

use {defmt_rtt_pipe as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::time::mhz;
use embassy_stm32::{
    Peri,
    peripherals::{PC8, PC9, PC10, PC11, PC12, PD2, SDMMC1},
    sdmmc::DataBlock,
};
use embassy_stm32::{
    bind_interrupts, peripherals,
    rng::{self, Rng},
};
use embassy_stm32::{
    crc::{Config as CrcConfig, Crc, InputReverseConfig, PolySize},
    sdmmc::{self, Sdmmc},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use firmware_common_new::{gps::GPSData, sensor_reading::SensorReading, time::BootTimestamp};
#[embassy_executor::task]
pub async fn sd_card_task(
    sdmmc1: Peri<'static, SDMMC1>,
    clk_pin: Peri<'static, PC12>,
    cmd_pin: Peri<'static, PD2>,
    d0_pin: Peri<'static, PC8>,
    d1_pin: Peri<'static, PC9>,
    d2_pin: Peri<'static, PC10>,
    d3_pin: Peri<'static, PC11>,

    // Sensor data channels
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mag_pubsub: &'static MagReadingPubSub,
    gps_watch: &'static watch::Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        3,
    >,
    battery_watch: &'static BatteryVWatch,

    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    bind_interrupts!(struct Irqs {
        SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
    });

    let mut sdmmc = Sdmmc::new_4bit(
        sdmmc1,
        Irqs,
        clk_pin,
        cmd_pin,
        d0_pin,
        d1_pin,
        d2_pin,
        d3_pin,
        Default::default(),
    );

    info!("Configured clock: {}", sdmmc.clock().0);
    sdmmc.init_sd_card(mhz(1)).await.unwrap();
    info!("SD card initialised!");

    let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();
    let mut mag_sub = mag_pubsub.subscriber().unwrap();
    let mut gps_receiver = gps_watch.receiver().unwrap();
    let mut battery_receiver = battery_watch.receiver().unwrap();
    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();


    



}
