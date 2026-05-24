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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, watch};
use embassy_time::{Duration, Instant, Ticker};
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

    let mut buf: [u8; 512] = [0x0; 512];

    let mut current_index = 0;

    // Note for size requirements, flight is ~1 min

    // data comes at 400 hz already

    // tag byte outside struct

    // add timestamp too

    // amp_online
    // amp_rebooted_inlast 5 secs

    // Telemetry Packet

    // merge into bigger structs


    // up to ozys

    // look into stuff like actual_air breakes percentage which arent actually ticker 10hz

    // still do 10hz for ticker

    // look into ozys fs (two block system at beginning)

    // write new data to second block then erase first
    // when reading read both blocks and take the newest
    // write checksums to both blocks

    // write checksums for every block

    // DONT OVERWRITE DATA




    let mut hight_rate_ticker = Ticker::every(Duration::from_hz(400));
    loop {
        match avionics_mode.get().await {
            crate::avionics_mode::AvionicsMode::Armed => {
                hight_rate_ticker.next().await;
                let current_timestamp = Instant::now().as_millis(); // will be used as an ID

                let imu_data_raw = imu_baro_sub.try_next_message();
                let mag_data_raw = mag_sub.try_next_message();
                let gps_data_raw = gps_receiver.try_changed();
                let battery_data_raw = battery_receiver.try_changed();

                // by this point, considering we waited 1/400 seconds, all readings should have updated, perhaps even multiple times idk.


                // skip non-read ones

                if imu_data_raw.is_some() {
                    match imu_data_raw.unwrap() {
                        embassy_sync::pubsub::WaitResult::Lagged(num) => {

                            // in this case just take the most recent reading
                        }
                        embassy_sync::pubsub::WaitResult::Message(sensor_data) => {



                        }
                    }
                }

                if mag_data_raw.is_some(){

                }
            }

            _ => {
                // Note, could also keep logging gps after landing?
            }
        }
    }
}

