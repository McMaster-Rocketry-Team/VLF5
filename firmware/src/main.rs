#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]
#![feature(try_blocks)]

use core::cell::RefCell;

use {defmt_rtt_pipe as _, panic_probe as _};

use air_brakes_controller_core::{FlightProfile, RocketParameters};
use binary_macros::base64;
use cortex_m::singleton;
use cortex_m_rt::entry;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::{
    Peri, Peripherals,
    can::{CanRx, CanTx},
    gpio::{Level, Output, Speed},
    interrupt::{self, InterruptExt as _, Priority},
    peripherals::{PA2, PA7},
};
use embassy_sync::{
    blocking_mutex::{
        Mutex as BlockingMutex,
        raw::{CriticalSectionRawMutex, NoopRawMutex},
    },
    mutex::Mutex,
    pubsub::PubSubBehavior as _,
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        custom_status::vl_custom_status::VLCustomStatus,
        messages::{
            baro_measurement::BaroMeasurementMessage, imu_measurement::IMUMeasurementMessage,
            mag_measurement::MagMeasurementMessage, vl_status::FlightStage,
        },
        sender::CanSender,
    },
    gps::GPSData,
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::{client::VLPAvionics, lora_config::LoraConfig, packets::fire_pyro::PyroSelect},
};

use crate::{
    avionics_mode::AvionicsMode,
    bootloader::watchdog_task,
    can::{can_bus_broadcast_unix_time_task, init_can_bus, start_can_bus_low_prio_tasks},
    clock_config::vlf5_clock_config,
    modes::{
        armed_mode::armed_mode, demo_mode::demo_mode, landed_mode::landed_mode,
        low_power_mode::low_power_mode, self_test_mode::self_test_mode,
    },
    tasks::{
        amp_control_task::{AmpControlWatch, amp_control_task},
        buzzer_task::{BuzzerPubSub, BuzzerTone, buzzer_task},
        gps_task::gps_task,
        pyro_task::{ContinuityUpdate, pyro_task},
        sensor_tasks::{
            BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub, adc_task, imu_baro_task,
            mag_task,
        },
        unix_clock::{UnixClock, unix_clock_task},
        vlp_avionics_daemon_task::vlp_avionics_daemon_task,
    },
};
use receive_vlp_task::receive_vlp_task;

mod avionics_mode;
mod bootloader;
mod can;
mod can_central;
mod clock_config;
mod drivers;
mod modes;
mod receive_vlp_task;
mod tasks;
mod time;
mod utils;

static VLP_KEY: &[u8] = base64!("file:vlp.key");
const LORA_CONFIG: LoraConfig = LoraConfig {
    frequency: 920_000_000,
    sf: 12,
    bw: 250000,
    cr: 8,
    power: 22,
};
pub const MAIN_BULKHEAD_NODE_ID: u16 = 0x4FC;
pub const DROGUE_BULKHEAD_NODE_ID: u16 = 0x328;
pub const OZYS_1_NODE_ID: u16 = 0x2B2;
pub const OZYS_2_NODE_ID: u16 = 0x0E1;

pub const FLIGHT_PROFILE: FlightProfile = FlightProfile {
    ignition_detection_acc_threshold: 4.0 * 9.81,
    drogue_chute_minimum_time_us: 1_000_000,
    drogue_chute_minimum_altitude_agl: 2000.0,
    drogue_chute_delay_us: 0,
    main_chute_altitude_agl: 457.2,
    main_chute_delay_us: 0,
};

pub const ROCKET_PARAMETERS: RocketParameters = RocketParameters {
    burnout_mass: 17.607,
    cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
    reference_area: 0.008982476,
};

pub type AvionicsModeWatch = Watch<CriticalSectionRawMutex, AvionicsMode, 10>;
pub type GPSReadingWatch = Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, GPSData>, 3>;
pub type VLStatusMutex = BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>;
pub type FlightStageMutex = BlockingMutex<NoopRawMutex, RefCell<FlightStage>>;
pub type ContinuityWatch = Watch<NoopRawMutex, ContinuityUpdate, 1>;
pub type FireSignal = Signal<NoopRawMutex, PyroSelect>;
pub type SetTargetWatch = Watch<NoopRawMutex, f32, 1>;

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(vlf5_clock_config());

    let buzzer_pubsub = singleton!(: BuzzerPubSub = BuzzerPubSub::new()).unwrap();
    let avionics_mode = singleton!(: AvionicsModeWatch = AvionicsModeWatch::new()).unwrap();
    avionics_mode.sender().send(AvionicsMode::SelfTest);
    let imu_baro_reading_pubsub =
        singleton!(: IMUBaroReadingPubSub = IMUBaroReadingPubSub::new()).unwrap();
    let mag_reading_pubsub = singleton!(: MagReadingPubSub = MagReadingPubSub::new()).unwrap();

    let gps_reading_watch = singleton!(: GPSReadingWatch = GPSReadingWatch::new()).unwrap();
    let vl_status =
        singleton!(: VLStatusMutex = BlockingMutex::new(RefCell::new(VLCustomStatus::new())))
            .unwrap();
    let unix_clock = singleton!(: UnixClock = UnixClock::new()).unwrap();

    let (can_tx, can_rx) = init_can_bus(p.FDCAN2, p.PB5, p.PB6);

    static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
    #[embassy_stm32::interrupt]
    unsafe fn USART2() {
        unsafe { EXECUTOR_HIGH.on_interrupt() }
    }

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::USART2);
    spawner.must_spawn(high_prio_main(
        spawner,
        can_tx,
        buzzer_pubsub,
        avionics_mode,
        imu_baro_reading_pubsub,
        mag_reading_pubsub,
        gps_reading_watch,
        vl_status,
        unix_clock,
    ));

    let executor_low = singleton!(: Executor = Executor::new()).unwrap();
    executor_low.run(|spawner| {
        spawner.must_spawn(low_prio_main(
            spawner,
            can_tx,
            can_rx,
            buzzer_pubsub,
            avionics_mode,
            imu_baro_reading_pubsub,
            mag_reading_pubsub,
            gps_reading_watch,
            vl_status,
            unix_clock,
        ));
    })
}

#[embassy_executor::task]
async fn high_prio_main(
    spawner: SendSpawner,
    can_tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    buzzer_pubsub: &'static BuzzerPubSub,
    avionics_mode_watch: &'static AvionicsModeWatch,
    imu_baro_reading_pubsub: &'static IMUBaroReadingPubSub,
    mag_reading_pubsub: &'static MagReadingPubSub,
    gps_reading_watch: &'static GPSReadingWatch,
    vl_status: &'static VLStatusMutex,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    spawner.must_spawn(buzzer_task(p.PC15, buzzer_pubsub));
    spawner.must_spawn(imu_baro_task(
        p.SPI4,
        p.PE2,
        p.PE6,
        p.PE5,
        p.PC13,
        p.DMA2_CH1,
        p.DMA2_CH0,
        p.PC14,
        p.EXTI14,
        p.SPI1,
        p.PA5,
        p.PD7,
        p.PA6,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        imu_baro_reading_pubsub,
        vl_status,
        avionics_mode_watch,
    ));
    spawner.must_spawn(mag_task(
        p.I2C2,
        p.PB10,
        p.PB11,
        p.DMA1_CH7,
        p.DMA1_CH6,
        mag_reading_pubsub,
        vl_status,
        avionics_mode_watch,
    ));
    spawner.must_spawn(unix_clock_task(
        p.PA15,
        p.EXTI15,
        unix_clock,
        gps_reading_watch,
    ));
    spawner.must_spawn(can_bus_broadcast_unix_time_task(can_tx, unix_clock));
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    can_tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    can_rx: CanRx<'static>,
    buzzer_pubsub: &'static BuzzerPubSub,
    avionics_mode_watch: &'static AvionicsModeWatch,
    imu_baro_reading_pubsub: &'static IMUBaroReadingPubSub,
    mag_reading_pubsub: &'static MagReadingPubSub,
    gps_reading_watch: &'static GPSReadingWatch,
    vl_status: &'static VLStatusMutex,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    let mut ps = Output::new(p.PA3, Level::Low, Speed::Low);

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let flight_stage =
        singleton!(: FlightStageMutex = BlockingMutex::new(RefCell::new(FlightStage::Armed)))
            .unwrap();
    let battery_v_watch = singleton!(: BatteryVWatch = BatteryVWatch::new()).unwrap();

    let continuity_watch = singleton!(: ContinuityWatch = ContinuityWatch::new()).unwrap();
    let fire_signal = singleton!(: FireSignal = FireSignal::new()).unwrap();
    let amp_control_watch = singleton!(: AmpControlWatch = AmpControlWatch::new()).unwrap();
    let target_agl_signal = singleton!(:SetTargetWatch= SetTargetWatch::new()).unwrap();

    spawner.must_spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_reading_watch.receiver().unwrap(),
    ));
    spawner.must_spawn(periodic_beep_task(buzzer_pubsub));

    spawner.must_spawn(adc_task(p.ADC1, p.PB0, battery_v_watch));
    spawner.must_spawn(pyro_task(
        p.PE9,
        p.PE13,
        p.EXTI13,
        p.PD8,
        p.PD13,
        p.PD9,
        p.PE12,
        p.EXTI12,
        continuity_watch,
        fire_signal,
    ));
    spawner.must_spawn(gps_task(
        p.USART1,
        p.PA10,
        p.PB14,
        gps_reading_watch.sender(),
        vl_status,
    ));
    spawner.must_spawn(vlp_avionics_daemon_task(
        vlp_avionics_client,
        VLP_KEY.try_into().unwrap(),
        LORA_CONFIG.clone(),
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
    let (can_sender, can_receiver, can_central) = start_can_bus_low_prio_tasks(
        &spawner,
        can_tx,
        can_rx,
        flight_stage,
        battery_v_watch,
        vl_status,
    )
    .await;

    spawner.must_spawn(receive_vlp_task(
        vlp_avionics_client,
        avionics_mode_watch,
        fire_signal,
        target_agl_signal,
        buzzer_pubsub,
        can_sender,
        can_central,
    ));

    spawner.must_spawn(broadcast_imu_baro_measurement_task(
        imu_baro_reading_pubsub,
        can_sender,
        unix_clock,
    ));
    spawner.must_spawn(broadcast_mag_measurement_task(
        mag_reading_pubsub,
        can_sender,
        unix_clock,
    ));
    spawner.must_spawn(amp_control_task(can_sender, amp_control_watch));

    spawner.must_spawn(watchdog_task(p.IWDG1));

    buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
    buzzer_pubsub.publish_immediate(BuzzerTone::Mid(250, 100));
    buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));

    loop {
        match avionics_mode_watch.try_get().unwrap() {
            AvionicsMode::Armed => {
                ps.set_low();
                armed_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_sender,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    continuity_watch,
                    fire_signal,
                    target_agl_signal,
                    can_receiver.subscriber().unwrap(),
                    flight_stage,
                    amp_control_watch,
                    unix_clock,
                )
                .await;
            }
            AvionicsMode::SelfTest => {
                ps.set_high();
                self_test_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    vl_status,
                    amp_control_watch,
                    flight_stage,
                    continuity_watch,
                    can_receiver.subscriber().unwrap(),
                    buzzer_pubsub,
                )
                .await
            }
            AvionicsMode::LowPower => {
                ps.set_high();
                low_power_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    flight_stage,
                )
                .await
            }
            AvionicsMode::Landed => {
                ps.set_high();
                landed_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    flight_stage,
                )
                .await
            }
            AvionicsMode::Demo => {
                ps.set_high();
                demo_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_sender,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    flight_stage,
                )
                .await
            }
        }
    }
}

#[embassy_executor::task]
async fn periodic_beep_task(buzzer_pubsub: &'static BuzzerPubSub) {
    let buzzer_pub = buzzer_pubsub.immediate_publisher();
    let mut ticker = Ticker::every(Duration::from_millis(5000));

    loop {
        ticker.next().await;
        buzzer_pub.publish_immediate(BuzzerTone::Low(50, 50));
    }
}

#[embassy_executor::task]
async fn power_led_task(
    blue_led: Peri<'static, PA2>,
    green_led: Peri<'static, PA7>,
    mut gps_reading: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        3,
    >,
) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let gps_fixed = if let Some(gps_reading) = gps_reading.try_get() {
            gps_reading.data.lat_lon.is_some()
        } else {
            false
        };
        let led = if gps_fixed {
            &mut green_led
        } else {
            &mut blue_led
        };
        led.set_low();
        Timer::after_millis(50).await;
        led.set_high();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn broadcast_imu_baro_measurement_task(
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    can_sender: &'static CanSender<NoopRawMutex>,
    unix_clock: &'static UnixClock,
) {
    let mut sub = imu_baro_pubsub.subscriber().unwrap();
    let mut last_send_time_us = Instant::now().as_micros();
    loop {
        let reading = sub.next_message_pure().await;
        let now_us = Instant::now().as_micros();
        if now_us - last_send_time_us < 9000 {
            continue;
        }
        last_send_time_us = now_us;

        if let Some(imu_data) = reading.data.0 {
            can_sender.send(
                IMUMeasurementMessage::new(
                    unix_clock
                        .convert_to_unix_us(reading.timestamp_us)
                        .unwrap_or(reading.timestamp_us),
                    &imu_data.acc,
                    &imu_data.gyro,
                )
                .into(),
            );
        }

        let baro_data = reading.data.1;
        can_sender.send(
            BaroMeasurementMessage::new(
                unix_clock
                    .convert_to_unix_us(reading.timestamp_us)
                    .unwrap_or(reading.timestamp_us),
                baro_data.pressure,
                baro_data.temperature,
            )
            .into(),
        );
    }
}

#[embassy_executor::task]
async fn broadcast_mag_measurement_task(
    mag_pubsub: &'static MagReadingPubSub,
    can_sender: &'static CanSender<NoopRawMutex>,
    unix_clock: &'static UnixClock,
) {
    let mut sub = mag_pubsub.subscriber().unwrap();
    loop {
        let reading = sub.next_message_pure().await;

        can_sender.send(
            MagMeasurementMessage::new(
                unix_clock
                    .convert_to_unix_us(reading.timestamp_us)
                    .unwrap_or(reading.timestamp_us),
                &reading.data.mag.into(),
            )
            .into(),
        );
    }
}
