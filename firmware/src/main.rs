#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;

use {defmt_rtt_pipe as _, panic_probe as _};

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
    blocking_mutex::Mutex as BlockingMutex,
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{self, PubSubBehavior as _, PubSubChannel, Subscriber},
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        custom_status::vl_custom_status::VLCustomStatus,
        messages::{
            baro_measurement::BaroMeasurementMessage, imu_measurement::IMUMeasurementMessage,
            vl_status::FlightStage,
        },
        sender::CanSender,
    },
    gps::GPSData,
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::{client::VLPAvionics, lora_config::LoraConfig, packets::fire_pyro::PyroSelect},
};

use crate::{
    avionics_mode::AvionicsMode,
    bootloader::watchdog_task,
    can::{can_bus_broadcast_unix_time_task, init_can_bus, start_can_bus_low_prio_tasks},
    clock_config::vlf5_clock_config,
    landed_mode::landed_mode,
    low_power_mode::low_power_mode,
    self_test_mode::self_test_mode,
    tasks::{
        buzzer_task::{BuzzerTone, buzzer_task},
        gps_task::gps_task,
        pyro_task::{ContinuityUpdate, pyro_task},
        sensor_tasks::{adc_task, baro_task, imu_task},
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
mod e22;
mod landed_mode;
mod low_power_mode;
mod lsm6dsm;
mod ms5607;
mod receive_vlp_task;
mod self_test_mode;
mod tasks;
mod time;
mod utils;

static VLP_KEY: &[u8] = base64!("file:vlp.key");
const LORA_CONFIG: LoraConfig = LoraConfig {
    frequency: 915_100_000,
    sf: 12,
    bw: 250000,
    cr: 8,
    power: 22,
};
pub const MAIN_BULKHEAD_NODE_ID: u16 = 0;
pub const DROGUE_BULKHEAD_NODE_ID: u16 = 1;

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(vlf5_clock_config());

    let tone_queue =
        singleton!(: PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> = PubSubChannel::new()).unwrap();
    let avionics_mode =
        singleton!(: Watch<CriticalSectionRawMutex, AvionicsMode, 10> = Watch::new()).unwrap();
    avionics_mode.sender().send(AvionicsMode::Armed);
    let imu_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, IMUData>, 2> = Watch::new()).unwrap();
    let baro_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, BaroData>, 2> = Watch::new()).unwrap();
    let gps_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, GPSData>, 3> = Watch::new())
            .unwrap();
    let vl_status = singleton!(: BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>> = BlockingMutex::new(RefCell::new(VLCustomStatus { imu_ok: false, baro_ok: false, mag_ok: false, gps_ok: false, sd_ok: false, can_bus_ok: false }))).unwrap();
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
        tone_queue.subscriber().unwrap(),
        avionics_mode,
        imu_reading_watch,
        baro_reading_watch,
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
            tone_queue,
            avionics_mode,
            imu_reading_watch,
            baro_reading_watch,
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
    tone_queue: Subscriber<'static, CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    baro_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    gps_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        3,
    >,
    vl_status: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    spawner.must_spawn(buzzer_task(p.PC15, tone_queue));
    spawner.must_spawn(imu_task(
        p.SPI4,
        p.PE2,
        p.PE6,
        p.PE5,
        p.PC13,
        p.DMA2_CH1,
        p.DMA2_CH0,
        imu_reading_watch.sender(),
        avionics_mode.receiver().unwrap(),
    ));
    spawner.must_spawn(baro_task(
        p.SPI1,
        p.PA5,
        p.PD7,
        p.PA6,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        baro_reading_watch.sender(),
        avionics_mode.receiver().unwrap(),
    ));
    spawner.must_spawn(unix_clock_task(
        p.PA15,
        p.EXTI15,
        unix_clock,
        gps_reading_watch.receiver().unwrap(),
    ));
    spawner.must_spawn(can_bus_broadcast_unix_time_task(can_tx, unix_clock));
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    can_tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    can_rx: CanRx<'static>,
    tone_queue: &'static PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    baro_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    gps_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        3,
    >,
    vl_status: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    // TODO
    let ps = Output::new(p.PA3, Level::Low, Speed::Low);

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let flight_stage = singleton!(:BlockingMutex<NoopRawMutex, RefCell<FlightStage>> = BlockingMutex::new(RefCell::new(FlightStage::Armed))).unwrap();
    let battery_v_watch =
        singleton!(: Watch<NoopRawMutex, SensorReading<BootTimestamp, f32>, 1> = Watch::new())
            .unwrap();

    let continuity_update =
        singleton!(: watch::Watch<NoopRawMutex, ContinuityUpdate, 1> = watch::Watch::new())
            .unwrap();
    let fire_signal = singleton!(: Signal<NoopRawMutex, PyroSelect> = Signal::new()).unwrap();

    spawner.must_spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_reading_watch.receiver().unwrap(),
    ));
    spawner.must_spawn(periodic_beep_task(tone_queue));

    spawner.must_spawn(adc_task(p.ADC1, p.PB0, battery_v_watch.sender()));
    spawner.must_spawn(pyro_task(
        p.PE9,
        p.PE13,
        p.EXTI13,
        p.PD8,
        p.PD13,
        p.PD9,
        p.PE12,
        p.EXTI12,
        continuity_update.dyn_sender(),
        fire_signal,
    ));
    spawner.must_spawn(gps_task(
        p.USART1,
        p.PA10,
        p.PB14,
        gps_reading_watch.sender(),
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
        battery_v_watch.dyn_receiver().unwrap(),
    )
    .await;

    spawner.must_spawn(receive_vlp_task(
        vlp_avionics_client,
        avionics_mode,
        fire_signal,
        tone_queue,
        can_sender,
        can_central,
    ));

    spawner.must_spawn(broadcast_imu_measurement_task(
        imu_reading_watch.receiver().unwrap(),
        can_sender,
        unix_clock,
    ));
    spawner.must_spawn(broadcast_baro_measurement_task(
        baro_reading_watch.receiver().unwrap(),
        can_sender,
        unix_clock,
    ));

    spawner.must_spawn(watchdog_task(p.IWDG1));

    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));
    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));

    loop {
        match avionics_mode.try_get().unwrap() {
            AvionicsMode::Armed => todo!(),
            AvionicsMode::SelfTest => {
                self_test_mode(
                    vlp_avionics_client,
                    avionics_mode,
                    can_central,
                    vl_status,
                    flight_stage,
                )
                .await
            }
            AvionicsMode::LowPower => {
                low_power_mode(
                    vlp_avionics_client,
                    avionics_mode,
                    can_central,
                    gps_reading_watch.dyn_receiver().unwrap(),
                    battery_v_watch.dyn_receiver().unwrap(),
                    baro_reading_watch.dyn_receiver().unwrap(),
                    can_receiver.subscriber().unwrap(),
                    flight_stage,
                )
                .await
            }
            AvionicsMode::Landed => {
                landed_mode(
                    vlp_avionics_client,
                    avionics_mode,
                    can_central,
                    gps_reading_watch.dyn_receiver().unwrap(),
                    battery_v_watch.dyn_receiver().unwrap(),
                    can_receiver.subscriber().unwrap(),
                    flight_stage,
                )
                .await
            }
        }
    }
}

#[embassy_executor::task]
async fn periodic_beep_task(
    tone_queue: &'static pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(3000));

    loop {
        ticker.next().await;
        tone_queue.publish_immediate(BuzzerTone::Low(100, 100));
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
async fn broadcast_imu_measurement_task(
    mut imu_reading_sub: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    can_sender: &'static CanSender<NoopRawMutex, 16>,
    unix_clock: &'static UnixClock,
) {
    loop {
        let imu_reading = imu_reading_sub.get().await;
        can_sender
            .send(
                IMUMeasurementMessage::new(
                    unix_clock
                        .convert_to_unix_us(imu_reading.timestamp_us)
                        .unwrap_or(imu_reading.timestamp_us),
                    &imu_reading.data.acc,
                    &imu_reading.data.gyro,
                )
                .into(),
            )
            .await;
    }
}

#[embassy_executor::task]
async fn broadcast_baro_measurement_task(
    mut baro_reading_sub: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        2,
    >,
    can_sender: &'static CanSender<NoopRawMutex, 16>,
    unix_clock: &'static UnixClock,
) {
    loop {
        let baro_reading = baro_reading_sub.get().await;
        can_sender
            .send(
                BaroMeasurementMessage::new(
                    unix_clock
                        .convert_to_unix_us(baro_reading.timestamp_us)
                        .unwrap_or(baro_reading.timestamp_us),
                    baro_reading.data.pressure,
                    baro_reading.data.temperature,
                )
                .into(),
            )
            .await;
    }
}
