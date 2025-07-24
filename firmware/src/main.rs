#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cortex_m::singleton;
use cortex_m_rt::entry;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::{
    Peri, Peripherals,
    gpio::{Level, Output, Speed},
    interrupt::{self, InterruptExt as _, Priority},
    peripherals::{PA2, PA7},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    pubsub::{PubSubBehavior as _, PubSubChannel, Subscriber},
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    gps::GPSData,
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::{client::VLPAvionics, lora_config::LoraConfig, packets::fire_pyro::PyroSelect},
};

use crate::{
    avionics_mode::AvionicsMode,
    bootloader::watchdog_task,
    can::start_can_bus_tasks,
    clock_config::vlf5_clock_config,
    tasks::{
        buzzer_task::{BuzzerTone, buzzer_task},
        gps_task::gps_task,
        pyro_task::{ContinuityUpdate, pyro_task},
        sensor_tasks::{adc_task, baro_task, imu_task},
        vlp_avionics_daemon_task::vlp_avionics_daemon_task,
    },
};

mod avionics_mode;
mod bootloader;
mod can;
mod clock_config;
mod e22;
mod lsm6dsm;
mod ms5607;
mod tasks;
mod time;
mod utils;

// TODO: read from base64
const VLP_KEY: [u8; 32] = [42u8; 32];
const LORA_CONFIG: LoraConfig = LoraConfig {
    frequency: 915_100_000,
    sf: 12,
    bw: 250000,
    cr: 8,
    power: 22,
};

#[entry]
fn main() -> ! {
    embassy_stm32::init(vlf5_clock_config());

    let tone_queue =
        singleton!(: PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> = PubSubChannel::new()).unwrap();
    let avionics_mode =
        singleton!(: Watch<CriticalSectionRawMutex, AvionicsMode, 10> = Watch::new()).unwrap();
    avionics_mode.sender().send(AvionicsMode::Armed);
    let imu_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, IMUData>, 1> = Watch::new()).unwrap();
    let baro_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, BaroData>, 1> = Watch::new()).unwrap();
    let battery_v_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, f32>, 1> = Watch::new()).unwrap();

    static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
    #[embassy_stm32::interrupt]
    unsafe fn USART2() {
        unsafe { EXECUTOR_HIGH.on_interrupt() }
    }

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::USART2);
    spawner.must_spawn(high_prio_main(
        spawner,
        tone_queue.subscriber().unwrap(),
        avionics_mode,
        imu_reading_watch,
        baro_reading_watch,
        battery_v_watch.sender(),
    ));

    let executor_low = singleton!(: Executor = Executor::new()).unwrap();
    executor_low.run(|spawner| {
        spawner.must_spawn(low_prio_main(
            spawner,
            tone_queue,
            avionics_mode,
            imu_reading_watch,
            baro_reading_watch,
            battery_v_watch,
        ));
    })
}

#[embassy_executor::task]
async fn high_prio_main(
    spawner: SendSpawner,
    tone_queue: Subscriber<'static, CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        1,
    >,
    baro_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        1,
    >,
    battery_v_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, f32>,
        1,
    >,
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
    spawner.must_spawn(adc_task(p.ADC1, p.PB0, battery_v_reading_sender));
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    tone_queue: &'static PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        1,
    >,
    baro_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, BaroData>,
        1,
    >,
    battery_v_watch: &'static Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, f32>, 1>,
) {
    let p = unsafe { Peripherals::steal() };
    let ps = Output::new(p.PA3, Level::Low, Speed::Low);

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let gps_reading =
        singleton!(: Watch<NoopRawMutex, SensorReading<BootTimestamp, GPSData>, 2> = Watch::new())
            .unwrap();
    let baro_reading =
        singleton!(: Watch<NoopRawMutex, SensorReading<BootTimestamp, BaroData>, 1> = Watch::new())
            .unwrap();
    let imu_reading =
        singleton!(: Watch<NoopRawMutex, SensorReading<BootTimestamp, IMUData>, 1> = Watch::new())
            .unwrap();

    let continuity_update =
        singleton!(: watch::Watch<NoopRawMutex, ContinuityUpdate, 1> = watch::Watch::new())
            .unwrap();
    let fire_signal = singleton!(: Signal<NoopRawMutex, PyroSelect> = Signal::new()).unwrap();

    spawner.must_spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_reading.dyn_receiver().unwrap(),
    ));
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
    spawner.must_spawn(gps_task(p.USART1, p.PA10, p.PB14, gps_reading.dyn_sender()));
    spawner.must_spawn(vlp_avionics_daemon_task(
        vlp_avionics_client,
        &VLP_KEY,
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
    start_can_bus_tasks(&spawner, p.FDCAN2, p.PB5, p.PB6).await;

    spawner.must_spawn(watchdog_task(p.IWDG1));

    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));
    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));
}

#[embassy_executor::task]
async fn power_led_task(
    blue_led: Peri<'static, PA2>,
    green_led: Peri<'static, PA7>,
    mut gps_reading: watch::DynReceiver<'static, SensorReading<BootTimestamp, GPSData>>,
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
