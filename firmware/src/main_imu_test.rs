#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use cortex_m_rt::entry;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::{
    Peri, Peripherals, bind_interrupts,
    can::{
        CanConfigurator, CanRx, CanTx, Frame, IT0InterruptHandler, IT1InterruptHandler,
        enums::{BusError, FrameCreateError},
        frame::Envelope,
    },
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt as _, Priority},
    peripherals::{FDCAN2, PA7},
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::Mutex as BlockingMutex,
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{self, PubSubBehavior as _, PubSubChannel, Subscriber},
    watch::{self, Watch},
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        CanBusFrame, CanBusRX, CanBusTX,
        custom_status::vl_custom_status::VLCustomStatus,
        id::can_node_id_from_serial_number,
        messages::{
            baro_measurement::BaroMeasurementMessage,
            imu_measurement::IMUMeasurementMessage,
            node_status::{NodeHealth, NodeMode, NodeStatusMessage},
            vl_status::FlightStage,
        },
        node_types::VOID_LAKE_NODE_TYPE,
        receiver::CanReceiver,
        sender::CanSender,
    },
    readings::{BaroData, IMUData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::lora_config::LoraConfig,
};
use stm32_device_signature::device_id;

use crate::{
    bootloader::watchdog_task,
    can::{init_can_bus, start_can_bus_low_prio_tasks},
    clock_config::vlf5_clock_config,
    lsm6dsm::LSM6DSM,
    tasks::{
        buzzer_task::{BuzzerTone, buzzer_task},
        sensor_tasks::adc_task,
        unix_clock::UnixClock,
    },
};

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

pub const MAIN_BULKHEAD_NODE_ID: u16 = 0;
pub const DROGUE_BULKHEAD_NODE_ID: u16 = 1;

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(vlf5_clock_config());

    let tone_queue =
        singleton!(: PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> = PubSubChannel::new()).unwrap();
    let imu_reading_watch =
        singleton!(: Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, IMUData>, 2> = Watch::new()).unwrap();
    let unix_clock = singleton!(: UnixClock = UnixClock::new()).unwrap();

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
        imu_reading_watch,
    ));

    let executor_low = singleton!(: Executor = Executor::new()).unwrap();
    executor_low.run(|spawner| {
        spawner.must_spawn(low_prio_main(
            spawner,
            tone_queue,
            imu_reading_watch,
            unix_clock,
        ));
    })
}

#[embassy_executor::task]
async fn high_prio_main(
    spawner: SendSpawner,
    tone_queue: Subscriber<'static, CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
) {
    let p = unsafe { Peripherals::steal() };
    spawner.must_spawn(buzzer_task(p.PC15, tone_queue));

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
    low_g_imu.power_up().await.unwrap();
    info!("IMU initialized");

    let mut imu_int1 = ExtiInput::new(p.PC14, p.EXTI14, Pull::None);
    let imu_reading_sender = imu_reading_watch.sender();
    let mut i = 0usize;
    loop {
        imu_int1.wait_for_rising_edge().await;
        imu_reading_sender.send(low_g_imu.read().await.unwrap());
        i += 1;
        if i == 400 {
            info!("read 400 imu");
            i = 0;
        }
    }
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    tone_queue: &'static PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    imu_reading_watch: &'static Watch<
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, IMUData>,
        2,
    >,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };

    bind_interrupts!(struct Irqs {
        FDCAN2_IT0 => IT0InterruptHandler<FDCAN2>;
        FDCAN2_IT1 => IT1InterruptHandler<FDCAN2>;
    });

    let mut can = CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);
    can.set_bitrate(1_000_000);
    let can = can.into_normal_mode();
    let (tx, rx, _) = can.split();
    let can_node_id = can_node_id_from_serial_number(device_id());
    info!("CAN Device ID: {}", can_node_id);

    let can_sender =
        singleton!(: CanSender<NoopRawMutex> = CanSender::new(VOID_LAKE_NODE_TYPE, can_node_id, Some(&defmt_rtt_pipe::PIPE)))
            .unwrap();

    spawner.must_spawn(can_bus_tx_task(can_sender, tx));
    spawner.must_spawn(node_status_task(can_sender));

    // TODO
    let mut ps = Output::new(p.PA3, Level::Low, Speed::Low);
    let battery_v_watch =
        singleton!(: Watch<NoopRawMutex, SensorReading<BootTimestamp, f32>, 1> = Watch::new())
            .unwrap();

    spawner.must_spawn(power_led_task(p.PA7));
    spawner.must_spawn(adc_task(p.ADC1, p.PB0, battery_v_watch.sender()));

    spawner.must_spawn(broadcast_imu_measurement_task(
        imu_reading_watch.receiver().unwrap(),
        can_sender,
        unix_clock,
    ));
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
async fn power_led_task(green_led: Peri<'static, PA7>) {
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        green_led.set_low();
        Timer::after_millis(50).await;
        green_led.set_high();
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
    can_sender: &'static CanSender<NoopRawMutex>,
    unix_clock: &'static UnixClock,
) {
    imu_reading_sub.get().await;
    let mut toggle = true;
    loop {
        let imu_reading = imu_reading_sub.changed().await;
        if toggle {
            can_sender.send(
                IMUMeasurementMessage::new(
                    imu_reading.timestamp_us,
                    &imu_reading.data.acc,
                    &imu_reading.data.gyro,
                )
                .into(),
            );
        }
        toggle = !toggle;
    }
}

#[embassy_executor::task]
async fn can_bus_tx_task(can_sender: &'static CanSender<NoopRawMutex>, tx: CanTx<'static>) {
    struct TxWrapper(CanTx<'static>);
    impl CanBusTX for TxWrapper {
        type Error = FrameCreateError;

        async fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            let frame = Frame::new_extended(id, data)?;

            self.0.write(&frame).await;
            // FIXME: not a big problem for now, but will be a problem if we implement OTA
            // it needs to flush all the can frames before rebooting
            // tx.flush_all().await;

            Ok(())
        }
    }

    let mut tx_wrapper = TxWrapper(tx);
    can_sender.run_daemon(&mut tx_wrapper).await;
}

#[embassy_executor::task]
async fn node_status_task(can_sender: &'static CanSender<NoopRawMutex>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        can_sender.send(
            NodeStatusMessage::new(
                Instant::now().as_secs() as u32,
                NodeHealth::Healthy,
                NodeMode::Operational,
                VLCustomStatus {
                    imu_ok: true,
                    baro_ok: true,
                    mag_ok: true,
                    gps_ok: true,
                    sd_ok: true,
                    can_bus_ok: true,
                },
            )
            .into(),
        );
        ticker.next().await;
    }
}
