// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod clock_config;
mod e22;
mod tasks;
mod time;
mod utils;

use crate::{
    clock_config::vlf5_clock_config,
    tasks::buzzer_task::{buzzer_task, BuzzerTone},
};

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use cortex_m_rt::entry;
use defmt::info;
use e22::E22;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA2, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;
use embassy_stm32::{
    exti::ExtiInput,
    interrupt::{self, InterruptExt as _, Priority},
};
use embassy_stm32::{
    gpio::{Level, Output, Pull, Speed},
    Peripherals,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{self, PubSubBehavior},
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use firmware_common_new::vlp::client::VLPGroundStation;
use firmware_common_new::vlp::lora::LoraPhy;
use firmware_common_new::vlp::lora_config::LoraConfig;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx126x};
use lora_phy::LoRa;

const VLP_KEY: [u8; 32] = [42u8; 32];

/// This program acts as a simple ground station, it simply logs everything it receives from lora to the console.
///
/// Blue led blinks means powered on
/// Every time a valid lora packet is received, green led blinks once

#[entry]
fn main() -> ! {
    embassy_stm32::init(vlf5_clock_config());

    let tone_queue =
        singleton!(: pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> = pubsub::PubSubChannel::new()).unwrap();

    static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
    #[embassy_stm32::interrupt]
    unsafe fn USART2() {
        EXECUTOR_HIGH.on_interrupt()
    }

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::USART2);
    spawner.must_spawn(high_prio_main(spawner, tone_queue.subscriber().unwrap()));

    let executor_low = singleton!(: Executor = Executor::new()).unwrap();
    executor_low.run(|spawner| {
        spawner.must_spawn(low_prio_main(spawner, tone_queue));
    })
}

#[embassy_executor::task]
async fn high_prio_main(
    spawner: SendSpawner,
    tone_queue: pubsub::Subscriber<'static, CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
) {
    let p = unsafe { Peripherals::steal() };
    spawner.must_spawn(buzzer_task(p.PC15, tone_queue));
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    tone_queue: &'static pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
) {
    let p = unsafe { Peripherals::steal() };
    let vlp_gcm_client =
        singleton!(: VLPGroundStation<NoopRawMutex> = VLPGroundStation::new()).unwrap();

    spawner.must_spawn(power_led_task(p.PA2));
    spawner.must_spawn(lora_daemon_task(
        vlp_gcm_client,
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

    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));

    let mut green_led = Output::new(p.PA7, Level::High, Speed::Low);
    loop {
        let packet = vlp_gcm_client.receive().await;
        info!("Received packet: {:?}", packet);
        green_led.set_low();
        Timer::after_millis(50).await;
        green_led.set_high();
    }
}

#[embassy_executor::task]
async fn power_led_task(blue_led: Peri<'static, PA2>) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        blue_led.set_low();
        Timer::after_millis(50).await;
        blue_led.set_high();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn lora_daemon_task(
    vlp_gcm_client: &'static VLPGroundStation<NoopRawMutex>,
    spi3: Peri<'static, SPI3>,
    sck: Peri<'static, PB3>,
    mosi: Peri<'static, PD6>,
    miso: Peri<'static, PB4>,
    cs: Peri<'static, PC7>,
    reset: Peri<'static, PD5>,
    dio1: Peri<'static, PD4>,
    dio1_exti: Peri<'static, EXTI4>,
    busy: Peri<'static, PD1>,
    busy_exti: Peri<'static, EXTI1>,
    txen: Peri<'static, PD0>,
    rxen: Peri<'static, PA8>,
    tx_dma: Peri<'static, DMA1_CH3>,
    rx_dma: Peri<'static, DMA1_CH2>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi3 =
        Mutex::<NoopRawMutex, _>::new(Spi::new(spi3, sck, mosi, miso, tx_dma, rx_dma, spi_config));
    let lora_spi_device =
        SpiDeviceWithConfig::new(&spi3, Output::new(cs, Level::High, Speed::High), spi_config);

    let config = sx126x::Config {
        chip: E22,
        tcxo_ctrl: None,
        use_dcdc: false,
        rx_boost: true,
    };
    let iv = GenericSx126xInterfaceVariant::new(
        Output::new(reset, Level::High, Speed::Low),
        ExtiInput::new(dio1, dio1_exti, Pull::Down),
        ExtiInput::new(busy, busy_exti, Pull::Down),
        Some(Output::new(rxen, Level::High, Speed::High)),
        Some(Output::new(txen, Level::High, Speed::High)),
    )
    .unwrap();
    let sx1262 = Sx126x::new(lora_spi_device, iv, config);
    let mut lora = LoRa::new(sx1262, false, Delay).await.unwrap();
    info!("LoRa initialized");
    let mut lora = LoraPhy::new(
        &mut lora,
        LoraConfig {
            frequency: 915_100_000,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        },
    );
    let mut daemon = vlp_gcm_client.daemon(&mut lora, &VLP_KEY);
    daemon.run().await;
}
