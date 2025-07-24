// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod avionics_mode;
mod clock_config;
mod e22;
mod lsm6dsm;
mod ms5607;
mod tasks;
mod time;
mod utils;

use core::mem;

use crate::{
    clock_config::vlf5_clock_config,
    ms5607::MS5607,
    tasks::{
        buzzer_task::{BuzzerTone, buzzer_task},
        gps_task::gps_task,
        pyro_task::{ContinuityUpdate, pyro_task},
        vlp_avionics_daemon_task::vlp_avionics_daemon_task,
    },
};
use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use cortex_m_rt::entry;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{PA2, PA7};
use embassy_stm32::{
    Peripherals,
    gpio::{Level, Output, Speed},
    interrupt::{InterruptExt as _, Priority},
};
use embassy_stm32::{
    adc::{self, Adc, AdcChannel as _},
    peripherals::{ADC1, PB0},
};
use embassy_stm32::{
    bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    usart::{self, BufferedUart, Config as UartConfig},
};
use embassy_stm32::{
    interrupt,
    peripherals::{DMA1_CH4, DMA1_CH5, PA5, PA6, PC6, PD7, SPI1},
};
use embassy_stm32::{
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{self, PubSubBehavior as _},
};
use embassy_sync::{signal, watch};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::vlp::packets::gps_beacon::GPSBeaconPacket;
use firmware_common_new::vlp::{client::VLPAvionics, packets::VLPUplinkPacket};
use firmware_common_new::{
    gps::{GPSData, run_gps_uart_receiver},
    vlp::packets::fire_pyro::PyroSelect,
};
use firmware_common_new::{
    sensor_reading::SensorReading, time::BootTimestamp, vlp::lora_config::LoraConfig,
};
use time::Clock;

const VLP_KEY: [u8; 32] = [42u8; 32];

/// This program acts as a GPS beacon, sends the current position to GCM over lora
/// using `GPSBeaconPacket` every second
///
/// Blue led blinks when gps no fix
/// Green led blinks when gps fix

#[entry]
fn main() -> ! {
    embassy_stm32::init(vlf5_clock_config());

    let tone_queue: &mut pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> =
        singleton!(: pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1> = pubsub::PubSubChannel::new()).unwrap();

    static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
    #[embassy_stm32::interrupt]
    unsafe fn USART2() {
        unsafe { EXECUTOR_HIGH.on_interrupt() }
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

    // PS: PA3 (low: force pwm)
    let ps = Output::new(p.PA3, Level::Low, Speed::Low);
    mem::forget(ps); // forget ps pin so it does not get reset to Hi-Z when main function finishes

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let gps_reading =
        singleton!(: watch::Watch<NoopRawMutex, SensorReading<BootTimestamp, GPSData>, 2> = watch::Watch::new())
            .unwrap();
    // (temperature, asl altitude)
    let baro_data =
        singleton!(: watch::Watch<NoopRawMutex, (f32, f32), 1> = watch::Watch::new()).unwrap();
    let continuity_update =
        singleton!(: watch::Watch<NoopRawMutex, ContinuityUpdate, 1> = watch::Watch::new())
            .unwrap();
    let fire_signal =
        singleton!(: signal::Signal<NoopRawMutex, PyroSelect> = signal::Signal::new()).unwrap();

    spawner.must_spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_reading.receiver().unwrap(),
    ));

    spawner.must_spawn(altimeter_task(
        p.SPI1,
        p.PA5,
        p.PD7,
        p.PA6,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        baro_data.sender(),
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

    spawner.must_spawn(send_beacon_packet_task(
        p.ADC1,
        p.PB0,
        vlp_avionics_client,
        gps_reading.receiver().unwrap(),
        continuity_update.receiver().unwrap(),
        baro_data.receiver().unwrap(),
    ));

    spawner.must_spawn(receive_vlp_task(
        vlp_avionics_client,
        fire_signal,
        tone_queue,
    ));

    spawner.must_spawn(vlp_avionics_daemon_task(
        vlp_avionics_client,
        &VLP_KEY,
        LoraConfig {
            frequency: 915_100_000,
            sf: 12,
            bw: 250000,
            cr: 8,
            power: 22,
        },
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
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));
    tone_queue.publish_immediate(BuzzerTone::Low(250, 100));
    tone_queue.publish_immediate(BuzzerTone::High(250, 250));
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
    mut gps_data: watch::Receiver<'static, NoopRawMutex, SensorReading<BootTimestamp, GPSData>, 2>,
) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let gps_fixed = if let Some(gps_data) = gps_data.try_get() {
            gps_data.data.lat_lon.is_some()
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
async fn altimeter_task(
    spi1: Peri<'static, SPI1>,
    sck: Peri<'static, PA5>,
    mosi: Peri<'static, PD7>,
    miso: Peri<'static, PA6>,
    cs: Peri<'static, PC6>,
    tx_dma: Peri<'static, DMA1_CH4>,
    rx_dma: Peri<'static, DMA1_CH5>,
    baro_data: watch::Sender<'static, NoopRawMutex, (f32, f32), 1>,
) {
    // baro
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

    let mut ticker = Ticker::every(Duration::from_hz(100));
    loop {
        let measurement = baro.read().await.unwrap();
        baro_data.send((measurement.data.temperature, measurement.data.altitude()));
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn send_beacon_packet_task(
    adc1: Peri<'static, ADC1>,
    pb0: Peri<'static, PB0>,
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    mut gps_data: watch::Receiver<'static, NoopRawMutex, SensorReading<BootTimestamp, GPSData>, 2>,
    mut continuity_update: watch::Receiver<'static, NoopRawMutex, ContinuityUpdate, 1>,
    mut baro_data: watch::Receiver<'static, NoopRawMutex, (f32, f32), 1>,
) {
    let mut adc = Adc::new(adc1);
    adc.set_resolution(adc::Resolution::BITS12V);
    adc.set_sample_time(adc::SampleTime::CYCLES810_5);
    let mut bat_v_m = pb0.degrade_adc();
    // let mut vrefint_channel = adc.enable_vrefint();

    let mut read_battery_voltage = || {
        let vrefint = 1.21f32;
        // for some reason reading vrefint gives lower than expected value,
        // using a hard coded value instead
        // let vrefint_raw = adc.blocking_read(&mut vrefint_channel);
        let vrefint_raw = 1502u16;
        let ratio = vrefint / vrefint_raw as f32;

        let pb0_raw = adc.blocking_read(&mut bat_v_m);
        let pb0 = pb0_raw as f32 * ratio;
        let batt_v = pb0 / 0.161;
        batt_v
    };

    gps_data.get().await;
    continuity_update.get().await;
    baro_data.get().await;
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut nonce = 0;
    loop {
        let gps_data = gps_data.try_get().unwrap();
        let continuity_update = continuity_update.try_get().unwrap();
        let battery_voltage = read_battery_voltage();
        let (temperature, altitude) = baro_data.try_get().unwrap();

        let packet = GPSBeaconPacket::new(
            nonce,
            gps_data.data.lat_lon,
            gps_data.data.num_of_fix_satellites,
            battery_voltage,
            temperature,
            altitude,
            continuity_update.pyro1_continuity,
            continuity_update.pyro1_fire,
            continuity_update.pyro2_continuity,
            continuity_update.pyro2_fire,
            continuity_update.short_circuit,
        );
        info!("TX: {}", packet);

        vlp_avionics_client.send(packet.into());
        ticker.next().await;
        nonce = nonce.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn receive_vlp_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    fire_signal: &'static signal::Signal<NoopRawMutex, PyroSelect>,
    tone_queue: &'static pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
) {
    loop {
        let (packet, _) = vlp_avionics_client.receive().await;
        info!("RX: {}", packet);

        match packet {
            VLPUplinkPacket::FirePyro(fire_pyro_packet) => {
                tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                Timer::after_millis(1000).await;
                tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                Timer::after_millis(1000).await;
                tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                Timer::after_millis(1000).await;
                fire_signal.signal(fire_pyro_packet.pyro);
            }
            _ => {}
        }
    }
}
