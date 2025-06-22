// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod clock_config;
mod e22;
mod time;
mod utils;

use core::mem;

use crate::clock_config::vlf5_clock_config;

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use e22::E22;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA2, PA7, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::Peri;
use embassy_stm32::{
    adc::{self, Adc, AdcChannel as _},
    exti::ExtiInput,
    peripherals::{ADC1, PB0},
};
use embassy_stm32::{
    bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    time::Hertz,
    usart::{self, BufferedUart, Config as UartConfig},
};
use embassy_sync::watch;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Ticker, Timer};
use firmware_common_new::gps::{run_gps_uart_receiver, GPSData};
use firmware_common_new::vlp::client::VLPAvionics;
use firmware_common_new::vlp::lora::LoraPhy;
use firmware_common_new::vlp::lora_config::LoraConfig;
use firmware_common_new::vlp::packets::gps_beacon::GPSBeaconPacket;
use firmware_common_new::vlp::packets::VLPDownlinkPacket;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx126x};
use lora_phy::LoRa;
use time::Clock;

const VLP_KEY: [u8; 32] = [42u8; 32];

/// This program acts as a GPS beacon, sends the current position to GCM over lora
/// using `GPSBeaconPacket` every second
///
/// Blue led blinks when gps no fix
/// Green led blinks when gps fix
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

    // PS: PA3 (low: force pwm)
    let ps = Output::new(p.PA3, Level::Low, Speed::Low);
    mem::forget(ps); // forget ps pin so it does not get reset to Hi-Z when main function finishes

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let gps_data =
        singleton!(: watch::Watch<NoopRawMutex, GPSData, 2> = watch::Watch::new()).unwrap();

    spawner.must_spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_data.receiver().unwrap(),
    ));

    spawner.must_spawn(gps_task(p.USART1, p.PA10, p.PB14, gps_data.sender()));
    
    spawner.must_spawn(send_beacon_packet_task(
        p.ADC1,
        p.PB0,
        vlp_avionics_client,
        gps_data.receiver().unwrap(),
    ));

    spawner.must_spawn(lora_daemon_task(
        vlp_avionics_client,
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
}

#[embassy_executor::task]
async fn power_led_task(
    blue_led: Peri<'static, PA2>,
    green_led: Peri<'static, PA7>,
    mut gps_data: watch::Receiver<'static, NoopRawMutex, GPSData, 2>,
) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let gps_fixed = if let Some(gps_data) = gps_data.try_get() {
            gps_data.lat_lon.is_some()
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
async fn gps_task(
    usart1: Peri<'static, USART1>,
    rx: Peri<'static, PA10>,
    tx: Peri<'static, PB14>,
    gps_data_sender: watch::Sender<'static, NoopRawMutex, GPSData, 2>,
) {
    bind_interrupts!(struct Irqs {
        USART1 => usart::BufferedInterruptHandler<USART1>;
    });

    let tx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let rx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let mut config = UartConfig::default();
    config.baudrate = 9600;

    let mut uart1 = BufferedUart::new(usart1, rx, tx, tx_buffer, rx_buffer, Irqs, config).unwrap();

    run_gps_uart_receiver(&mut uart1, Clock, |gps_data| {
        let gps_data = gps_data.data;
        info!("{:?}", gps_data);
        gps_data_sender.send(gps_data);
    })
    .await;
}

#[embassy_executor::task]
async fn send_beacon_packet_task(
    adc1: Peri<'static, ADC1>,
    pb0: Peri<'static, PB0>,
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    mut gps_data: watch::Receiver<'static, NoopRawMutex, GPSData, 2>,
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

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let gps_data = gps_data.try_get().unwrap();
        let battery_voltage = read_battery_voltage();
        info!(
            "satellites: {}, fixed: {}, batt v: {}V",
            gps_data.num_of_fix_satellites,
            gps_data.lat_lon.is_some(),
            battery_voltage
        );

        vlp_avionics_client.send(VLPDownlinkPacket::GPSBeacon(GPSBeaconPacket::new(
            gps_data.lat_lon,
            gps_data.num_of_fix_satellites,
            battery_voltage,
        )));
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn lora_daemon_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
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
    spi_config.frequency = Hertz(250_000);
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
    let mut daemon = vlp_avionics_client.daemon(&mut lora, &VLP_KEY);
    daemon.run().await;
}
