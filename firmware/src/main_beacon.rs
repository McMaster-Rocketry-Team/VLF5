// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod e22;
mod time;
mod utils;

use {defmt_rtt as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use e22::E22;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA2, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::Peri;
use embassy_stm32::{
    bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    time::{mhz, Hertz},
    usart::{self, BufferedUart, Config as UartConfig},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Timer};
use firmware_common_new::gps::run_gps_uart_receiver;
use firmware_common_new::vlp::client::VLPAvionics;
use firmware_common_new::vlp::lora::LoraPhy;
use firmware_common_new::vlp::lora_config::LoraConfig;
use firmware_common_new::vlp::packets::gps_beacon::GPSBeaconPacket;
use firmware_common_new::vlp::packets::VLPDownlinkPacket;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use time::Clock;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;
        let mut config = embassy_stm32::Config::default();
        let rcc = &mut config.rcc;

        rcc.hsi = None;
        rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });
        rcc.csi = false;

        rcc.hsi48 = None;
        rcc.sys = Sysclk::PLL1_P;

        rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL60,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV20),
            divr: None,
        });
        rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: Some(PllDiv::DIV4),
            divq: Some(PllDiv::DIV4),
            divr: Some(PllDiv::DIV4),
        });
        rcc.pll3 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllDiv::DIV8),
        });

        rcc.d1c_pre = AHBPrescaler::DIV1;
        rcc.ahb_pre = AHBPrescaler::DIV2;
        rcc.apb1_pre = APBPrescaler::DIV2;
        rcc.apb2_pre = APBPrescaler::DIV2;
        rcc.apb3_pre = APBPrescaler::DIV2;
        rcc.apb4_pre = APBPrescaler::DIV2;

        rcc.timer_prescaler = TimerPrescaler::DefaultX2;
        rcc.voltage_scale = VoltageScale::Scale0;

        rcc.ls = LsConfig::default_lsi();
        rcc.mux.spi123sel = Saisel::PLL2_P;
        rcc.mux.usart16910sel = Usart16910sel::PLL2_Q;
        rcc.mux.rngsel = Rngsel::PLL1_Q;
        rcc.mux.i2c1235sel = I2c1235sel::PLL3_R;
        rcc.mux.spi45sel = Spi45sel::PLL2_Q;
        rcc.mux.adcsel = Adcsel::PLL2_P;
        rcc.mux.usbsel = Usbsel::PLL1_Q;
        rcc.mux.fdcansel = Fdcansel::PLL2_Q;
        rcc.mux.sdmmcsel = Sdmmcsel::PLL2_R;

        config
    };
    let p = embassy_stm32::init(config);

    // red_led: PB1
    // green_led: PA7
    // blue_led: PA2

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();

    spawner.must_spawn(power_led_task(p.PA2));

    spawner.must_spawn(gps_task(vlp_avionics_client, p.USART1, p.PA10, p.PB14));

    spawner.must_spawn(lora_task(
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
async fn power_led_task(blue_led: Peri<'static, PA2>) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);

    loop {
        blue_led.set_low();
        Timer::after_millis(50).await;
        blue_led.set_high();
        Timer::after_millis(950).await;
    }
}

#[embassy_executor::task]
async fn gps_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    usart1: Peri<'static, USART1>,
    rx: Peri<'static, PA10>,
    tx: Peri<'static, PB14>,
) {
    bind_interrupts!(struct Irqs {
        USART1 => usart::BufferedInterruptHandler<USART1>;
    });

    let tx_buffer = singleton!(: [u8; 32] = [0; 32]).unwrap();
    let rx_buffer = singleton!(: [u8; 32] = [0; 32]).unwrap();
    let mut config = UartConfig::default();
    config.baudrate = 9600;

    let mut uart1 = BufferedUart::new(usart1, rx, tx, tx_buffer, rx_buffer, Irqs, config).unwrap();

    run_gps_uart_receiver(&mut uart1, Clock, |gps_data| {
        info!("GPS Data: {:?}", gps_data);
        let gps_data = gps_data.data;
        vlp_avionics_client.send(VLPDownlinkPacket::GPSBeacon(GPSBeaconPacket::new(
            gps_data.lat_lon,
            gps_data.num_of_fix_satellites,
            0.0,
        )));
    })
    .await;
}

#[embassy_executor::task]
async fn lora_task(
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
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
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
    let key = [0u8; 32];
    let mut daemon = vlp_avionics_client.daemon(&mut lora, &key);
    daemon.run().await;
}
