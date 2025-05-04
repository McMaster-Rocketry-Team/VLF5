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
use embassy_stm32::adc::{self, Adc, AdcChannel, SampleTime};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::opamp::OpAmp;
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA2, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
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

// bind_interrupts!(struct Irqs {
//     ADC1 => adc::InterruptHandler<peripherals::ADC1>;
// });

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

    // PS: PA3 (low: force pwm)
    // red_led: PB1
    // green_led: PA7
    // blue_led: PA2

    // let mut adc = Adc::new(p.ADC2, Irqs);
    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(adc::Resolution::BITS12V);
    let mut bat_v_m = p.PB0.degrade_adc();

    adc.set_sample_time(SampleTime::CYCLES810_5);

    let mut vrefint_channel = adc.enable_vrefint();
    loop {
        let VREFINT = 1.21f32;
        let vrefint = adc.blocking_read(&mut vrefint_channel);
        info!("vrefint: {}", vrefint);
        let ratio = VREFINT / vrefint as f32;

        let measured = adc.blocking_read(&mut bat_v_m);
        info!("measured: {}V", measured as f32 * ratio);
        Timer::after_millis(500).await;
    }
}
