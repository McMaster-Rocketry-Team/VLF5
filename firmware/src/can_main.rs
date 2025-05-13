// only use std when feature = "std" is enabled or during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use cortex_m::singleton;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::can::enums::FrameCreateError;
use embassy_stm32::can::{CanConfigurator, CanTx, Frame};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{FDCAN2, PA2};
use embassy_stm32::time::mhz;
use embassy_stm32::{bind_interrupts, can, Peri};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::can_bus::id::can_node_id_from_serial_number;
use firmware_common_new::can_bus::messages::node_status::{NodeHealth, NodeMode};
use firmware_common_new::can_bus::messages::NodeStatusMessage;
use firmware_common_new::can_bus::node_types::VOID_LAKE_NODE_TYPE;
use firmware_common_new::can_bus::sender::CanSender;
use firmware_common_new::can_bus::CanBusTX;
use stm32_device_signature::device_id;

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
    info!("Hello VLF5!");

    spawner.must_spawn(power_led_task(p.PA2));

    let can_node_id = can_node_id_from_serial_number(device_id());
    info!("CAN Device ID: {}", can_node_id);

    let can_sender =
        singleton!(: CanSender<NoopRawMutex, 4> = CanSender::new(VOID_LAKE_NODE_TYPE, can_node_id))
            .unwrap();

    bind_interrupts!(struct Irqs {
        FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
        FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
    });

    let mut can = CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);
    can.set_bitrate(250_000);
    let can = can.into_normal_mode();
    let (tx, _rx, _) = can.split();

    spawner.must_spawn(can_bus_tx_task(can_sender, tx));
    spawner.must_spawn(node_status_task(can_sender));
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
async fn can_bus_tx_task(can_sender: &'static CanSender<NoopRawMutex, 4>, tx: CanTx<'static>) {
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
async fn node_status_task(can_sender: &'static CanSender<NoopRawMutex, 4>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        can_sender
            .send(
                NodeStatusMessage {
                    uptime_s: Instant::now().as_secs() as u32,
                    health: NodeHealth::Healthy,
                    mode: NodeMode::Operational,
                    custom_status: 0,
                }
                .into(),
            )
            .await;
        ticker.next().await;
    }
}
