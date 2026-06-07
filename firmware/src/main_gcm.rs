// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]
#![feature(try_blocks)]

mod clock_config;

use crate::clock_config::vlf5_clock_config;
use embassy_stm32::{
    bind_interrupts,
    gpio::AnyPin,
    peripherals::{PA7, PA11, PA12, USB_OTG_FS},
    usb::Driver,
};
use embassy_usb::{
    Builder,
    class::cdc_acm::{BufferedReceiver, CdcAcmClass, Sender, State},
    driver::EndpointError,
};
use embedded_io_async::{Read as _, Write as _};
use stm32_device_signature::device_id_hex;

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use defmt::{error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{
    DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, peripherals::IWDG1, wdg::IndependentWatchdog};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Timer};
use firmware_common_new::rpc::{half_duplex_serial::HalfDuplexSerial, lora_rpc::*};
use firmware_common_new::vlp::lora_config::LoraConfig;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx126x};
use lora_phy::{
    LoRa, RxMode,
    mod_params::{PacketStatus, RadioError},
    sx126x::{DeviceSel, Sx126xVariant},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());
    info!("Hello VLF5 GCM!");

    if cfg!(not(debug_assertions)) {
        spawner.must_spawn(watchdog_task(p.IWDG1));
    } else {
        defmt::warn!("Watchdog is disabled in debug build");
    }

    let class = start_usb_tasks(&spawner, p.USB_OTG_FS, p.PA12, p.PA11).await;

    let (tx, rx) = class.split();
    let usb_rx_buffer = singleton!(:[u8; 64] = [0u8; 64]).unwrap();
    let rx = rx.into_buffered(usb_rx_buffer);
    let mut usb_serial = USBSerial(tx, rx);

    let mut lora_rpc = LoraRpc::new(
        &spawner, p.SPI3, p.PB3, p.PD6, p.PB4, p.PC7, p.PD5, p.PD4, p.EXTI4, p.PD1, p.EXTI1, p.PD0,
        p.PA8, p.DMA1_CH3, p.DMA1_CH2,
    )
    .await;

    loop {
        usb_serial.0.wait_connection().await;
        let result = lora_rpc.run_server(&mut usb_serial).await;
        if let Err(e) = result {
            error!("lora rpc stopped: {}", e);
        }
    }
}

#[embassy_executor::task]
async fn watchdog_task(iwdg: Peri<'static, IWDG1>) {
    let mut iwdg = IndependentWatchdog::new(iwdg, 100_000);
    iwdg.unleash();
    loop {
        Timer::after_millis(50).await;
        iwdg.pet();
    }
}

struct USBSerial(
    Sender<'static, Driver<'static, USB_OTG_FS>>,
    BufferedReceiver<'static, Driver<'static, USB_OTG_FS>>,
);

impl HalfDuplexSerial for USBSerial {
    type Error = EndpointError;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.1.read(buf).await
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).await
    }

    async fn clear_read_buffer(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub async fn start_usb_tasks(
    spawner: &Spawner,
    usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pa12: Peri<'static, PA12>,
    pa11: Peri<'static, PA11>,
) -> CdcAcmClass<'static, Driver<'static, USB_OTG_FS>> {
    let ep_out_buffer = singleton!(:[u8; 256] = [0u8; 256]).unwrap();
    let mut driver_config = embassy_stm32::usb::Config::default();
    driver_config.vbus_detection = false;

    bind_interrupts!(struct Irqs {
        OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
    });

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, ep_out_buffer, driver_config);

    // https://www.notion.so/mcmasterrocketry/Controls-PCB-USB-ID-22fd3a029ea580f085a9c8ce1c589e96
    let mut config = embassy_usb::Config::new(0x120a, 0x0005);
    config.self_powered = false;
    config.manufacturer = Some("MacRocketry");
    config.product = Some("The ENDGAME GCM");
    config.serial_number = Some(device_id_hex());

    let config_descriptor = singleton!(:[u8; 256] = [0; 256]).unwrap();
    let bos_descriptor = singleton!(:[u8; 256] = [0; 256]).unwrap();
    let control_buf = singleton!(:[u8; 64] = [0; 64]).unwrap();

    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [],
        control_buf,
    );

    let state = singleton!(: State = State::new()).unwrap();

    let class = CdcAcmClass::new(&mut builder, state, 64);

    spawner.must_spawn(usb_task(builder.build()));

    class
}

#[embassy_executor::task]
async fn usb_task(
    mut usb: embassy_usb::UsbDevice<'static, embassy_stm32::usb::Driver<'static, USB_OTG_FS>>,
) {
    usb.run().await;
}

#[embassy_executor::task(pool_size = 2)]
pub async fn blink_led_once(pin: Peri<'static, AnyPin>) {
    let _led = Output::new(pin, Level::Low, Speed::Low);
    Timer::after_millis(50).await;
}

pub struct LoraRpc<'a> {
    spawner: &'a Spawner,
    lora: LoRa<
        Sx126x<
            SpiDeviceWithConfig<
                'static,
                NoopRawMutex,
                Spi<'static, embassy_stm32::mode::Async>,
                Output<'static>,
            >,
            GenericSx126xInterfaceVariant<Output<'static>, ExtiInput<'static>>,
            E22,
        >,
        Delay,
    >,
    config: LoraConfig,
}

impl<'a> LoraRpc<'a> {
    pub async fn new(
        spawner: &'a Spawner,
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
    ) -> Self {
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = Hertz(1_000_000);
        let spi3 = singleton!(: Mutex<NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>> = Mutex::<NoopRawMutex, _>::new(Spi::new(
            spi3, sck, mosi, miso, tx_dma, rx_dma, spi_config,
        ))).unwrap();
        let lora_spi_device: SpiDeviceWithConfig<
            'static,
            NoopRawMutex,
            Spi<'static, embassy_stm32::mode::Async>,
            Output<'static>,
        > = SpiDeviceWithConfig::new(spi3, Output::new(cs, Level::High, Speed::High), spi_config);

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
        let lora = LoRa::new(sx1262, false, Delay).await.unwrap();
        info!("LoRa initialized");

        Self {
            spawner,
            lora,
            config: LoraConfig {
                frequency: 915_000_000,
                sf: 12,
                bw: 250_000,
                cr: 8,
                power: 0,
            },
        }
    }
}

impl<'a> LoraRpcServer for LoraRpc<'a> {
    async fn configure(&mut self, config: LoraConfig) -> ConfigureResponse {
        self.config = config;
        info!("New LoRa config: {}", self.config);

        ConfigureResponse {}
    }

    async fn rx(&mut self, timeout_ms: u32) -> RxResponse {
        let mut buffer = [0u8; 256];

        let result: Result<(u8, PacketStatus), RadioError> = try {
            let modulation_params = self.lora.create_modulation_params(
                self.config.sf_phy(),
                self.config.bw_phy(),
                self.config.cr_phy(),
                self.config.frequency,
            )?;
            let rx_pkt_params = self.lora.create_rx_packet_params(
                8,
                false,
                buffer.len() as u8,
                false,
                false,
                &modulation_params,
            )?;

            let timeout_us = timeout_ms as u32 * 1_000;
            let symbol_time_us = self.config.symbol_time_us();
            let timeout_symbols = (timeout_us / symbol_time_us) as u16;
            let listen_mode = RxMode::Single(timeout_symbols.min(254));

            self.lora
                .prepare_for_rx(listen_mode, &modulation_params, &rx_pkt_params)
                .await
                .unwrap();

            self.lora.rx(&rx_pkt_params, &mut buffer).await?
        };

        match result {
            Ok(value) => {
                self.spawner
                    .spawn(blink_led_once(unsafe { PA7::steal().into() }))
                    .ok();
                RxResponse {
                    result: LoraRpcRxResult::Success {
                        len: value.0,
                        data: buffer,
                        rssi: value.1.rssi,
                        snr: value.1.snr,
                    },
                }
            }
            Err(RadioError::ReceiveTimeout) => RxResponse {
                result: LoraRpcRxResult::Timeout,
            },
            Err(e) => {
                warn!("{}", e);
                RxResponse {
                    result: LoraRpcRxResult::Error,
                }
            }
        }
    }

    async fn tx(&mut self, len: u32, data: [u8; 256]) -> TxResponse {
        let result: Result<(), RadioError> = try {
            let modulation_params = self.lora.create_modulation_params(
                self.config.sf_phy(),
                self.config.bw_phy(),
                self.config.cr_phy(),
                self.config.frequency,
            )?;
            let mut tx_params =
                self.lora
                    .create_tx_packet_params(8, false, false, false, &modulation_params)?;

            self.lora
                .prepare_for_tx(
                    &modulation_params,
                    &mut tx_params,
                    self.config.power,
                    &data[..(len as usize)],
                )
                .await?;
            self.lora.tx().await?;
        };

        if let Err(e) = &result {
            warn!("{}", e);
        } else {
            self.spawner
                .spawn(blink_led_once(unsafe { PA8::steal().into() }))
                .ok();
        }

        TxResponse {
            success: result.is_ok(),
        }
    }

    async fn tx_then_rx(
        &mut self,
        len: u32,
        data: [u8; 256],
        rx_timeout_ms: u32,
    ) -> TxThenRxResponse {
        let tx_result = self.tx(len, data).await;
        if !tx_result.success {
            return TxThenRxResponse {
                tx_success: false,
                result: LoraRpcRxResult::Error,
            };
        } else {
            self.spawner
                .spawn(blink_led_once(unsafe { PA8::steal().into() }))
                .ok();
        }

        let rx_result = self.rx(rx_timeout_ms).await;

        if matches!(rx_result.result, LoraRpcRxResult::Success { .. }) {
            self.spawner
                .spawn(blink_led_once(unsafe { PA7::steal().into() }))
                .ok();
        }

        return TxThenRxResponse {
            tx_success: true,
            result: rx_result.result,
        };
    }
}

pub struct E22;

impl Sx126xVariant for E22 {
    fn get_device_sel(&self) -> DeviceSel {
        DeviceSel::HighPowerPA
    }

    fn use_dio2_as_rfswitch(&self) -> bool {
        false
    }
}
