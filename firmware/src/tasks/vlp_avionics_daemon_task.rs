use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{
    peripherals::{
        DMA1_CH2, DMA1_CH3, EXTI1, EXTI4, PA8, PB3, PB4, PC7, PD0, PD1, PD4, PD5, PD6, SPI3,
    },
    Peri,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use firmware_common_new::vlp::client::VLPAvionics;
use firmware_common_new::vlp::lora::LoraPhy;
use firmware_common_new::vlp::lora_config::LoraConfig;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx126x};
use lora_phy::LoRa;

use crate::drivers::e22::E22;

bind_interrupts!(struct Spi3Irqs {
    DMA1_STREAM2 => dma::InterruptHandler<DMA1_CH2>;
    DMA1_STREAM3 => dma::InterruptHandler<DMA1_CH3>;
});

bind_interrupts!(struct LoraExtiIrqs {
    EXTI4 => exti::InterruptHandler<interrupt::typelevel::EXTI4>;
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
});

#[embassy_executor::task]
pub async fn vlp_avionics_daemon_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    vlp_key: &'static [u8; 32],
    lora_config: LoraConfig,
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
    let spi3 = Mutex::<NoopRawMutex, _>::new(Spi::new(
        spi3, sck, mosi, miso, tx_dma, rx_dma, Spi3Irqs, spi_config,
    ));
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
        ExtiInput::new(dio1, dio1_exti, Pull::Down, LoraExtiIrqs),
        ExtiInput::new(busy, busy_exti, Pull::Down, LoraExtiIrqs),
        Some(Output::new(rxen, Level::High, Speed::High)),
        Some(Output::new(txen, Level::High, Speed::High)),
    )
    .unwrap();
    let sx1262 = Sx126x::new(lora_spi_device, iv, config);
    let mut lora = LoRa::new(sx1262, false, Delay).await.unwrap();
    info!("LoRa initialized");
    let mut lora = LoraPhy::new(&mut lora, lora_config);
    let mut daemon = vlp_avionics_client.daemon(&mut lora, vlp_key);
    daemon.run().await;
}
