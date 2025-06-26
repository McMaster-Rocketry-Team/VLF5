#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use {defmt_rtt_pipe as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::sdmmc::DataBlock;
use embassy_stm32::time::mhz;
use embassy_stm32::{
    bind_interrupts, peripherals,
    rng::{self, Rng},
};
use embassy_stm32::{
    crc::{Config as CrcConfig, Crc, InputReverseConfig, PolySize},
    sdmmc::{self, Sdmmc},
};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    let mut sdmmc = Sdmmc::new_4bit(
        p.SDMMC1,
        Irqs,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        Default::default(),
    );

    info!("Configured clock: {}", sdmmc.clock().0);
    sdmmc.init_sd_card(mhz(1)).await.unwrap();

    // let size = sdcard.num_bytes().unwrap();
    // let block_count = (size / 512) as u32;
    // info!("Card size is {} bytes, {} blocks", size, block_count);

    let mut rng = Rng::new(p.RNG, Irqs);
    let crc_config =
        CrcConfig::new(InputReverseConfig::None, false, PolySize::Width8, 69, 69).unwrap();
    let mut crc = Crc::new(p.CRC, crc_config);

    let mut block = DataBlock([0u8; 512]);
    for block_i in 0..10 {
        info!(
            "Testing block {} ({}MiB).....",
            block_i,
            (block_i as f32) * 512.0 / 1024.0 / 1024.0
        );

        rng.async_fill_bytes(&mut block.0[..508]).await.unwrap();
        crc.reset();
        let check_sum = crc.feed_bytes(&block.0[..508]);
        block.0[508..512].copy_from_slice(&check_sum.to_le_bytes());

        sdmmc.write_block(block_i, &block).await.unwrap();

        sdmmc.read_block(block_i, &mut block).await.unwrap();

        crc.reset();
        let check_sum = crc.feed_bytes(&block.0[..508]);
        let check_sum2 = u32::from_le_bytes(block.0[508..512].try_into().unwrap());
        if check_sum != check_sum2 {
            info!("Failed, checksum mismatch: {} != {}", check_sum, check_sum2);
            break;
        }
    }

    info!("SD Card test passed!")
}
