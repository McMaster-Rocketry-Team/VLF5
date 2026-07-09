//! Baro noise-characterization bench (not flight firmware).
//!
//! Streams raw MS5607 pressure (Pa) + temperature (degC) over defmt RTT as fast as
//! the sensor converts (~300 Hz), while the board sits perfectly still. Capture a
//! stationary window and compute the drift-removed altitude-noise std to set
//! `BARO_NOISE_M` in `hil/sensor_replay.rs`. Measured 2026-07: sigma ~= 0.36 m.
//!
//! Run: `cargo run --release --bin baro_bench --probe 0483:374b:066EFF525086874967123920`

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]

use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::singleton;
use cortex_m_rt::entry;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Executor;
use embassy_stm32::{
    Peri, bind_interrupts, dma,
    gpio::{Level, Output, Speed},
    mode::Async,
    peripherals::{DMA1_CH4, DMA1_CH5, PA5, PA6, PC6, PD7, SPI1},
    spi::{self, Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};

#[macro_export]
macro_rules! sleep {
    ($ms:expr) => {{
        use embassy_time::{Duration, Timer};
        Timer::after(Duration::from_millis($ms)).await;
    }};
}

#[path = "clock_config.rs"]
mod clock_config;
#[path = "drivers/ms5607.rs"]
mod ms5607;

use ms5607::MS5607;

bind_interrupts!(struct Spi1Irqs {
    DMA1_STREAM4 => dma::InterruptHandler<DMA1_CH4>;
    DMA1_STREAM5 => dma::InterruptHandler<DMA1_CH5>;
});

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(clock_config::vlf5_clock_config());
    let executor = singleton!(: Executor = Executor::new()).unwrap();
    executor.run(|spawner| {
        spawner
            .spawn(bench_task(
                p.SPI1, p.PA5, p.PD7, p.PA6, p.PC6, p.DMA1_CH4, p.DMA1_CH5,
            ).unwrap());
    })
}

#[embassy_executor::task]
async fn bench_task(
    baro_spi1: Peri<'static, SPI1>,
    baro_sck: Peri<'static, PA5>,
    baro_mosi: Peri<'static, PD7>,
    baro_miso: Peri<'static, PA6>,
    baro_cs: Peri<'static, PC6>,
    baro_tx_dma: Peri<'static, DMA1_CH4>,
    baro_rx_dma: Peri<'static, DMA1_CH5>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi1 = singleton!(: Mutex<NoopRawMutex, Spi<'static, Async, spi::mode::Master>> = Mutex::new(Spi::new(
        baro_spi1,
        baro_sck,
        baro_mosi,
        baro_miso,
        baro_tx_dma,
        baro_rx_dma,
        Spi1Irqs,
        spi_config,
    ))).unwrap();
    let baro_spi_device =
        SpiDeviceWithConfig::new(spi1, Output::new(baro_cs, Level::High, Speed::High), spi_config);
    let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
    let mut baro = MS5607::new(baro_spi_device, baro_buffer);
    baro.reset().await.unwrap();
    info!("BARO_BENCH_START");

    loop {
        match baro.read().await {
            Ok(r) => info!("BARO {} {}", r.data.pressure, r.data.temperature),
            Err(_) => info!("BARO_ERR"),
        }
    }
}
