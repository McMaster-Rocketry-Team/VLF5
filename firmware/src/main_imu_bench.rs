//! IMU noise-characterization bench (not flight firmware).
//!
//! The companion to `baro_bench`. Reads the real LSM6DSM on its real DRDY
//! interrupt — the exact path `imu_baro_task` uses in flight — while the
//! board sits perfectly still, and reports per-axis statistics over defmt
//! RTT.
//!
//! # Why windowed statistics instead of a raw stream
//!
//! Streaming six channels at 416 Hz over RTT would be bandwidth-bound and the
//! host would drop samples, which is exactly the failure mode that quietly
//! biases a noise measurement. So the statistics are computed on-device, and
//! over the SAME 2 s window the airbrakes estimator's pad calibration uses
//! (`PAD_WINDOW_S`), because the numbers this bench exists to set are:
//!
//! * per-axis 1-sigma noise -> `ACCEL_NOISE_MS2` / `GYRO_NOISE_DPS` in
//!   `hil/imu_sim.rs`, and the sensor model in the Osiris replay tests;
//! * the per-window MEANS, whose spread across windows is what
//!   `PAD_ACCEL_REJECT_M_S2` (0.1 m/s^2) and `BIAS_REJECT_RAD_S`
//!   (0.15 deg/s) screen on. A quiet bench must sit far inside both, or the
//!   calibration would reject its own good windows on the rail.
//!
//! Sample timing is reported too (mean/min/max dt), because every integration
//! in the airbrakes estimator uses measured dt, and the nominal 416 Hz is an
//! assumption worth checking against the part.
//!
//! Run:
//! ```text
//! cargo run --release --bin imu_bench -- --probe 0483:374b:066BFF525086874967123919
//! ```
//! Leave it running, undisturbed, for a minute or so. Each `IMUW` line is one
//! finished window; `scripts/imu_bench_stats.py` aggregates the capture.

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
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    mode::Async,
    peripherals::{DMA2_CH0, DMA2_CH1, EXTI14, PC13, PC14, PE2, PE5, PE6, SPI4},
    spi::{self, Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};

#[path = "clock_config.rs"]
mod clock_config;
#[path = "drivers/lsm6dsm.rs"]
mod lsm6dsm;
#[path = "tasks/exti15_10_irqs.rs"]
mod exti15_10_irqs;

use exti15_10_irqs::ExtI15_10Irqs;
use lsm6dsm::LSM6DSM;

bind_interrupts!(struct Spi4Irqs {
    DMA2_STREAM0 => dma::InterruptHandler<DMA2_CH0>;
    DMA2_STREAM1 => dma::InterruptHandler<DMA2_CH1>;
});

/// One calibration window, matching the estimator's `PAD_WINDOW_S`.
const WINDOW_S: f32 = 2.0;

/// Streaming mean and variance for one channel (Welford), so a window costs
/// no buffer and no second pass.
#[derive(Default, Clone, Copy)]
struct Welford {
    n: u32,
    mean: f32,
    m2: f32,
}

impl Welford {
    fn update(&mut self, x: f32) {
        self.n += 1;
        let d = x - self.mean;
        self.mean += d / self.n as f32;
        self.m2 += d * (x - self.mean);
    }

    fn std(&self) -> f32 {
        if self.n < 2 {
            0.0
        } else {
            sqrtf(self.m2 / (self.n - 1) as f32)
        }
    }
}

/// `f32::sqrt` is not available in core on this target.
fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(clock_config::vlf5_clock_config());
    let executor = singleton!(: Executor = Executor::new()).unwrap();
    executor.run(|spawner| {
        spawner.spawn(
            bench_task(
                p.SPI4, p.PE2, p.PE6, p.PE5, p.PC13, p.DMA2_CH1, p.DMA2_CH0, p.PC14, p.EXTI14,
            )
            .unwrap(),
        );
    })
}

#[embassy_executor::task]
async fn bench_task(
    imu_spi4: Peri<'static, SPI4>,
    imu_sck: Peri<'static, PE2>,
    imu_mosi: Peri<'static, PE6>,
    imu_miso: Peri<'static, PE5>,
    imu_cs: Peri<'static, PC13>,
    imu_tx_dma: Peri<'static, DMA2_CH1>,
    imu_rx_dma: Peri<'static, DMA2_CH0>,
    imu_int1: Peri<'static, PC14>,
    imu_int1_exti: Peri<'static, EXTI14>,
) {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);

    let spi4 = singleton!(: Mutex<NoopRawMutex, Spi<'static, Async, spi::mode::Master>> = Mutex::new(Spi::new(
        imu_spi4,
        imu_sck,
        imu_mosi,
        imu_miso,
        imu_tx_dma,
        imu_rx_dma,
        Spi4Irqs,
        spi_config,
    ))).unwrap();
    let imu_spi_device = SpiDeviceWithConfig::new(
        spi4,
        Output::new(imu_cs, Level::High, Speed::High),
        spi_config,
    );
    let mut imu = LSM6DSM::new(imu_spi_device);
    imu.reset().await.unwrap();
    let mut imu_int1 = ExtiInput::new(imu_int1, imu_int1_exti, Pull::None, ExtI15_10Irqs);
    imu.power_up().await.unwrap();

    // The part needs a moment after power-up before its output is worth
    // measuring; throw the first half second away rather than fold a
    // settling transient into window 0.
    info!("IMU_BENCH_START settling");
    let settle_until = embassy_time::Instant::now() + embassy_time::Duration::from_millis(500);
    while embassy_time::Instant::now() < settle_until {
        imu_int1.wait_for_rising_edge().await;
        let _ = imu.read().await;
    }
    info!("IMU_BENCH_READY window_s={} — hold still", WINDOW_S);

    let mut window = 0u32;
    loop {
        // six channels: ax ay az gx gy gz
        let mut acc = [Welford::default(); 3];
        let mut gyro = [Welford::default(); 3];
        let mut dt_us = Welford::default();
        let mut dt_min = u32::MAX;
        let mut dt_max = 0u32;

        let start = embassy_time::Instant::now();
        let mut prev: Option<u64> = None;
        loop {
            imu_int1.wait_for_rising_edge().await;
            let r = imu.read().await.unwrap();

            for i in 0..3 {
                acc[i].update(r.data.acc[i]);
                gyro[i].update(r.data.gyro[i]);
            }
            if let Some(p) = prev {
                let d = (r.timestamp_us - p) as u32;
                dt_us.update(d as f32);
                dt_min = dt_min.min(d);
                dt_max = dt_max.max(d);
            }
            prev = Some(r.timestamp_us);

            if (embassy_time::Instant::now() - start).as_micros() as f32 * 1e-6 >= WINDOW_S {
                break;
            }
        }

        // One line per window, fixed field order, so the host script can
        // parse it without knowing anything about defmt.
        info!(
            "IMUW {} n={} accmean={} {} {} accstd={} {} {} gyromean={} {} {} gyrostd={} {} {} dt_us={} min={} max={}",
            window,
            acc[0].n,
            acc[0].mean,
            acc[1].mean,
            acc[2].mean,
            acc[0].std(),
            acc[1].std(),
            acc[2].std(),
            gyro[0].mean,
            gyro[1].mean,
            gyro[2].mean,
            gyro[0].std(),
            gyro[1].std(),
            gyro[2].std(),
            dt_us.mean,
            dt_min,
            dt_max,
        );
        window += 1;
    }
}
