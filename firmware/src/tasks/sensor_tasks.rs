use crate::{
    AvionicsModeWatch, VLStatusMutex,
    avionics_mode::AvionicsMode,
    drivers::{lis2mdl::LIS2MDL, lsm6dsm::LSM6DSM, ms5607::MS5607},
    utils::run_with_timeout,
};
use cortex_m::singleton;
use defmt::{error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_stm32::{
    Peri,
    adc::{self, Adc},
    bind_interrupts,
    dma,
    exti,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    i2c::{self, Config as I2cConfig, Error as I2cError, I2c},
    interrupt,
    mode::Async,
    peripherals::{
        ADC1, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, DMA2_CH0, DMA2_CH1, EXTI14, I2C2, PA5, PA6,
        PB0, PB10, PB11, PC6, PC13, PC14, PD7, PE2, PE5, PE6, SPI1, SPI4,
    },
    spi::{self, Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub::{DynPublisher, PubSubChannel},
    watch::Watch,
};
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_async::i2c::I2c as HalI2c;
use embedded_hal_async::spi::SpiDevice;
use firmware_common_new::{
    readings::{BaroData, IMUData, MagData},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

bind_interrupts!(struct Spi4Irqs {
    DMA2_STREAM0 => dma::InterruptHandler<DMA2_CH0>;
    DMA2_STREAM1 => dma::InterruptHandler<DMA2_CH1>;
});

bind_interrupts!(struct Spi1Irqs {
    DMA1_STREAM4 => dma::InterruptHandler<DMA1_CH4>;
    DMA1_STREAM5 => dma::InterruptHandler<DMA1_CH5>;
});

use super::exti15_10_irqs::ExtI15_10Irqs;

bind_interrupts!(struct I2c2Irqs {
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
    DMA1_STREAM6 => dma::InterruptHandler<DMA1_CH6>;
    DMA1_STREAM7 => dma::InterruptHandler<DMA1_CH7>;
});

/// Subscribers: data_logger, CAN broadcast, mode telemetry, and (in Armed) state estimator.
pub type IMUBaroReadingPubSub = PubSubChannel<
    CriticalSectionRawMutex,
    SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>,
    10,
    4,
    1,
>;

// --- HIL sensor injection ----------------------------------------------------
// HIL's rule: substitute at the sensor boundary only. A static bench can't
// produce flight sensor values, so HIL builds synthesize the baro reading AND
// the IMU's accel/gyro values from one scripted flight (one shared script
// clock — the two sensors must tell one consistent story), while the SPI
// buses, the IMU's data-ready interrupt timing, and every other task/GPIO
// stay real. Two seams, both in this file, everything downstream of them is
// the production path:
//   * `read_baro_or_sim` — the real MS5607 read in flight builds, the
//     trajectory generator in HIL (the real baro is never touched).
//   * `imu_values_or_sim` — the LSM6DSM is ALWAYS read for real, so DRDY
//     pacing and sample timestamps stay genuine (the estimators' measured-dt
//     paths see real timing); HIL swaps only the values.
#[cfg(feature = "hil-replay")]
use crate::hil::HilSimState;

/// Flight-build stand-in so the read loops carry an identical signature in both
/// builds; the real state lives in [`crate::hil::HilSimState`].
#[cfg(not(feature = "hil-replay"))]
struct HilSimState;
#[cfg(not(feature = "hil-replay"))]
impl HilSimState {
    const fn new() -> Self {
        Self
    }
}

/// Flight build: read the real barometer over SPI.
#[cfg(not(feature = "hil-replay"))]
#[inline]
async fn read_baro_or_sim<B: SpiDevice>(
    baro: &mut MS5607<'static, B>,
    _hil: &mut HilSimState,
    _mode_watch: &AvionicsModeWatch,
) -> Result<BaroData, B::Error> {
    Ok(baro.read().await?.data)
}

/// HIL build: synthesize the barometer from the scripted flight (never touches the
/// real baro, so a bench baro fault can't abort the simulated flight).
#[cfg(feature = "hil-replay")]
#[inline]
async fn read_baro_or_sim<B: SpiDevice>(
    _baro: &mut MS5607<'static, B>,
    hil: &mut HilSimState,
    mode_watch: &AvionicsModeWatch,
) -> Result<BaroData, B::Error> {
    Ok(hil.next_baro(mode_watch.try_get().unwrap_or(AvionicsMode::SelfTest)))
}

/// Flight build: the real IMU reading passes through untouched.
#[cfg(not(feature = "hil-replay"))]
#[inline]
fn imu_values_or_sim(
    reading: SensorReading<BootTimestamp, IMUData>,
    _hil: &mut HilSimState,
    _mode_watch: &AvionicsModeWatch,
) -> SensorReading<BootTimestamp, IMUData> {
    reading
}

/// HIL build: keep the real reading's timestamp — the LSM6DSM was genuinely
/// read on its data-ready interrupt — and replace only the values with the
/// scripted specific force / angular rate, on the same script clock as the
/// baro. This is what lets the whole airbrakes estimator (pad calibration →
/// ignition → dead reckoning → birth → apogee) fly on the bench with zero
/// application-layer overrides.
#[cfg(feature = "hil-replay")]
#[inline]
fn imu_values_or_sim(
    reading: SensorReading<BootTimestamp, IMUData>,
    hil: &mut HilSimState,
    mode_watch: &AvionicsModeWatch,
) -> SensorReading<BootTimestamp, IMUData> {
    SensorReading::new(
        reading.timestamp_us,
        hil.next_imu(mode_watch.try_get().unwrap_or(AvionicsMode::SelfTest)),
    )
}

// --- Bus error tolerance ------------------------------------------------------
// A transfer on any of these buses can glitch (SPI overrun, an I2C NACK) or,
// worse, never complete at all. Neither may cost the sensor for the rest of the
// flight: losing `imu_baro_task` means no estimator input, so no apogee
// detection and no deploy. So every transfer below is bounded in time and a
// sensor is only declared dead after it fails repeatedly in a row — the same
// shape as the SD path in `sd_card_writer`.

/// Consecutive failed reads before a sensor is treated as dead rather than
/// glitching. Higher than the SD path's `SD_IO_ERROR_LIMIT` of 3 because these
/// run at ~416 Hz, where five in a row is still only ~12 ms of data.
const SENSOR_IO_ERROR_LIMIT: u8 = 5;

/// Attempts for a one-shot init / power-mode transfer before giving up.
const SENSOR_INIT_ATTEMPTS: u8 = 3;

/// A read that has not completed in this long is abandoned and counted as a
/// failure. Nominal reads are ~0.15 ms (IMU: 13 B at 1 MHz) and ~1.8 ms (baro,
/// including its ADC conversion waits), so this cannot fire in normal
/// operation. It exists so a stalled DMA transfer cannot park the task forever
/// — nothing else would notice, since `watchdog_task` pets unconditionally and
/// has no liveness input from here.
const SENSOR_IO_TIMEOUT_MS: u64 = 20;

/// The same bound for init / power-mode transfers, which is necessarily far
/// looser: those carry deliberate settling delays that dwarf the transfers
/// themselves — 40 ms inside `LSM6DSM::reset`, 30 ms inside `LIS2MDL::reset`,
/// 20 ms plus six coefficient reads inside `MS5607::reset`. Anything near
/// [`SENSOR_IO_TIMEOUT_MS`] would time out every single attempt and take the
/// sensor down at boot.
const SENSOR_INIT_TIMEOUT_MS: u64 = 500;

/// Why a sensor transfer failed.
#[derive(defmt::Format)]
enum SensorFault<E> {
    /// The driver reported an error.
    Io(E),
    /// The transfer did not finish within [`SENSOR_IO_TIMEOUT_MS`].
    Timeout,
}

/// Flatten a [`run_with_timeout`] result into one `Result`.
fn sensor_result<T, E>(result: Result<Result<T, E>, u64>) -> Result<T, SensorFault<E>> {
    match result {
        Ok(Ok(value)) => Ok(value),
        Ok(Err(e)) => Err(SensorFault::Io(e)),
        Err(_) => Err(SensorFault::Timeout),
    }
}

/// Spend or restore a sensor's consecutive-failure budget.
///
/// * `Ok(Some(v))` — the read succeeded; the budget is restored in full.
/// * `Ok(None)` — it failed, but the budget is not spent: carry on without
///   this sample.
/// * `Err(fault)` — the budget is exhausted; the sensor is dead.
fn tolerate<T, E>(
    result: Result<T, SensorFault<E>>,
    failures: &mut u8,
) -> Result<Option<T>, SensorFault<E>> {
    match result {
        Ok(value) => {
            *failures = 0;
            Ok(Some(value))
        }
        Err(fault) => {
            *failures = failures.saturating_add(1);
            if *failures >= SENSOR_IO_ERROR_LIMIT {
                Err(fault)
            } else {
                Ok(None)
            }
        }
    }
}

/// Run a one-shot sensor transfer with a timeout, retrying transients up to
/// [`SENSOR_INIT_ATTEMPTS`] times. `$op` is re-evaluated per attempt, so it may
/// borrow the driver mutably. Evaluates to `Result<T, SensorFault<E>>`.
macro_rules! init_io {
    ($op:expr, $retry_msg:literal) => {{
        let mut outcome = Err(SensorFault::Timeout);
        for _ in 0..SENSOR_INIT_ATTEMPTS {
            outcome = sensor_result(run_with_timeout(SENSOR_INIT_TIMEOUT_MS, $op).await);
            if outcome.is_ok() {
                break;
            }
            warn!($retry_msg);
        }
        outcome
    }};
}

#[embassy_executor::task]
pub async fn imu_baro_task(
    imu_spi4: Peri<'static, SPI4>,
    imu_sck: Peri<'static, PE2>,
    imu_mosi: Peri<'static, PE6>,
    imu_miso: Peri<'static, PE5>,
    imu_cs: Peri<'static, PC13>,
    imu_tx_dma: Peri<'static, DMA2_CH1>,
    imu_rx_dma: Peri<'static, DMA2_CH0>,
    imu_int1: Peri<'static, PC14>,
    imu_int1_exti: Peri<'static, EXTI14>,

    baro_spi1: Peri<'static, SPI1>,
    baro_sck: Peri<'static, PA5>,
    baro_mosi: Peri<'static, PD7>,
    baro_miso: Peri<'static, PA6>,
    baro_cs: Peri<'static, PC6>,
    baro_tx_dma: Peri<'static, DMA1_CH4>,
    baro_rx_dma: Peri<'static, DMA1_CH5>,

    pubsub: &'static IMUBaroReadingPubSub,

    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.imu_ok = true;
        s.baro_ok = true;
    });

    let result = try {
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
        init_io!(imu.reset(), "IMU reset failed, retrying").map_err(IMUOrBaroError::IMU)?;
        let mut imu_int1 = ExtiInput::new(imu_int1, imu_int1_exti, Pull::None, ExtI15_10Irqs);
        info!("IMU initialized");

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
        let baro_spi_device = SpiDeviceWithConfig::new(
            spi1,
            Output::new(baro_cs, Level::High, Speed::High),
            spi_config,
        );
        let baro_buffer = singleton!(: [u8; 8] = [0; 8]).unwrap();
        let mut baro = MS5607::new(baro_spi_device, baro_buffer);
        init_io!(baro.reset(), "baro reset failed, retrying").map_err(IMUOrBaroError::Baro)?;
        info!("Barometer initialized");

        let publisher = pubsub.dyn_publisher().unwrap();
        // In flight builds this is a zero-sized stand-in; in HIL it holds the
        // shared script clock + per-sensor sample counters across mode changes
        // (so a re-arm replays).
        let mut hil = HilSimState::new();
        loop {
            match avionics_mode.get().await {
                AvionicsMode::Armed | AvionicsMode::SelfTest => {
                    init_io!(imu.power_up(), "IMU power-up failed, retrying")
                        .map_err(IMUOrBaroError::IMU)?;
                    match select(
                        read_imu_baro_loop(
                            &mut imu_int1,
                            &mut imu,
                            &mut baro,
                            &publisher,
                            &mut hil,
                            avionics_mode_watch,
                        ),
                        avionics_mode.changed_and(|m| {
                            *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest
                        }),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(e)?,
                        Either::Second(_) => {}
                    };
                    // A failed power-down only leaves the IMU drawing current.
                    // Never a reason to end the task and lose both sensors for
                    // the rest of the flight — log it and carry on.
                    if init_io!(imu.power_down(), "IMU power-down failed, retrying").is_err() {
                        warn!("IMU left powered up");
                    }
                }
                AvionicsMode::LowPower | AvionicsMode::Demo => {
                    match select(
                        read_baro_low_power_loop(
                            &mut baro,
                            &publisher,
                            &mut hil,
                            avionics_mode_watch,
                        ),
                        avionics_mode.changed(),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(IMUOrBaroError::Baro(e))?,
                        Either::Second(_) => {},
                    };
                }
                AvionicsMode::Landed => {
                    avionics_mode.changed().await;
                }
            }
        }
    };

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();

        match result {
            Err(IMUOrBaroError::Baro(baro_error)) => {
                error!("baro error: {}", baro_error);
                s.baro_ok = false;
            }
            Err(IMUOrBaroError::IMU(imu_error)) => {
                error!("imu error: {}", imu_error);
                s.imu_ok = false;
            }
            Err(IMUOrBaroError::IMUAndBaro(imu_error, baro_error)) => {
                error!("baro error: {}", baro_error);
                s.baro_ok = false;
                error!("imu error: {}", imu_error);
                s.imu_ok = false;
            }
        }
    });
}

enum IMUOrBaroError<I: SpiDevice, B: SpiDevice> {
    IMU(SensorFault<I::Error>),
    Baro(SensorFault<B::Error>),
    IMUAndBaro(SensorFault<I::Error>, SensorFault<B::Error>),
}

async fn read_baro_low_power_loop<B: SpiDevice>(
    baro: &mut MS5607<'static, B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
    hil: &mut HilSimState,
    mode_watch: &AvionicsModeWatch,
) -> Result<!, SensorFault<B::Error>> {
    let mut ticker = Ticker::every(Duration::from_hz(5));
    let mut baro_failures = 0u8;

    loop {
        let read =
            run_with_timeout(SENSOR_IO_TIMEOUT_MS, read_baro_or_sim(baro, hil, mode_watch)).await;
        match tolerate(sensor_result(read), &mut baro_failures)? {
            Some(baro_data) => publisher.publish_immediate(SensorReading::new(
                Instant::now().as_micros(),
                (None, baro_data),
            )),
            // Tolerated: skip this tick rather than publish a stale reading.
            None => warn!("baro read failed ({} consecutive)", baro_failures),
        }
        ticker.next().await;
    }
}

async fn read_imu_baro_loop<I: SpiDevice, B: SpiDevice>(
    imu_int1: &mut ExtiInput<'static, Async>,
    imu: &mut LSM6DSM<I>,
    baro: &mut MS5607<'static, B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, (Option<IMUData>, BaroData)>>,
    hil: &mut HilSimState,
    mode_watch: &AvionicsModeWatch,
) -> Result<!, IMUOrBaroError<I, B>> {
    // Per-sensor failure budgets. Local, so each entry into armed / self-test
    // starts with a full one.
    let mut imu_failures = 0u8;
    let mut baro_failures = 0u8;

    loop {
        imu_int1.wait_for_rising_edge().await;

        let (imu_read, baro_read) = join(
            run_with_timeout(SENSOR_IO_TIMEOUT_MS, imu.read()),
            run_with_timeout(SENSOR_IO_TIMEOUT_MS, read_baro_or_sim(baro, hil, mode_watch)),
        )
        .await;

        let imu_outcome = tolerate(sensor_result(imu_read), &mut imu_failures);
        if matches!(imu_outcome, Ok(None)) {
            warn!("IMU read failed ({} consecutive)", imu_failures);
        }
        let baro_outcome = tolerate(sensor_result(baro_read), &mut baro_failures);
        if matches!(baro_outcome, Ok(None)) {
            warn!("baro read failed ({} consecutive)", baro_failures);
        }

        match (imu_outcome, baro_outcome) {
            (Err(imu_fault), Err(baro_fault)) => {
                Err(IMUOrBaroError::IMUAndBaro(imu_fault, baro_fault))?
            }
            (Err(imu_fault), Ok(_)) => Err(IMUOrBaroError::IMU(imu_fault))?,
            (Ok(_), Err(baro_fault)) => Err(IMUOrBaroError::Baro(baro_fault))?,
            // The baro is what makes a sample: the deployment estimator is
            // sample-clocked and baro-only. The IMU half is already `Option` on
            // the wire and the estimators handle its absence (the airbrakes
            // half skips that sample, its measured-dt path bridges the gap), so
            // a lone IMU glitch still publishes the baro reading, and only a
            // baro glitch costs the whole sample.
            (Ok(imu_reading), Ok(Some(baro_data))) => {
                // HIL: swap the just-read IMU's values for the scripted ones,
                // keeping its genuine DRDY timestamp. Flight: pass-through.
                let imu_reading =
                    imu_reading.map(|reading| imu_values_or_sim(reading, hil, mode_watch));
                // Without an IMU reading there is no DRDY timestamp to carry;
                // the edge fired moments ago, so `now` is the honest stand-in.
                let timestamp_us = imu_reading
                    .as_ref()
                    .map(|reading| reading.timestamp_us)
                    .unwrap_or_else(|| Instant::now().as_micros());
                publisher.publish_immediate(SensorReading::new(
                    timestamp_us,
                    (imu_reading.map(|reading| reading.data), baro_data),
                ))
            }
            (Ok(_), Ok(None)) => {}
        };
    }
}

pub type MagReadingPubSub =
    PubSubChannel<CriticalSectionRawMutex, SensorReading<BootTimestamp, MagData>, 10, 2, 1>;

#[embassy_executor::task]
pub async fn mag_task(
    i2c: Peri<'static, I2C2>,
    scl: Peri<'static, PB10>,
    sda: Peri<'static, PB11>,
    tx_dma: Peri<'static, DMA1_CH7>,
    rx_dma: Peri<'static, DMA1_CH6>,

    pubsub: &'static MagReadingPubSub,

    vl_status: &'static VLStatusMutex,
    avionics_mode_watch: &'static AvionicsModeWatch,
) {
    vl_status.lock(|s| {
        let mut s = s.borrow_mut();
        s.mag_ok = true;
    });

    let mut avionics_mode = avionics_mode_watch.receiver().unwrap();

    let result: Result<!, SensorFault<I2cError>> = try {
        let mut config = I2cConfig::default();
        config.sda_pullup = true;
        config.scl_pullup = true;
        config.frequency = Hertz(400_000);
        let i2c: I2c<'_, embassy_stm32::mode::Async, i2c::Master> =
            I2c::new(i2c, scl, sda, tx_dma, rx_dma, I2c2Irqs, config);
        let mut mag = LIS2MDL::new(i2c);
        init_io!(mag.reset(), "mag reset failed, retrying")?;
        info!("Magnetometer initialized");

        let publisher = pubsub.dyn_publisher().unwrap();
        loop {
            match avionics_mode.get().await {
                AvionicsMode::Armed | AvionicsMode::SelfTest => {
                    init_io!(mag.power_up(), "mag power-up failed, retrying")?;
                    match select(
                        read_mag_loop(&mut mag, &publisher),
                        avionics_mode.changed_and(|m| {
                            *m != AvionicsMode::Armed && *m != AvionicsMode::SelfTest
                        }),
                    )
                    .await
                    {
                        Either::First(Err(e)) => Err(e)?,
                        Either::Second(_) => {}
                    };
                    // As with the IMU: a failed power-down costs current, not
                    // the sensor.
                    if init_io!(mag.power_down(), "mag power-down failed, retrying").is_err() {
                        warn!("mag left powered up");
                    }
                }
                AvionicsMode::LowPower | AvionicsMode::Demo | AvionicsMode::Landed => {
                    avionics_mode.changed().await;
                }
            }
        }
    };

    vl_status.lock(|s| {
        let mut s = s.borrow_mut();

        let error = result.unwrap_err();
        error!("mag error: {}", error);
        s.mag_ok = false;
    });
}

async fn read_mag_loop<B: HalI2c>(
    mag: &mut LIS2MDL<B>,
    publisher: &DynPublisher<'static, SensorReading<BootTimestamp, MagData>>,
) -> Result<!, SensorFault<B::Error>> {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut mag_failures = 0u8;

    loop {
        let read = run_with_timeout(SENSOR_IO_TIMEOUT_MS, mag.read()).await;
        match tolerate(sensor_result(read), &mut mag_failures)? {
            Some(reading) => publisher.publish_immediate(reading),
            // Tolerated: skip this tick. Nothing consumes mag for control, so a
            // gap only costs a logged sample.
            None => warn!("mag read failed ({} consecutive)", mag_failures),
        }
        ticker.next().await;
    }
}

/// Receivers: CAN node_status, data_logger, and the active mode (low-power/armed/…).
pub type BatteryVWatch = Watch<NoopRawMutex, SensorReading<BootTimestamp, f32>, 4>;

#[embassy_executor::task]
pub async fn adc_task(
    adc1: Peri<'static, ADC1>,
    battery_v_pin: Peri<'static, PB0>,
    battery_v_watch: &'static BatteryVWatch,
) {
    let mut adc = Adc::new_with_config(
        adc1,
        adc::AdcConfig {
            resolution: Some(adc::Resolution::Bits12),
            ..Default::default()
        },
    );
    let mut battery_v_pin = battery_v_pin;

    let mut read_battery_voltage = || {
        let vrefint = 1.21f32;
        // for some reason reading vrefint gives lower than expected value,
        // using a hard coded value instead
        // let vrefint_raw = adc.blocking_read(&mut vrefint_channel);
        let vrefint_raw = 1502u16;
        let ratio = vrefint / vrefint_raw as f32;

        // TODO: move to async?
        let pb0_raw = adc.blocking_read(&mut battery_v_pin, adc::SampleTime::Cycles645);
        let pb0 = pb0_raw as f32 * ratio;
        let batt_v = pb0 / 0.161;
        batt_v
    };

    let mut ticker = Ticker::every(Duration::from_hz(10));

    let sender = battery_v_watch.sender();
    loop {
        ticker.next().await;
        sender.send(SensorReading::new(
            Instant::now().as_micros(),
            read_battery_voltage(),
        ));
    }
}
