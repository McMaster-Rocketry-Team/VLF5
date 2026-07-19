use core::cell::RefCell;

use cortex_m::singleton;
use defmt::warn;
use embassy_futures::join::join3;
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    usart::{self, BufferedUart, BufferedUartRx, Config as UartConfig, Error as UartError},
};
use embassy_sync::{
    blocking_mutex::Mutex as BlockingMutex,
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    watch,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{ErrorType, Read, ReadExactError, Write as _};
use firmware_common_new::{
    gps::{GPSData, run_gps_uart_receiver},
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

use crate::{VLStatusMutex, time::Clock};

#[embassy_executor::task]
pub async fn gps_task(
    usart1: Peri<'static, USART1>,
    rx: Peri<'static, PA10>,
    tx: Peri<'static, PB14>,
    gps_reading_sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        4,
    >,
    vl_status: &'static VLStatusMutex,
) {
    bind_interrupts!(struct Irqs {
        USART1 => usart::BufferedInterruptHandler<USART1>;
    });

    let tx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let rx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let mut config = UartConfig::default();
    config.baudrate = 9600;

    let uart1: BufferedUart<'static> =
        BufferedUart::new(usart1, rx, tx, tx_buffer, rx_buffer, Irqs, config).unwrap();
    let (mut uart_tx, uart_rx) = uart1.split();

    let last_read_time_us = singleton!(: BlockingMutex<NoopRawMutex, RefCell<u64>> = BlockingMutex::new(RefCell::new(0))).unwrap();
    let mut uart_wrapper = UartWrapper {
        uart: uart_rx,
        last_read_time_us,
    };

    let configure_gps_fut = async {
        // The CAM-M8Q boots into the "portable" dynamic model, whose navigation
        // filter rejects launch dynamics — the Void Lake flight log shows GPS
        // altitude pinned at pad height for the entire ascent while lat/lon kept
        // updating. Switch it to "airborne <4g". The setting does not survive a
        // power cycle, so it is sent on every boot; the re-sends cover the module
        // still starting up when the first frame goes out.
        let frame = ubx_cfg_nav5_airborne_4g();
        for _ in 0..3 {
            if uart_tx.write_all(&frame).await.is_err() {
                warn!("GPS: failed to send CFG-NAV5");
            }
            Timer::after_secs(1).await;
        }
    };

    let update_vl_status_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(5));
        loop {
            ticker.next().await;

            let last_read_time_us = last_read_time_us.lock(|t| *t.borrow());
            let difference_us = Instant::now().as_micros() - last_read_time_us;
            vl_status.lock(|s| {
                let mut s = s.borrow_mut();
                s.gps_ok = difference_us < 1_000_000;
            });
        }
    };

    let gps_receiver_fut = run_gps_uart_receiver(&mut uart_wrapper, Clock, |reading| {
        gps_reading_sender.send(reading);
    });

    join3(configure_gps_fut, update_vl_status_fut, gps_receiver_fut).await;
}

/// UBX-CFG-NAV5 frame applying only the dynamic platform model (mask bit 0):
/// dynModel 8, "airborne <4g". All other payload fields are masked out and
/// ignored by the receiver.
fn ubx_cfg_nav5_airborne_4g() -> [u8; 44] {
    let mut frame = [0u8; 44];
    frame[0] = 0xB5; // sync
    frame[1] = 0x62;
    frame[2] = 0x06; // class CFG
    frame[3] = 0x24; // id NAV5
    frame[4] = 36; // payload length, little endian
    frame[6] = 0x01; // mask: apply dynModel only
    frame[8] = 8; // dynModel: airborne <4g

    // Fletcher-8 over class..payload
    let mut ck_a = 0u8;
    let mut ck_b = 0u8;
    for byte in &frame[2..42] {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    frame[42] = ck_a;
    frame[43] = ck_b;
    frame
}

struct UartWrapper {
    uart: BufferedUartRx<'static>,
    last_read_time_us: &'static BlockingMutex<NoopRawMutex, RefCell<u64>>,
}

impl ErrorType for UartWrapper {
    type Error = UartError;
}

impl Read for UartWrapper {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let result = self.uart.read(buf).await;

        if result.is_ok() {
            self.last_read_time_us.lock(|t| {
                *t.borrow_mut() = Instant::now().as_micros();
            });
        }

        result
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        let result = self.uart.read_exact(buf).await;

        if result.is_ok() {
            self.last_read_time_us.lock(|t| {
                *t.borrow_mut() = Instant::now().as_micros();
            });
        }

        result
    }
}
