use core::cell::RefCell;

use cortex_m::singleton;
use embassy_futures::join::join;
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{PA10, PB14, USART1},
    usart::{self, BufferedUart, Config as UartConfig, Error as UartError},
};
use embassy_sync::{
    blocking_mutex::Mutex as BlockingMutex,
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    watch,
};
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::{ErrorType, Read, ReadExactError};
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
        3,
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

    let last_read_time_us = singleton!(: BlockingMutex<NoopRawMutex, RefCell<u64>> = BlockingMutex::new(RefCell::new(0))).unwrap();
    let mut uart_wrapper = UartWrapper {
        uart: uart1,
        last_read_time_us,
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

    join(update_vl_status_fut, gps_receiver_fut).await;
}

struct UartWrapper {
    uart: BufferedUart<'static>,
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
