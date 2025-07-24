use cortex_m::singleton;
use embassy_stm32::{bind_interrupts, peripherals::{PA10, PB14, USART1}, usart::{self, BufferedUart, Config as UartConfig}, Peri};
use embassy_sync::watch;
use firmware_common_new::{gps::{run_gps_uart_receiver, GPSData}, sensor_reading::SensorReading, time::BootTimestamp};

use crate::time::Clock;

#[embassy_executor::task]
pub async fn gps_task(
    usart1: Peri<'static, USART1>,
    rx: Peri<'static, PA10>,
    tx: Peri<'static, PB14>,
    gps_reading_sender: watch::DynSender<'static, SensorReading<BootTimestamp, GPSData>>,
) {
    bind_interrupts!(struct Irqs {
        USART1 => usart::BufferedInterruptHandler<USART1>;
    });

    let tx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let rx_buffer = singleton!(: [u8; 64] = [0; 64]).unwrap();
    let mut config = UartConfig::default();
    config.baudrate = 9600;

    let mut uart1 = BufferedUart::new(usart1, rx, tx, tx_buffer, rx_buffer, Irqs, config).unwrap();

    run_gps_uart_receiver(&mut uart1, Clock, |reading| {
        gps_reading_sender.send(reading);
    })
    .await;
}