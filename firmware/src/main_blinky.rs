// only use std during testing
#![cfg_attr(not(test), no_std)]
#![no_main]

mod clock_config;

use crate::clock_config::vlf5_clock_config;

use {defmt_rtt_pipe as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Ticker};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

    let mut red_led = Output::new(p.PB1, Level::High, Speed::Low);
    let mut green_led = Output::new(p.PA7, Level::High, Speed::Low);
    let mut blue_led = Output::new(p.PA2, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        red_led.set_low();
        ticker.next().await;
        red_led.set_high();

        green_led.set_low();
        ticker.next().await;
        green_led.set_high();

        blue_led.set_low();
        ticker.next().await;
        blue_led.set_high();
    }
}
