use core::cell::RefCell;

use embassy_stm32::{
    Peri,
    exti::ExtiInput,
    gpio::Pull,
    peripherals::{EXTI15, PA15},
};
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;
use firmware_common_new::{gps::GPSData, sensor_reading::SensorReading};

use crate::GPSReadingWatch;

#[embassy_executor::task]
pub async fn unix_clock_task(
    pa15: Peri<'static, PA15>,
    exti15: Peri<'static, EXTI15>,
    unix_clock: &'static UnixClock,
    gps_reading_watch: &'static GPSReadingWatch,
) {
    let mut gps_reading = gps_reading_watch.receiver().unwrap();
    let mut pps = ExtiInput::new(pa15, exti15, Pull::Down);

    gps_reading.get().await;
    loop {
        pps.wait_for_high().await;
        let pps_time_us = Instant::now().as_micros();
        if let Some(SensorReading {
            timestamp_us: reading_timestamp_us,
            data:
                GPSData {
                    timestamp: Some(gps_timestamp_s),
                    ..
                },
            ..
        }) = gps_reading.try_get()
        {
            if pps_time_us - reading_timestamp_us < 800_000 {
                let current_unix_timestamp_us = ((gps_timestamp_s + 1) as u64) * 1_000_000;
                let new_offset = current_unix_timestamp_us - pps_time_us;
                unix_clock.unix_offset.lock(|s| {
                    let mut unix_offset = s.borrow_mut();
                    unix_offset.replace(new_offset);
                });
            }
        }
    }
}

pub struct UnixClock {
    unix_offset: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<u64>>>,
}

impl UnixClock {
    pub fn new() -> Self {
        Self {
            unix_offset: BlockingMutex::new(RefCell::new(None)),
        }
    }
    pub fn is_ready(&self) -> bool {
        self.unix_offset.lock(|offset| offset.borrow().is_some())
    }

    pub fn convert_to_unix_us(&self, boot_timestamp_us: u64) -> Option<u64> {
        self.unix_offset.lock(|offset| {
            let offset = offset.borrow();
            offset.map(|offset| offset + boot_timestamp_us)
        })
    }

    pub fn now_us(&self) -> Option<u64> {
        self.unix_offset.lock(|offset| {
            let offset = offset.borrow();
            offset.map(|offset| offset + Instant::now().as_micros())
        })
    }

    pub fn now_us_or_boot_time(&self) -> u64 {
        let now_boot_time = Instant::now().as_micros();

        self.unix_offset.lock(|offset| {
            let offset = offset.borrow();
            offset
                .map(|offset| offset + now_boot_time)
                .unwrap_or(now_boot_time)
        })
    }
}

impl firmware_common_new::time::Clock for &UnixClock {
    fn now_us(&self) -> u64 {
        let now = Instant::now().as_micros();
        self.convert_to_unix_us(now).unwrap_or(now)
    }
}
