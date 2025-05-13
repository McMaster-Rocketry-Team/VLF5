use embassy_time::Instant;

#[derive(Clone, Copy)]
pub struct Clock;

impl firmware_common_new::time::Clock for Clock {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}
