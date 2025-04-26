use embassy_time::Instant;


#[derive(Clone, Copy)]
pub struct Clock;

impl firmware_common_new::time::Clock for Clock {
    fn now_ms(&self) -> f64 {
        Instant::now().as_micros() as f64 / 1000.0
    }
}
