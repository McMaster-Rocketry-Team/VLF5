use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals::PC15,
    Peri,
};
use embassy_sync::pubsub;
use embassy_time::{Duration, Ticker, Timer};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BuzzerTone(
    pub u32, // frequency
    pub u32, // duration (ms)
    pub u32, // silent duration (ms)
);

#[embassy_executor::task]
pub async fn buzzer_task(
    buzzer_ctrl: Peri<'static, PC15>,
    mut tone_queue: pubsub::DynSubscriber<'static, BuzzerTone>,
) {
    let mut buzzer_ctrl = Output::new(buzzer_ctrl, Level::Low, Speed::Low);

    loop {
        let BuzzerTone(frequency, duration, silent_duration) = tone_queue.next_message_pure().await;

        let mut ticker = Ticker::every(Duration::from_hz(frequency as u64 * 2));
        for _ in 0..(frequency * duration / 1000) {
            buzzer_ctrl.set_high();
            ticker.next().await;
            buzzer_ctrl.set_low();
            ticker.next().await;
        }
        Timer::after(Duration::from_millis(silent_duration as u64)).await;
    }
}
