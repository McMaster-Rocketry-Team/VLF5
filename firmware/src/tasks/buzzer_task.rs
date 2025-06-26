use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals::PC15,
    Peri,
};
use embassy_sync::pubsub;
use embassy_time::{Duration, Ticker, Timer};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuzzerTone {
    Low(
        /// Duration
        u32,
        /// Silent duration
        u32,
    ),
    High(
        /// Duration
        u32,
        /// Silent duration
        u32,
    ),
    Custom(
        /// Frequency
        u32,
        /// Duration
        u32,
        /// Silent duration
        u32,
    ),
}

#[embassy_executor::task]
pub async fn buzzer_task(
    buzzer_ctrl: Peri<'static, PC15>,
    mut tone_queue: pubsub::DynSubscriber<'static, BuzzerTone>,
) {
    let mut buzzer_ctrl = Output::new(buzzer_ctrl, Level::Low, Speed::Low);

    loop {
        let (frequency, duration, silent_duration) = match tone_queue.next_message_pure().await {
            BuzzerTone::Low(duration, silent_duration) => (2600, duration, silent_duration),
            BuzzerTone::High(duration, silent_duration) => (3000, duration, silent_duration),
            BuzzerTone::Custom(frequency, duration, silent_duration) => {
                (frequency, duration, silent_duration)
            }
        };

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
