use embassy_stm32::{peripherals::IWDG1, wdg::IndependentWatchdog, Peri};
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn watchdog_task(wdt: Peri<'static, IWDG1>) {
    let mut wdt = IndependentWatchdog::new(wdt, 500_000);
    wdt.unleash();

    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        wdt.pet();
        ticker.next().await;
    }
}
