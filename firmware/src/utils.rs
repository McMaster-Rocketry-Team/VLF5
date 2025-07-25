use core::future::Future;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

#[macro_export]
macro_rules! sleep {
    ($ms:expr) => {{
        use embassy_time::{Duration, Timer};
        Timer::after(Duration::from_millis($ms)).await;
    }};
}

#[macro_export]
macro_rules! checkBit {
    ($byte:expr, $bit:expr) => {
        $byte & $bit == $bit
    };
}

pub async fn run_with_timeout<F: Future>(ms: u64, future: F) -> Result<F::Output, u64> {
    let timeout_fut = Timer::after(Duration::from_millis(ms));
    match select(timeout_fut, future).await {
        Either::First(_) => Err(ms),
        Either::Second(result) => Ok(result),
    }
}
