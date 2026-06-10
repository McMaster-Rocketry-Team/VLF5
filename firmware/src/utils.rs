use core::future::Future;
use embassy_futures::select::{Either, select};
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    pubsub::{self, Subscriber},
};
use embassy_time::{Duration, Timer};

/// Non-blocking drain of a pubsub subscriber: pulls every message currently
/// queued for this subscriber and returns the most recent one, or `None` if
/// nothing new is queued.
///
/// Use this when you only care about the latest value and want to keep a fast
/// (e.g. IMU-clocked) loop from blocking on a slower source. Draining only
/// advances *this* subscriber's cursor — other subscribers are unaffected.
/// `Lagged` markers are skipped (the `_pure` variant), so an overflowed ring
/// just yields the newest available message instead of an error.
pub fn drain_latest<M, T, const CAP: usize, const SUBS: usize, const PUBS: usize>(
    sub: &mut Subscriber<'_, M, T, CAP, SUBS, PUBS>,
) -> Option<T>
where
    M: RawMutex,
    T: Clone,
{
    let mut newest = None;
    while let Some(msg) = sub.try_next_message_pure() {
        newest = Some(msg);
    }
    newest
}

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

pub struct SubscriberWithLastValue<
    'a,
    M: RawMutex,
    T: Clone,
    const CAP: usize,
    const SUBS: usize,
    const PUBS: usize,
> {
    sub: pubsub::Subscriber<'a, M, T, CAP, SUBS, PUBS>,
    last_value: Option<T>,
}

impl<'a, M: RawMutex, T: Clone, const CAP: usize, const SUBS: usize, const PUBS: usize>
    SubscriberWithLastValue<'a, M, T, CAP, SUBS, PUBS>
{
    pub fn new(
        channel: &'a pubsub::PubSubChannel<M, T, CAP, SUBS, PUBS>,
    ) -> Result<Self, pubsub::Error> {
        Ok(Self {
            sub: channel.subscriber()?,
            last_value: None,
        })
    }
    
    pub async fn get(&mut self) -> T {
        while let Some(value) = self.sub.try_next_message_pure() {
            self.last_value = Some(value)
        }

        if let Some(value) = &self.last_value {
            return value.clone();
        } else {
            let value = self.sub.next_message_pure().await;
            self.last_value = Some(value.clone());
            return value;
        }
    }
}
