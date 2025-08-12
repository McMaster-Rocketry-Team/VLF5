use core::future::Future;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::RawMutex, pubsub};
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
