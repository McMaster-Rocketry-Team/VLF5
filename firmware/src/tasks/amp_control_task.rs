use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker};
use firmware_common_new::can_bus::{messages::amp_control::AmpControlMessage, sender::CanSender};

pub type AmpControlWatch = Watch<NoopRawMutex, AmpControlMessage, 1>;

#[embassy_executor::task]
pub async fn amp_control_task(
    can_sender: &'static CanSender<NoopRawMutex>,
    amp_control_watch: &'static AmpControlWatch,
) {
    let mut ticker = Ticker::every(Duration::from_hz(2));
    let mut amp_control = amp_control_watch.receiver().unwrap();
    loop {
        select(ticker.next(), amp_control.changed()).await;
        can_sender.send(amp_control.get().await.into());
    }
}
