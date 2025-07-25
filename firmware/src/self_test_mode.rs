use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    watch::Watch,
};
use embassy_time::{Duration, Ticker};
use firmware_common_new::{
    can_bus::{
        messages::avionics_status::{AvionicsStatusMessage, FlightStage},
        sender::CanSender,
    },
    vlp::client::VLPAvionics,
};

use crate::{avionics_mode::AvionicsMode, can_central::CanCentral, tasks::unix_clock::UnixClock};

pub async fn self_test_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    can_sender: &'static CanSender<NoopRawMutex, &'static UnixClock, 16>,
    can_central: &'static CanCentral<NoopRawMutex>,
) {


    
    let send_avionics_status_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(1));

        loop {
            can_sender
                .send(
                    AvionicsStatusMessage {
                        flight_stage: FlightStage::SelfTest,
                    }
                    .into(),
                )
                .await;
            ticker.next().await;
        }
    };
}
