use core::cell::RefCell;

use embassy_futures::{join::join3, select::select};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    blocking_mutex::Mutex as BlockingMutex,
    watch::Watch,
};
use embassy_time::{Duration, Ticker};
use firmware_common_new::{
    can_bus::{
        messages::avionics_status::{AvionicsStatusMessage, FlightStage},
        node_types::{
            AERO_RUST_NODE_TYPE, AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE,
            PAYLOAD_ACTIVATION_NODE_TYPE, PAYLOAD_EPS1_NODE_TYPE, PAYLOAD_EPS2_NODE_TYPE,
            PAYLOAD_ROCKET_WIFI_NODE_TYPE,
        },
        sender::CanSender,
    },
    vlp::{
        client::VLPAvionics,
        packets::self_test_result::{NodeStatus, SelfTestResultPacketBuilder},
    },
};

use crate::{
    DROGUE_BULKHEAD_NODE_ID, MAIN_BULKHEAD_NODE_ID, avionics_mode::AvionicsMode,
    can_central::CanCentral,
};

// TODO: other tasks should update this struct
#[derive(Debug, defmt::Format, Clone)]
pub struct VLSelfTestStatus {
    pub imu_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub gps_ok: bool,
    pub sd_ok: bool,
    pub can_bus_ok: bool,
}

pub async fn self_test_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    can_sender: &'static CanSender<NoopRawMutex, 16>,
    can_central: &'static CanCentral<NoopRawMutex>,
    vl_self_test_status: &'static BlockingMutex<CriticalSectionRawMutex,RefCell<VLSelfTestStatus>>
) {
    let packet_builder = SelfTestResultPacketBuilder::<NoopRawMutex>::new();

    let update_node_status_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(1));
        loop {
            let all_nodes = can_central.get_all_nodes();
            packet_builder.update(|packet| {
                packet.amp = NodeStatus::offline();
                packet.icarus = NodeStatus::offline();
                packet.ozys1 = NodeStatus::offline();
                packet.ozys2 = NodeStatus::offline();
                packet.aero_rust = NodeStatus::offline();
                packet.payload_activation_pcb = NodeStatus::offline();
                packet.rocket_wifi = NodeStatus::offline();
                packet.payload_eps1 = NodeStatus::offline();
                packet.payload_eps2 = NodeStatus::offline();
                packet.main_bulkhead_pcb = NodeStatus::offline();
                packet.drogue_bulkhead_pcb = NodeStatus::offline();

                let mut first_ozys = true;
                for node in all_nodes {
                    if node.typ == AMP_NODE_TYPE {
                        packet.amp = node.into();
                    } else if node.typ == ICARUS_NODE_TYPE {
                        packet.icarus = node.into();
                    } else if node.typ == OZYS_NODE_TYPE {
                        if first_ozys {
                            packet.ozys1 = node.into();
                            first_ozys = false;
                        } else {
                            packet.ozys2 = node.into();
                        }
                    } else if node.typ == AERO_RUST_NODE_TYPE {
                        packet.aero_rust = node.into();
                    } else if node.typ == PAYLOAD_ACTIVATION_NODE_TYPE {
                        packet.payload_activation_pcb = node.into();
                    } else if node.typ == PAYLOAD_ROCKET_WIFI_NODE_TYPE {
                        packet.rocket_wifi = node.into();
                    } else if node.typ == PAYLOAD_EPS1_NODE_TYPE {
                        packet.payload_eps1 = node.into();
                    } else if node.typ == PAYLOAD_EPS2_NODE_TYPE {
                        packet.payload_eps2 = node.into();
                    } else if node.id == MAIN_BULKHEAD_NODE_ID {
                        packet.main_bulkhead_pcb = node.into();
                    } else if node.id == DROGUE_BULKHEAD_NODE_ID {
                        packet.drogue_bulkhead_pcb = node.into();
                    }
                }

                vl_self_test_status.lock(|r|{
                    let vl_self_test_status = r.borrow();
                    packet.imu_ok = vl_self_test_status.imu_ok;
                    packet.baro_ok = vl_self_test_status.baro_ok;
                    packet.mag_ok = vl_self_test_status.mag_ok;
                    packet.gps_ok = vl_self_test_status.gps_ok;
                    packet.sd_ok = vl_self_test_status.sd_ok;
                    packet.can_bus_ok = vl_self_test_status.can_bus_ok;
                })
            });

            ticker.next().await;
        }
    };

    let send_packet_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(2));

        loop {
            ticker.next().await;

            vlp_avionics_client.send(packet_builder.create_packet().into());
        }
    };

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

    let fut = join3(
        update_node_status_fut,
        send_packet_fut,
        send_avionics_status_fut,
    );

    let wait_self_test_mode_end_fut = async {
        let mut receiver = avionics_mode.receiver().unwrap();
        while receiver.get().await == AvionicsMode::SelfTest {}
    };

    select(fut, wait_self_test_mode_end_fut).await;
}
