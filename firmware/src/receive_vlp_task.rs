use defmt::info;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    pubsub::{self, PubSubBehavior as _},
    signal,
    watch::Watch,
};
use embassy_time::Timer;
use firmware_common_new::{
    can_bus::{
        messages::{
            amp_overwrite::AmpOverwriteMessage, amp_reset_output::AmpResetOutputMessage,
            payload_eps_output_overwrite::PayloadEPSOutputOverwriteMessage, reset::ResetMessage,
        },
        node_types::{
            AERO_RUST_NODE_TYPE, AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE,
            PAYLOAD_ACTIVATION_NODE_TYPE, PAYLOAD_EPS1_NODE_TYPE, PAYLOAD_EPS2_NODE_TYPE,
            PAYLOAD_ROCKET_WIFI_NODE_TYPE, VOID_LAKE_NODE_TYPE,
        },
        sender::CanSender,
    },
    vlp::{
        client::VLPAvionics,
        packets::{VLPUplinkPacket, fire_pyro::PyroSelect, reset::DeviceToReset},
    },
};

use crate::{
    DROGUE_BULKHEAD_NODE_ID, MAIN_BULKHEAD_NODE_ID, avionics_mode::AvionicsMode,
    can_central::CanCentral, tasks::buzzer_task::BuzzerTone,
};

#[embassy_executor::task]
pub async fn receive_vlp_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    fire_signal: &'static signal::Signal<NoopRawMutex, PyroSelect>,
    tone_queue: &'static pubsub::PubSubChannel<CriticalSectionRawMutex, BuzzerTone, 10, 1, 1>,
    can_sender: &'static CanSender<NoopRawMutex, 16>,
    can_central: &'static CanCentral<NoopRawMutex>,
) {
    let avionics_mode_sender = avionics_mode.sender();
    loop {
        let (packet, _) = vlp_avionics_client.receive().await;
        match packet {
            VLPUplinkPacket::ChangeMode(packet) => {
                avionics_mode_sender.send(packet.mode.into());
            }
            VLPUplinkPacket::Reset(packet) => {
                #[derive(defmt::Format)]
                enum NodeSelection {
                    All,
                    NodeType(u8),
                    NodeId(u16),
                    None,
                }

                let node_selection = match packet.device {
                    DeviceToReset::All => NodeSelection::All,
                    DeviceToReset::AMPOut1 => {
                        can_sender
                            .send(AmpResetOutputMessage { output: 1 }.into())
                            .await;
                        NodeSelection::None
                    }
                    DeviceToReset::AMPOut2 => {
                        can_sender
                            .send(AmpResetOutputMessage { output: 2 }.into())
                            .await;
                        NodeSelection::None
                    }
                    DeviceToReset::AMPOut3 => {
                        can_sender
                            .send(AmpResetOutputMessage { output: 3 }.into())
                            .await;
                        NodeSelection::None
                    }
                    DeviceToReset::AMPOut4 => {
                        can_sender
                            .send(AmpResetOutputMessage { output: 4 }.into())
                            .await;
                        NodeSelection::None
                    }

                    DeviceToReset::VoidLake => NodeSelection::NodeType(VOID_LAKE_NODE_TYPE),
                    DeviceToReset::AMP => NodeSelection::NodeType(AMP_NODE_TYPE),

                    DeviceToReset::Icarus => NodeSelection::NodeType(ICARUS_NODE_TYPE),
                    DeviceToReset::PayloadActivationPCB => {
                        NodeSelection::NodeType(PAYLOAD_ACTIVATION_NODE_TYPE)
                    }
                    DeviceToReset::RocketWifi => {
                        NodeSelection::NodeType(PAYLOAD_ROCKET_WIFI_NODE_TYPE)
                    }
                    DeviceToReset::OzysAll => NodeSelection::NodeType(OZYS_NODE_TYPE),
                    DeviceToReset::MainBulkhead => NodeSelection::NodeId(MAIN_BULKHEAD_NODE_ID),
                    DeviceToReset::DrogueBulkhead => NodeSelection::NodeId(DROGUE_BULKHEAD_NODE_ID),
                    DeviceToReset::PayloadEPS1 => NodeSelection::NodeType(PAYLOAD_EPS1_NODE_TYPE),
                    DeviceToReset::PayloadEPS2 => NodeSelection::NodeType(PAYLOAD_EPS2_NODE_TYPE),
                    DeviceToReset::AeroRust => NodeSelection::NodeType(AERO_RUST_NODE_TYPE),
                };
                info!("node selection: {}", node_selection);

                match node_selection {
                    NodeSelection::All => {
                        can_sender
                            .send(
                                ResetMessage {
                                    node_id: 0,
                                    reset_all: true,
                                    into_bootloader: false,
                                }
                                .into(),
                            )
                            .await;
                    }
                    NodeSelection::NodeType(node_type) => {
                        for node in can_central.get_nodes::<4>(node_type) {
                            can_sender
                                .send(
                                    ResetMessage {
                                        node_id: node.id,
                                        reset_all: false,
                                        into_bootloader: false,
                                    }
                                    .into(),
                                )
                                .await;
                        }
                    }
                    NodeSelection::NodeId(node_id) => {
                        can_sender
                            .send(
                                ResetMessage {
                                    node_id,
                                    reset_all: false,
                                    into_bootloader: false,
                                }
                                .into(),
                            )
                            .await;
                    }
                    NodeSelection::None => {}
                }
            }
            VLPUplinkPacket::PayloadEPSOutputOverwrite(packet) => {
                for eps1 in can_central.get_nodes::<2>(PAYLOAD_EPS1_NODE_TYPE) {
                    can_sender
                        .send(
                            PayloadEPSOutputOverwriteMessage {
                                out_3v3: packet.eps1_3v3,
                                out_5v: packet.eps1_5v,
                                out_9v: packet.eps1_9v,
                                node_id: eps1.id,
                            }
                            .into(),
                        )
                        .await;
                }

                for eps2 in can_central.get_nodes::<2>(PAYLOAD_EPS2_NODE_TYPE) {
                    can_sender
                        .send(
                            PayloadEPSOutputOverwriteMessage {
                                out_3v3: packet.eps2_3v3,
                                out_5v: packet.eps2_5v,
                                out_9v: packet.eps2_9v,
                                node_id: eps2.id,
                            }
                            .into(),
                        )
                        .await;
                }
            }
            VLPUplinkPacket::AMPOutputOverwrite(packet) => {
                can_sender
                    .send(
                        AmpOverwriteMessage {
                            out1: packet.out1,
                            out2: packet.out2,
                            out3: packet.out3,
                            out4: packet.out4,
                        }
                        .into(),
                    )
                    .await;
            }
            VLPUplinkPacket::FirePyro(packet) => {
                if avionics_mode.try_get() == Some(AvionicsMode::Armed) {
                    tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                    Timer::after_millis(1000).await;
                    tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                    Timer::after_millis(1000).await;
                    tone_queue.publish_immediate(BuzzerTone::Low(500, 500));
                    Timer::after_millis(1000).await;
                    fire_signal.signal(packet.pyro);
                }
            }
        }
    }
}
