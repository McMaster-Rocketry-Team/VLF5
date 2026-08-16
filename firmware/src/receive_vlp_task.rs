use defmt::info;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use firmware_common_new::{
    can_bus::{
        messages::{
            amp_overwrite::AmpOverwriteMessage, amp_reset_output::AmpResetOutputMessage,
            reset::ResetMessage,
        },
        node_types::{
            AERO_RUST_NODE_TYPE, AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE,
            PAYLOAD_SDRM_NODE_TYPE, VOID_LAKE_NODE_TYPE,
        },
        sender::CanSender,
    },
    vlp::{
        client::VLPAvionics,
        packets::{VLPUplinkPacket, reset::DeviceToReset},
    },
};

use crate::{
    AvionicsModeWatch, FireSignal, SetTargetWatch,
    avionics_mode::AvionicsMode,
    can_central::CanCentral,
    tasks::{
        buzzer_task::{BuzzerPubSub, BuzzerTone},
        sd_card_writer::{StorageCmdSignal, StorageCommand},
    },
};

#[embassy_executor::task]
pub async fn receive_vlp_task(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    fire_pyro_request: &'static FireSignal,
    target_agl_watch: &'static SetTargetWatch,
    can_sender: &'static CanSender<NoopRawMutex>,
    can_central: &'static CanCentral<NoopRawMutex>,
    storage_cmd: &'static StorageCmdSignal,
) {
    let avionics_mode = avionics_mode_watch.sender();

    loop {
        let (packet, _) = vlp_avionics_client.receive().await;
        match packet {
            VLPUplinkPacket::ChangeMode(packet) => {
                info!("changing mode into {}", packet);
                avionics_mode.send(packet.mode.into());
            }
            VLPUplinkPacket::Reset(packet) => {
                // No `NodeId` arm: every remaining reset target is a whole
                // node type. The bulkheads were the only devices that shared
                // a type and so had to be addressed individually.
                #[derive(defmt::Format)]
                enum NodeSelection {
                    All,
                    NodeType(u8),
                    None,
                }

                let node_selection = match packet.device {
                    DeviceToReset::All => NodeSelection::All,
                    DeviceToReset::AMPOut1 => {
                        can_sender.send(AmpResetOutputMessage { output: 1 }.into());
                        NodeSelection::None
                    }
                    DeviceToReset::AMPOut2 => {
                        can_sender.send(AmpResetOutputMessage { output: 2 }.into());
                        NodeSelection::None
                    }
                    DeviceToReset::AMPOut3 => {
                        can_sender.send(AmpResetOutputMessage { output: 3 }.into());
                        NodeSelection::None
                    }
                    // DeviceToReset::AMPOut4 => {
                    //     can_sender.send(AmpResetOutputMessage { output: 4 }.into());
                    //     NodeSelection::None
                    // }

                    DeviceToReset::VoidLake => NodeSelection::NodeType(VOID_LAKE_NODE_TYPE),
                    DeviceToReset::AMP => NodeSelection::NodeType(AMP_NODE_TYPE),

                    DeviceToReset::Icarus => NodeSelection::NodeType(ICARUS_NODE_TYPE),
                    DeviceToReset::PayloadSDRM => {
                        NodeSelection::NodeType(PAYLOAD_SDRM_NODE_TYPE)
                    }
                    DeviceToReset::OzysAll => NodeSelection::NodeType(OZYS_NODE_TYPE),
                    DeviceToReset::AeroRust => NodeSelection::NodeType(AERO_RUST_NODE_TYPE),
                };
                info!("node selection: {}", node_selection);

                match node_selection {
                    NodeSelection::All => {
                        can_sender.send(
                            ResetMessage {
                                node_id: 0,
                                reset_all: true,
                            }
                            .into(),
                        );
                    }
                    NodeSelection::NodeType(node_type) => {
                        for node in can_central.get_nodes::<4>(node_type) {
                            can_sender.send(
                                ResetMessage {
                                    node_id: node.id,
                                    reset_all: false,
                                }
                                .into(),
                            );
                        }
                    }
                    NodeSelection::None => {}
                }
            }
            VLPUplinkPacket::AMPOutputOverwrite(packet) => {
                can_sender.send(
                    AmpOverwriteMessage {
                        out1: packet.out1,
                        out2: packet.out2,
                        out3: packet.out3,
                        // out4: packet.out4,
                    }
                    .into(),
                );
            }
            VLPUplinkPacket::FirePyro(packet) => {
                if avionics_mode.try_get() == Some(AvionicsMode::Armed) {
                    // Hand the fire request off to `fire_pyro_countdown_task` so the
                    // ~3 s buzzer countdown runs off this loop. Blocking here would
                    // stop `vlp.receive()` from draining the rx Signal, silently
                    // dropping any uplinks that arrive during the countdown.
                    let _ = fire_pyro_request.try_send(packet.pyro);
                }
            }
            VLPUplinkPacket::SetTargetApogee(packet) => {
                let agl = packet.get_altitude_agl();
                info!("SetTargetApogee: {} m AGL (persisting to SD)", agl);
                target_agl_watch.sender().send(agl);
                storage_cmd.signal(StorageCommand::SaveTargetApogee(agl));
            }
        }
    }
}

/// Plays the manual `fire-pyro` buzzer countdown and then fires the pyro, off the
/// `receive_vlp_task` uplink loop.
///
/// `receive_vlp_task` hands a `PyroSelect` here via `fire_pyro_request` (a
/// non-blocking `try_send`) so its `vlp.receive()` loop keeps draining uplinks
/// instead of stalling for the ~3 s countdown. This task then drives the same
/// `fire_signal` the autonomous apogee-deploy path uses, so the pyro fires
/// identically. Autonomous deploy is unaffected: it still sends to `fire_signal`
/// directly and never passes through here.
#[embassy_executor::task]
pub async fn fire_pyro_countdown_task(
    fire_pyro_request: &'static FireSignal,
    fire_signal: &'static FireSignal,
    buzzer_pubsub: &'static BuzzerPubSub,
) {
    let buzzer_pub = buzzer_pubsub.immediate_publisher();

    loop {
        let pyro = fire_pyro_request.receive().await;
        buzzer_pub.publish_immediate(BuzzerTone::Low(500, 500));
        Timer::after_millis(1000).await;
        buzzer_pub.publish_immediate(BuzzerTone::Low(500, 500));
        Timer::after_millis(1000).await;
        buzzer_pub.publish_immediate(BuzzerTone::Low(500, 500));
        Timer::after_millis(1000).await;
        let _ = fire_signal.try_send(pyro);
    }
}
