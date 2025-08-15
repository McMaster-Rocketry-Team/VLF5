use defmt::info;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        messages::{
            amp_control::AmpControlMessage, amp_status::{AmpStatusMessage, PowerOutputStatus}, vl_status::FlightStage, CanBusMessageEnum
        },
        node_types::{
            AMP_NODE_TYPE, BULKHEAD_NODE_TYPE, ICARUS_NODE_TYPE,
            OZYS_NODE_TYPE, PAYLOAD_ACTIVATION_NODE_TYPE, PAYLOAD_EPS1_NODE_TYPE,
            PAYLOAD_EPS2_NODE_TYPE, PAYLOAD_ROCKET_WIFI_NODE_TYPE,
        },
    },
    vlp::{
        client::VLPAvionics,
        packets::{self_test_result::{NodeStatus, SelfTestResultPacketBuilder}, VLPDownlinkPacket},
    },
};

use crate::{
    AvionicsModeWatch, DROGUE_BULKHEAD_NODE_ID, FlightStageMutex, MAIN_BULKHEAD_NODE_ID,
    OZYS_1_NODE_ID, OZYS_2_NODE_ID, VLStatusMutex,
    can::CanReceiverSub,
    can_central::CanCentral,
    tasks::amp_control_task::AmpControlWatch,
    utils::run_with_timeout,
};

pub async fn self_test_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    vl_status: &'static VLStatusMutex,
    amp_control_watch: &'static AmpControlWatch,
    flight_stage: &'static FlightStageMutex,
    mut can_receiver_sub: CanReceiverSub,
) {
    info!("enter self test mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::SelfTest;
    });

    let self_test_fut = async {
        let packet_builder = SelfTestResultPacketBuilder::<NoopRawMutex>::new();

        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: false,
            out2_enable: false,
            out3_enable: false,
            out4_enable: false,
        });
        Timer::after_millis(1000).await;
        can_central.clear();
        Timer::after_millis(1000).await;

        // test vl
        packet_builder.update(|packet| {
            vl_status.lock(|s| {
                let status = s.borrow();
                packet.imu_ok = status.imu_ok;
                packet.baro_ok = status.baro_ok;
                packet.mag_ok = status.mag_ok;
                packet.gps_ok = status.gps_ok;
                packet.sd_ok = status.sd_ok;
                packet.can_bus_ok = status.can_bus_ok;
            });
        });

        // test amp
        packet_builder.update(|packet| {
            if let Some(amp) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                packet.amp = NodeStatus::from_message(&amp.status);
            } else {
                packet.amp = NodeStatus::offline();
            }
        });

        // test bulkhead pcbs
        {
            packet_builder.update(|packet| {
                if let Some(main_bulkhead) = can_central
                    .get_nodes::<4>(BULKHEAD_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == MAIN_BULKHEAD_NODE_ID)
                {
                    packet.main_bulkhead_pcb = NodeStatus::from_message(&main_bulkhead.status);
                } else {
                    packet.main_bulkhead_pcb = NodeStatus::offline();
                }

                if let Some(drogue_bulkhead) = can_central
                    .get_nodes::<4>(BULKHEAD_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == DROGUE_BULKHEAD_NODE_ID)
                {
                    packet.drogue_bulkhead_pcb = NodeStatus::from_message(&drogue_bulkhead.status);
                } else {
                    packet.drogue_bulkhead_pcb = NodeStatus::offline();
                }
            });
        }

        // test amp out 1
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: true,
                out2_enable: false,
                out3_enable: false,
                out4_enable: false,
            });
            Timer::after_millis(10000).await; // longer time for ICARUS to home
            let out1_power_good = if let Some(amp_status_message) =
                get_amp_status_message(&mut can_receiver_sub).await
            {
                amp_status_message.out1.status == PowerOutputStatus::PowerGood
            } else {
                false
            };
            packet_builder.update(|packet| {
                packet.amp_out1_power_good = out1_power_good;
            });

            packet_builder.update(|packet| {
                if let Some(icarus) = can_central.get_nodes::<1>(ICARUS_NODE_TYPE).first() {
                    packet.icarus = NodeStatus::from_message(&icarus.status);
                } else {
                    packet.icarus = NodeStatus::offline();
                }
            });
        }

        // test amp out 2
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: false,
                out2_enable: true,
                out3_enable: false,
                out4_enable: false,
            });
            Timer::after_millis(10000).await; // longer time for payload activation pcb to connect to payload
            let out2_power_good = if let Some(amp_status_message) =
                get_amp_status_message(&mut can_receiver_sub).await
            {
                amp_status_message.out2.status == PowerOutputStatus::PowerGood
            } else {
                false
            };
            packet_builder.update(|packet| {
                packet.amp_out2_power_good = out2_power_good;
            });

            packet_builder.update(|packet| {
                if let Some(ozys_1) = can_central
                    .get_nodes::<4>(OZYS_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == OZYS_1_NODE_ID)
                {
                    packet.ozys1 = NodeStatus::from_message(&ozys_1.status);
                } else {
                    packet.ozys1 = NodeStatus::offline();
                }

                if let Some(payload_activation_pcb) = can_central
                    .get_nodes::<1>(PAYLOAD_ACTIVATION_NODE_TYPE)
                    .first()
                {
                    packet.payload_activation_pcb =
                        NodeStatus::from_message(&payload_activation_pcb.status);
                } else {
                    packet.payload_activation_pcb = NodeStatus::offline();
                }

                if let Some(rocket_wifi) = can_central
                    .get_nodes::<1>(PAYLOAD_ROCKET_WIFI_NODE_TYPE)
                    .first()
                {
                    packet.rocket_wifi = NodeStatus::from_message(&rocket_wifi.status);
                } else {
                    packet.rocket_wifi = NodeStatus::offline();
                }

                if let Some(eps_1) = can_central.get_nodes::<1>(PAYLOAD_EPS1_NODE_TYPE).first() {
                    packet.payload_eps1 = NodeStatus::from_message(&eps_1.status);
                } else {
                    packet.payload_eps1 = NodeStatus::offline();
                }

                if let Some(eps_2) = can_central.get_nodes::<1>(PAYLOAD_EPS2_NODE_TYPE).first() {
                    packet.payload_eps2 = NodeStatus::from_message(&eps_2.status);
                } else {
                    packet.payload_eps2 = NodeStatus::offline();
                }
            });
        }

        // test amp out 3
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: false,
                out2_enable: false,
                out3_enable: true,
                out4_enable: false,
            });
            Timer::after_millis(2000).await;
            let out3_power_good = if let Some(amp_status_message) =
                get_amp_status_message(&mut can_receiver_sub).await
            {
                amp_status_message.out3.status == PowerOutputStatus::PowerGood
            } else {
                false
            };
            packet_builder.update(|packet| {
                packet.amp_out3_power_good = out3_power_good;
            });

            packet_builder.update(|packet| {
                if let Some(ozys_2) = can_central
                    .get_nodes::<4>(OZYS_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == OZYS_2_NODE_ID)
                {
                    packet.ozys2 = NodeStatus::from_message(&ozys_2.status);
                } else {
                    packet.ozys2 = NodeStatus::offline();
                }
            });
        }

        // test amp out 4
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: false,
                out2_enable: false,
                out3_enable: false,
                out4_enable: true,
            });
            Timer::after_millis(2000).await;
            let out4_power_good = if let Some(amp_status_message) =
                get_amp_status_message(&mut can_receiver_sub).await
            {
                amp_status_message.out4.status == PowerOutputStatus::PowerGood
            } else {
                false
            };
            packet_builder.update(|packet| {
                packet.amp_out4_power_good = out4_power_good;
            });
        }

        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: false,
            out2_enable: false,
            out3_enable: false,
            out4_enable: false,
        });

        let packet: VLPDownlinkPacket = packet_builder.create_packet().into();
        info!("self test done: {}", packet);
        let mut ticker = Ticker::every(Duration::from_hz(2));

        loop {
            ticker.next().await;
            vlp_avionics_client.send(packet.clone());
        }
    };

    let wait_self_test_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed().await;
    };

    select(self_test_fut, wait_self_test_mode_end_fut).await;
}

async fn get_amp_status_message(can_receiver_sub: &mut CanReceiverSub) -> Option<AmpStatusMessage> {
    can_receiver_sub.clear();
    let wait_message_fut = async {
        loop {
            let message = can_receiver_sub.next_message_pure().await;
            if let CanBusMessageEnum::AmpStatus(message) = message.data.message {
                break message;
            }
        }
    };

    run_with_timeout(2000, wait_message_fut).await.ok()
}
