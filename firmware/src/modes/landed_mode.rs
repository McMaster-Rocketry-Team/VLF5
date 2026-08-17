use defmt::info;
use embassy_futures::{join::join4, select::select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        messages::{
            CanBusMessageEnum, amp_control::AmpControlMessage,
            amp_status::PowerOutputStatus, vl_status::FlightStage,
        },
        node_types::AMP_NODE_TYPE,
    },
    vlp::{client::VLPAvionics, packets::landed_telemetry::LandedTelemetryPacketBuilder},
};

use crate::{
    AvionicsModeWatch, FlightStageMutex, GPSReadingWatch,
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
    modes::{StatusStreamReceipt, mark_status_received, status_stream_stale},
    tasks::{amp_control_task::AmpControlWatch, sensor_tasks::BatteryVWatch},
};

pub async fn landed_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    gps_reading_watch: &'static GPSReadingWatch,
    battery_v_watch: &'static BatteryVWatch,
    mut can_receiver_sub: CanReceiverSub,
    amp_control_watch: &'static AmpControlWatch,
    flight_stage: &'static FlightStageMutex,
) {
    info!("enter landed mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::Landed;
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true,
        out2_enable: true, // leave camera on for 15 minutes for it to save its data
        out3_enable: false,
    });

    let stop_camera_fut = async {
        Timer::after_secs(60 * 15).await;
        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: true,
            out2_enable: false,
            out3_enable: false,
        });
    };

    let packet_builder = LandedTelemetryPacketBuilder::<NoopRawMutex>::new();

    // When AMP's status stream last delivered. Landed mode is where a frozen
    // relayed value does the most damage: this mode ends only on an operator
    // uplink, and out 1 is deliberately left powered, so a search party can be
    // reading a battery voltage sampled hours ago and still rendered as a
    // healthy pack.
    let amp_status_receipt = StatusStreamReceipt::new(None);

    let update_packet_sensor_fut = async {
        let mut gps_reading = gps_reading_watch.receiver().unwrap();
        let mut battery_v_reading = battery_v_watch.receiver().unwrap();
        gps_reading.get().await;
        battery_v_reading.get().await;

        let mut ticker = Ticker::every(Duration::from_hz(1));

        loop {
            let gps_data = gps_reading.try_get().unwrap().data;
            let battery_v = battery_v_reading.try_get().unwrap().data;

            packet_builder.update(|packet| {
                packet.num_of_fix_satellites = gps_data.num_of_fix_satellites;
                // Passed through as the `Option` it already is: this is the
                // packet a search party navigates to, so "no fix yet" must not
                // arrive as a coordinate in the Gulf of Guinea.
                packet.lat_lon = gps_data.lat_lon;
                packet.battery_v = battery_v;

                if let Some(node) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                    packet.amp_online = node.is_online();
                    packet.amp_rebooted_in_last_5s = node.rebooted_in_last_5s();
                } else {
                    packet.amp_online = false;
                    packet.amp_rebooted_in_last_5s = false;
                }

                // The fields below come only from `update_packet_can_fut`'s
                // `AmpStatus` arm, so nothing else ever clears them. Note this
                // is a different question from `amp_online` just above, which
                // is the heartbeat's answer and stays as it is: AMP's status
                // task and its heartbeat are independent, so silence here does
                // not mean the node is gone.
                if status_stream_stale(&amp_status_receipt) {
                    packet.shared_battery_v = None;
                    // `Unknown` rather than `Disabled` — `Disabled` is an
                    // output AMP reports as commanded off, a deliberate state
                    // it is not entitled to claim on AMP's behalf.
                    packet.amp_out1 = PowerOutputStatus::Unknown;
                    packet.amp_out2 = PowerOutputStatus::Unknown;
                    packet.amp_out3 = PowerOutputStatus::Unknown;
                    packet.amp_out1_overwrote = false;
                    packet.amp_out2_overwrote = false;
                    packet.amp_out3_overwrote = false;
                }
            });

            ticker.next().await;
        }
    };

    let update_packet_can_fut = async {
        loop {
            let message = can_receiver_sub.next_message_pure().await.data.message;
            match message {
                CanBusMessageEnum::AmpStatus(message) => {
                    // Stamped beside the fields it fills, so the receipt and the
                    // values it vouches for cannot drift apart.
                    mark_status_received(&amp_status_receipt);
                    packet_builder.update(|packet| {
                        packet.shared_battery_v = Some(message.shared_battery_mv as f32 / 1000.0);
                        packet.amp_out1 = message.out1.status;
                        packet.amp_out1_overwrote = message.out1.overwrote;
                        packet.amp_out2 = message.out2.status;
                        packet.amp_out2_overwrote = message.out2.overwrote;
                        packet.amp_out3 = message.out3.status;
                        packet.amp_out3_overwrote = message.out3.overwrote;
                        // packet.amp_out4 = message.out1.status;
                        // packet.amp_out4_overwrote = message.out1.overwrote;
                    });
                }
                _ => {}
            }
        }
    };

    let send_packet_fut = async {
        let mut ticker = Ticker::every(Duration::from_secs(5));

        loop {
            ticker.next().await;

            vlp_avionics_client.send(packet_builder.create_packet().into());
        }
    };

    let fut = join4(
        stop_camera_fut,
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_packet_fut,
    );

    let wait_landed_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::Landed).await;
    };

    select(fut, wait_landed_mode_end_fut).await;
}
