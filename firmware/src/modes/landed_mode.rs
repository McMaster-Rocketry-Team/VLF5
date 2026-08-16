use defmt::info;
use embassy_futures::{join::join4, select::select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        messages::{CanBusMessageEnum, amp_control::AmpControlMessage, vl_status::FlightStage},
        node_types::AMP_NODE_TYPE,
    },
    vlp::{client::VLPAvionics, packets::landed_telemetry::LandedTelemetryPacketBuilder},
};

use crate::{
    AvionicsModeWatch, FlightStageMutex, GPSReadingWatch,
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
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
        *r.borrow_mut() = (FlightStage::Landed, false);
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true, // leave camera on for 15 minutes for it to save its data
        out2_enable: false,
        out3_enable: false,
        out4_enable: false,
    });

    let stop_camera_fut = async {
        Timer::after_secs(60 * 15).await;
        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: false,
            out2_enable: false,
            out3_enable: false,
            out4_enable: false,
        });
    };

    let packet_builder = LandedTelemetryPacketBuilder::<NoopRawMutex>::new();

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
                let (lat, lon) = gps_data.lat_lon.unwrap_or((0.0, 0.0));
                packet.lat = lat;
                packet.lon = lon;
                packet.battery_v = battery_v;

                if let Some(node) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                    packet.amp_online = node.is_online();
                    packet.amp_rebooted_in_last_5s = node.rebooted_in_last_5s();
                } else {
                    packet.amp_online = false;
                    packet.amp_rebooted_in_last_5s = false;
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
                    packet_builder.update(|packet| {
                        packet.shared_battery_v = message.shared_battery_mv as f32 / 1000.0;
                        packet.amp_out1 = message.out1.status;
                        packet.amp_out1_overwrote = message.out1.overwrote;
                        packet.amp_out2 = message.out1.status;
                        packet.amp_out2_overwrote = message.out1.overwrote;
                        packet.amp_out3 = message.out1.status;
                        packet.amp_out3_overwrote = message.out1.overwrote;
                        packet.amp_out4 = message.out1.status;
                        packet.amp_out4_overwrote = message.out1.overwrote;
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
