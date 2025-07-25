use embassy_futures::{join::join4, select::select};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker};
use firmware_common_new::{
    can_bus::{
        messages::{
            CanBusMessageEnum,
            avionics_status::{AvionicsStatusMessage, FlightStage},
        },
        node_types::AMP_NODE_TYPE,
        sender::CanSender,
    },
    gps::GPSData,
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::{client::VLPAvionics, packets::landed_telemetry::LandedTelemetryPacketBuilder},
};

use crate::{avionics_mode::AvionicsMode, can::CanReceiverSub, can_central::CanCentral, tasks::unix_clock::UnixClock};

pub async fn landed_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
    can_central: &'static CanCentral<NoopRawMutex>,
    mut gps_reading: watch::DynReceiver<'static, SensorReading<BootTimestamp, GPSData>>,
    mut battery_v_reading: watch::DynReceiver<'static, SensorReading<BootTimestamp, f32>>,
    can_sender: &'static CanSender<NoopRawMutex, &'static UnixClock, 16>,
    mut can_receiver_sub: CanReceiverSub,
) {
    let packet_builder = LandedTelemetryPacketBuilder::<NoopRawMutex>::new();

    let update_packet_sensor_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(1));
        gps_reading.get().await;
        battery_v_reading.get().await;

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
                        flight_stage: FlightStage::LowPower,
                    }
                    .into(),
                )
                .await;
            ticker.next().await;
        }
    };

    let fut = join4(
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_packet_fut,
        send_avionics_status_fut,
    );

    let wait_landed_mode_end_fut = async {
        let mut receiver = avionics_mode.receiver().unwrap();
        while receiver.get().await == AvionicsMode::Landed {}
    };

    select(fut, wait_landed_mode_end_fut).await;
}
