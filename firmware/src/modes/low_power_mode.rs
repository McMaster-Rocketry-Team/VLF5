use defmt::info;
use embassy_futures::{join::join3, select::select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker};
use firmware_common_new::{
    can_bus::{
        messages::{amp_control::AmpControlMessage, vl_status::FlightStage, CanBusMessageEnum},
        node_types::AMP_NODE_TYPE,
    },
    vlp::{client::VLPAvionics, packets::low_power_telemetry::LowPowerTelemetryPacketBuilder},
};

use crate::{
    avionics_mode::AvionicsMode, can::CanReceiverSub, can_central::CanCentral, tasks::{amp_control_task::AmpControlWatch, sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub}}, utils::SubscriberWithLastValue, AvionicsModeWatch, FlightStageMutex, GPSReadingWatch
};

pub async fn low_power_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    gps_reading_watch: &'static GPSReadingWatch,
    battery_v_watch: &'static BatteryVWatch,
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mut can_receiver_sub: CanReceiverSub,
    amp_control_watch: &'static AmpControlWatch,
    flight_stage: &'static FlightStageMutex,
) {
    info!("enter low power mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = (FlightStage::LowPower, false);
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: false,
        out2_enable: false,
        out3_enable: false,
    });

    let packet_builder = LowPowerTelemetryPacketBuilder::<NoopRawMutex>::new();

    let update_packet_sensor_fut = async {
        let mut imu_baro_sub = SubscriberWithLastValue::new(imu_baro_pubsub).unwrap();
        let mut gps_reading = gps_reading_watch.receiver().unwrap();
        let mut battery_v_reading = battery_v_watch.receiver().unwrap();
        gps_reading.get().await;
        battery_v_reading.get().await;

        let mut ticker = Ticker::every(Duration::from_hz(1));

        loop {
            let gps_data = gps_reading.try_get().unwrap().data;
            let battery_v = battery_v_reading.try_get().unwrap().data;
            let baro_data = imu_baro_sub.get().await.data.1;
            packet_builder.update(|packet| {
                packet.num_of_fix_satellites = gps_data.num_of_fix_satellites;
                packet.gps_fixed = gps_data.lat_lon.is_some();
                packet.vl_battery_v = battery_v;
                packet.air_temperature = baro_data.temperature;
                packet.amp_online = can_central
                    .get_nodes::<1>(AMP_NODE_TYPE)
                    .first()
                    .map(|node| node.is_online())
                    .unwrap_or(false)
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

    let fut = join3(
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_packet_fut,
    );

    let wait_low_power_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::LowPower).await;
    };

    select(fut, wait_low_power_mode_end_fut).await;
}
