use core::f32::consts::PI;

use crate::{
    AvionicsModeWatch, AirBrakesWatch, FlightStageMutex, GPSReadingWatch, publish_airbrakes_commanded,
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
    tasks::{
        amp_control_task::AmpControlWatch,
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub},
    },
    utils::SubscriberWithLastValue,
};
use defmt::info;
use embassy_futures::{join::join4, select::select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Ticker};
use firmware_common_new::{
    can_bus::{
        messages::{
            CanBusMessageEnum, airbrakes_control::AirBrakesControlMessage,
            amp_control::AmpControlMessage, vl_status::FlightStage,
        },
        node_types::AMP_NODE_TYPE,
        sender::CanSender,
    },
    vlp::{client::VLPAvionics, packets::low_power_telemetry::LowPowerTelemetryPacketBuilder},
};
use micromath::F32Ext;

pub async fn demo_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_sender: &'static CanSender<NoopRawMutex>,
    can_central: &'static CanCentral<NoopRawMutex>,
    gps_reading_watch: &'static GPSReadingWatch,
    battery_v_watch: &'static BatteryVWatch,
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mut can_receiver_sub: CanReceiverSub,
    amp_control_watch: &'static AmpControlWatch,
    air_brakes_watch: &'static AirBrakesWatch,
    flight_stage: &'static FlightStageMutex,
) {
    info!("enter demo mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::LowPower;
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true,
        out2_enable: false,
        out3_enable: false,
        out4_enable: false,
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

    let control_airbrakes_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(10));
        let frequency = 0.5f32;

        loop {
            let now_s = Instant::now().as_millis() as f32 / 1000.0;
            let airbrake_extension_percentage = (frequency * PI * 2.0 * now_s).sin() * 0.5 + 0.5;
            publish_airbrakes_commanded(air_brakes_watch, airbrake_extension_percentage);
            can_sender.send(AirBrakesControlMessage::new(airbrake_extension_percentage).into());

            ticker.next().await;
        }
    };

    let fut = join4(
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_packet_fut,
        control_airbrakes_fut,
    );

    let wait_low_power_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::Demo).await;
    };

    select(fut, wait_low_power_mode_end_fut).await;
}
