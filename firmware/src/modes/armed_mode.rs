use core::cell::RefCell;

use air_brakes_controller_core::{
    AirBrakesMPC, Measurement, RocketState, RocketStateEstimator, approximate_speed_of_sound,
};
use defmt::info;
use embassy_futures::{
    join::{join, join5},
    select::select,
};
use embassy_sync::{
    blocking_mutex::{Mutex as BlockingMutex, raw::NoopRawMutex},
    signal::Signal,
};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        messages::{
            CanBusMessageEnum, airbrakes_control::AirBrakesControlMessage,
            amp_control::AmpControlMessage, rocket_state::RocketStateMessage,
            vl_status::FlightStage,
        },
        node_types::{
            AMP_NODE_TYPE, BULKHEAD_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE,
            PAYLOAD_ACTIVATION_NODE_TYPE, PAYLOAD_EPS1_NODE_TYPE, PAYLOAD_EPS2_NODE_TYPE,
            PAYLOAD_ROCKET_WIFI_NODE_TYPE,
        },
        sender::CanSender,
    },
    sensor_reading::SensorReading,
    vlp::{client::VLPAvionics, packets::telemetry::TelemetryPacketBuilder},
};
use nalgebra::Vector2;

use crate::{
    AvionicsModeWatch, AirBrakesWatch, ContinuityWatch, DROGUE_BULKHEAD_NODE_ID, FLIGHT_PROFILE,
    FireSignal, FlightStageMutex, GPSReadingWatch, MAIN_BULKHEAD_NODE_ID, OZYS_1_NODE_ID, OZYS_2_NODE_ID,
    ROCKET_PARAMETERS, SetTargetWatch, publish_airbrakes_commanded,
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
    tasks::{
        amp_control_task::AmpControlWatch,
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub},
        unix_clock::UnixClock,
    },
    utils::SubscriberWithLastValue,
};

pub async fn armed_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_sender: &'static CanSender<NoopRawMutex>,
    can_central: &'static CanCentral<NoopRawMutex>,
    gps_reading_watch: &'static GPSReadingWatch,
    battery_v_watch: &'static BatteryVWatch,
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    continuity_watch: &'static ContinuityWatch,
    fire_signal: &'static FireSignal,
    target_agl_watch: &'static SetTargetWatch,
    mut can_receiver_sub: CanReceiverSub,
    flight_stage: &'static FlightStageMutex,
    amp_control_watch: &'static AmpControlWatch,
    air_brakes_watch: &'static AirBrakesWatch,
    unix_clock: &'static UnixClock,
) {
    info!("enter armed mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::Armed;
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true,
        out2_enable: false,
        out3_enable: true,
        out4_enable: true,
    });

    let packet_builder = TelemetryPacketBuilder::<NoopRawMutex>::new();
    let state_estimator = BlockingMutex::<NoopRawMutex, _>::new(RefCell::new(
        RocketStateEstimator::new(FLIGHT_PROFILE.clone()),
    ));

    let update_packet_sensor_fut = async {
        let mut imu_baro_sub = SubscriberWithLastValue::new(imu_baro_pubsub).unwrap();

        let mut gps_reading = gps_reading_watch.receiver().unwrap();
        let mut battery_v_reading = battery_v_watch.receiver().unwrap();
        let mut continuity = continuity_watch.receiver().unwrap();
        gps_reading.get().await;
        battery_v_reading.get().await;
        continuity.get().await;

        let mut ticker = Ticker::every(Duration::from_hz(5));

        loop {
            let gps_data = gps_reading.try_get().unwrap().data;
            let battery_v = battery_v_reading.try_get().unwrap().data;
            let baro_data = imu_baro_sub.get().await.data.1;
            let continuity = continuity.try_get().unwrap();
            packet_builder.update(|packet| {
                packet.gps_location = Some(gps_data);
                packet.vl_battery_v = battery_v;
                packet.air_temperature = baro_data.temperature;

                packet.pyro_main_continuity = continuity.pyro_main_continuity;
                packet.pyro_drogue_continuity = continuity.pyro_drogue_continuity;

                if let Some(amp) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                    packet.amp_online = amp.is_online();
                    packet.amp_uptime_s = amp.status.uptime_s;
                } else {
                    packet.amp_online = false;
                    packet.amp_uptime_s = 0;
                }

                if let Some(main_bulkhead) = can_central
                    .get_nodes::<4>(BULKHEAD_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == MAIN_BULKHEAD_NODE_ID)
                {
                    packet.main_bulkhead_online = main_bulkhead.is_online();
                    packet.main_bulkhead_uptime_s = main_bulkhead.status.uptime_s;
                } else {
                    packet.main_bulkhead_online = false;
                    packet.main_bulkhead_uptime_s = 0;
                }

                if let Some(drogue_bulkhead) = can_central
                    .get_nodes::<4>(BULKHEAD_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == DROGUE_BULKHEAD_NODE_ID)
                {
                    packet.drogue_bulkhead_online = drogue_bulkhead.is_online();
                    packet.drogue_bulkhead_uptime_s = drogue_bulkhead.status.uptime_s;
                } else {
                    packet.drogue_bulkhead_online = false;
                    packet.drogue_bulkhead_uptime_s = 0;
                }

                if let Some(icarus) = can_central.get_nodes::<1>(ICARUS_NODE_TYPE).first() {
                    packet.icarus_online = icarus.is_online();
                    packet.icarus_uptime_s = icarus.status.uptime_s;
                } else {
                    packet.icarus_online = false;
                    packet.icarus_uptime_s = 0;
                }

                if let Some(ozys_1) = can_central
                    .get_nodes::<4>(OZYS_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == OZYS_1_NODE_ID)
                {
                    packet.ozys1_online = ozys_1.is_online();
                    packet.ozys1_uptime_s = ozys_1.status.uptime_s;
                } else {
                    packet.ozys1_online = false;
                    packet.ozys1_uptime_s = 0;
                }

                if let Some(ozys_2) = can_central
                    .get_nodes::<4>(OZYS_NODE_TYPE)
                    .iter()
                    .find(|node| node.id == OZYS_2_NODE_ID)
                {
                    packet.ozys2_online = ozys_2.is_online();
                    packet.ozys2_uptime_s = ozys_2.status.uptime_s;
                } else {
                    packet.ozys2_online = false;
                    packet.ozys2_uptime_s = 0;
                }

                if let Some(payload_activation_pcb) = can_central
                    .get_nodes::<1>(PAYLOAD_ACTIVATION_NODE_TYPE)
                    .first()
                {
                    packet.payload_activation_pcb_online = payload_activation_pcb.is_online();
                    packet.payload_activation_pcb_uptime_s = payload_activation_pcb.status.uptime_s;
                } else {
                    packet.payload_activation_pcb_online = false;
                    packet.payload_activation_pcb_uptime_s = 0;
                }

                if let Some(rocket_wifi) = can_central
                    .get_nodes::<1>(PAYLOAD_ROCKET_WIFI_NODE_TYPE)
                    .first()
                {
                    packet.rocket_wifi_online = rocket_wifi.is_online();
                    packet.rocket_wifi_uptime_s = rocket_wifi.status.uptime_s;
                } else {
                    packet.rocket_wifi_online = false;
                    packet.rocket_wifi_uptime_s = 0;
                }

                if let Some(eps_1) = can_central.get_nodes::<1>(PAYLOAD_EPS1_NODE_TYPE).first() {
                    packet.eps1_online = eps_1.is_online();
                    packet.eps1_uptime_s = eps_1.status.uptime_s;
                } else {
                    packet.eps1_online = false;
                    packet.eps1_uptime_s = 0;
                }

                if let Some(eps_2) = can_central.get_nodes::<1>(PAYLOAD_EPS2_NODE_TYPE).first() {
                    packet.eps2_online = eps_2.is_online();
                    packet.eps2_uptime_s = eps_2.status.uptime_s;
                } else {
                    packet.eps2_online = false;
                    packet.eps2_uptime_s = 0;
                }

                state_estimator.lock(|s| {
                    let estimator = s.borrow();

                    match estimator.state() {
                        RocketState::OnPad => {
                            packet.altitude_agl = 0.0;
                            packet.air_speed = 0.0;
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Armed;
                        }
                        RocketState::PoweredAscent {
                            velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        } => {
                            packet.altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.air_speed = velocity.magnitude();
                            packet.tilt_deg = velocity.angle(&Vector2::new(0.0, 1.0)).to_degrees();
                            packet.flight_stage = FlightStage::PoweredAscent;
                        }
                        RocketState::Coasting {
                            velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        } => {
                            packet.altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.air_speed = velocity.magnitude();
                            packet.tilt_deg = velocity.angle(&Vector2::new(0.0, 1.0)).to_degrees();
                            packet.flight_stage = FlightStage::Coasting;
                        }
                        RocketState::DrogueChute {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                            ..
                        } => {
                            packet.altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.air_speed = vertical_velocity.abs();
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Coasting;
                        }
                        RocketState::MainChute {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                            ..
                        } => {
                            packet.altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.air_speed = vertical_velocity.abs();
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Coasting;
                        }
                        RocketState::Landed => {
                            packet.altitude_agl = 0.0;
                            packet.air_speed = 0.0;
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Landed;
                        }
                        RocketState::FailedToReachMinApogee => {
                            packet.altitude_agl = 0.0;
                            packet.air_speed = 0.0;
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Landed;
                        }
                    }
                });
            });

            ticker.next().await;
        }
    };

    let update_packet_can_fut = async {
        loop {
            let message = can_receiver_sub.next_message_pure().await.data;
            let node_id = message.id.node_id;
            let node_type = message.id.node_type;
            packet_builder.update(|packet| match message.message {
                CanBusMessageEnum::AmpStatus(message) => {
                    packet.shared_battery_v = message.shared_battery_mv as f32 / 1000.0;
                    packet.amp_out1_overwrote = message.out1.overwrote;
                    packet.amp_out1 = message.out1.status;
                    packet.amp_out2_overwrote = message.out2.overwrote;
                    packet.amp_out2 = message.out2.status;
                    packet.amp_out3_overwrote = message.out3.overwrote;
                    packet.amp_out3 = message.out3.status;
                    packet.amp_out4_overwrote = message.out4.overwrote;
                    packet.amp_out4 = message.out4.status;
                }
                CanBusMessageEnum::BrightnessMeasurement(message) => {
                    if node_id == MAIN_BULKHEAD_NODE_ID {
                        packet.main_bulkhead_brightness = message.brightness_lux();
                    } else if node_id == DROGUE_BULKHEAD_NODE_ID {
                        packet.drogue_bulkhead_brightness = message.brightness_lux();
                    }
                }
                CanBusMessageEnum::IcarusStatus(message) => {
                    packet.air_brakes_actual_extension_percentage =
                        message.actual_extension_percentage();
                    packet.air_brakes_servo_temp = message.servo_temperature();
                }
                CanBusMessageEnum::PayloadEPSStatus(message) => {
                    if node_type == PAYLOAD_EPS1_NODE_TYPE {
                        packet.eps1_battery1_v = message.battery1_mv as f32 / 1000.0;
                        packet.eps1_battery1_temperature = message.battery1_temperature();
                        packet.eps1_battery2_v = message.battery2_mv as f32 / 1000.0;
                        packet.eps1_battery2_temperature = message.battery2_temperature();
                        packet.eps1_output_3v3_current =
                            message.output_3v3.current_ma as f32 / 1000.0;
                        packet.eps1_output_3v3_overwrote = message.output_3v3.overwrote;
                        packet.eps1_output_3v3_status = message.output_3v3.status;
                        packet.eps1_output_5v_current =
                            message.output_5v.current_ma as f32 / 1000.0;
                        packet.eps1_output_5v_overwrote = message.output_5v.overwrote;
                        packet.eps1_output_5v_status = message.output_5v.status;
                        packet.eps1_output_9v_current =
                            message.output_9v.current_ma as f32 / 1000.0;
                        packet.eps1_output_9v_overwrote = message.output_9v.overwrote;
                        packet.eps1_output_9v_status = message.output_9v.status;
                    } else if node_type == PAYLOAD_EPS2_NODE_TYPE {
                        packet.eps2_battery1_v = message.battery1_mv as f32 / 1000.0;
                        packet.eps2_battery1_temperature = message.battery1_temperature();
                        packet.eps2_battery2_v = message.battery2_mv as f32 / 1000.0;
                        packet.eps2_battery2_temperature = message.battery2_temperature();
                        packet.eps2_output_3v3_current =
                            message.output_3v3.current_ma as f32 / 1000.0;
                        packet.eps2_output_3v3_overwrote = message.output_3v3.overwrote;
                        packet.eps2_output_3v3_status = message.output_3v3.status;
                        packet.eps2_output_5v_current =
                            message.output_5v.current_ma as f32 / 1000.0;
                        packet.eps2_output_5v_overwrote = message.output_5v.overwrote;
                        packet.eps2_output_5v_status = message.output_5v.status;
                        packet.eps2_output_9v_current =
                            message.output_9v.current_ma as f32 / 1000.0;
                        packet.eps2_output_9v_overwrote = message.output_9v.overwrote;
                        packet.eps2_output_9v_status = message.output_9v.status;
                    }
                }
                _ => {}
            });
        }
    };

    let send_telemetry_packet_fut = async {
        let mut ticker = Ticker::every(Duration::from_secs(2));

        loop {
            ticker.next().await;

            vlp_avionics_client.send(packet_builder.create_packet().into());
        }
    };

    let send_rocket_state_fut = async {
        let mut ticker = Ticker::every(Duration::from_hz(10));

        loop {
            let state = state_estimator.lock(|s| s.borrow().state());

            let message = match state {
                RocketState::PoweredAscent {
                    velocity,
                    altitude_asl,
                    launch_pad_altitude_asl,
                } => Some(RocketStateMessage::new(
                    unix_clock.now_us_or_boot_time(),
                    &velocity.into(),
                    altitude_asl - launch_pad_altitude_asl,
                    false,
                )),
                RocketState::Coasting {
                    velocity,

                    altitude_asl,
                    launch_pad_altitude_asl,
                } => Some(RocketStateMessage::new(
                    unix_clock.now_us_or_boot_time(),
                    &velocity.into(),
                    altitude_asl - launch_pad_altitude_asl,
                    true,
                )),
                _ => None,
            };

            if let Some(message) = message {
                can_sender.send(message.into());
            }

            ticker.next().await;
        }
    };

    let start_airbrakes_signal = Signal::<NoopRawMutex, ()>::new();
    let update_state_estimator_fut = async {
        let mut airbrakes_started = false;
        let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();

        loop {
            let reading = imu_baro_sub.next_message_pure().await;
            if let SensorReading {
                data: (Some(imu_data), baro_data),
                ..
            } = reading
            {
                let (pyro, state) = state_estimator.lock(|s| {
                    let mut estimator = s.borrow_mut();

                    let pyro = estimator.update(&Measurement::new(
                        &imu_data.acc,
                        &imu_data.gyro.map(|d| d.to_radians()),
                        baro_data.altitude_asl(),
                    ));

                    let state = estimator.state();

                    (pyro, state)
                });

                if let Some(pyro) = pyro {
                    fire_signal.signal(pyro);
                }

                if !airbrakes_started
                    && let RocketState::Coasting {
                        velocity,
                        altitude_asl,
                        ..
                    } = &state
                    && velocity.magnitude() <= 0.85 * approximate_speed_of_sound(*altitude_asl)
                {
                    start_airbrakes_signal.signal(());
                    airbrakes_started = true;
                }

                if let RocketState::Landed | RocketState::FailedToReachMinApogee = state {
                    Timer::after_secs(30).await;
                    avionics_mode_watch.sender().send(AvionicsMode::Landed);
                }

                flight_stage.lock(|r| {
                    *r.borrow_mut() = match state {
                        RocketState::OnPad => FlightStage::Armed,
                        RocketState::PoweredAscent { .. } => FlightStage::PoweredAscent,
                        RocketState::Coasting { .. } => FlightStage::Coasting,
                        RocketState::DrogueChute { .. } => FlightStage::DrogueDeployed,
                        RocketState::MainChute { .. } => FlightStage::MainDeployed,
                        RocketState::Landed | RocketState::FailedToReachMinApogee => {
                            FlightStage::Landed
                        }
                    };
                });
            }
        }
    };

    let control_airbrakes_fut = async {
        publish_airbrakes_commanded(air_brakes_watch, 0.0);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
        });

        start_airbrakes_signal.wait().await;
        let launch_pad_altitude_asl = state_estimator.lock(|s| {
            let estimator = s.borrow();
            if let RocketState::Coasting {
                launch_pad_altitude_asl,
                ..
            } = estimator.state()
            {
                launch_pad_altitude_asl
            } else {
                0.0
            }
        });

        let mut airbrakes_mpc = AirBrakesMPC::new(
            ROCKET_PARAMETERS.clone(),
            launch_pad_altitude_asl + target_agl_watch.try_get().unwrap_or(4000.0), // note : you may want to change this default value on launch day just incase
        );

        let mut ticker = Ticker::every(Duration::from_hz(10));
        loop {
            let state = state_estimator.lock(|s| s.borrow().state());

            if let RocketState::Coasting {
                velocity,
                altitude_asl,
                ..
            } = state
            {
                let airbrake_extension_percentage = airbrakes_mpc.update(altitude_asl, velocity);
                publish_airbrakes_commanded(air_brakes_watch, airbrake_extension_percentage);
                can_sender.send(AirBrakesControlMessage::new(airbrake_extension_percentage).into());
                packet_builder.update(|packet| {
                    packet.air_brakes_commanded_extension_percentage =
                        airbrake_extension_percentage;
                });
            } else {
                break;
            }

            ticker.next().await;
        }

        publish_airbrakes_commanded(air_brakes_watch, 0.0);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
        });
    };

    let wait_armed_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::Armed).await;
    };

    let fut = join5(
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_telemetry_packet_fut,
        send_rocket_state_fut,
        join(update_state_estimator_fut, control_airbrakes_fut),
    );

    select(fut, wait_armed_mode_end_fut).await;
}
