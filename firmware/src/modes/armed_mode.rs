use core::cell::RefCell;

use air_brakes_controller_core::{AirBrakesMPC, FlightEstimators, ImuSample, RocketState};
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
    flight_data_record::{
        AB_APOGEE, AB_BARO_TRUSTED, AB_VOTE_BARO_RATE, AB_VOTE_DEPLOYMENT, AB_VOTE_INERTIAL,
    },
    can_bus::{
        custom_status::payload_sdrm_custom_status::PayloadSDRMCustomStatus,
        messages::{
            CanBusMessageEnum, airbrakes_control::AirBrakesControlMessage,
            amp_control::AmpControlMessage, custom_payload_status::CustomPayloadStatusMessage,
            rocket_state::RocketStateMessage, vl_status::FlightStage,
        },
        node_types::{
            AMP_NODE_TYPE, BULKHEAD_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE,
            PAYLOAD_SDRM_NODE_TYPE,
        },
        sender::CanSender,
    },
    vlp::{client::VLPAvionics, packets::telemetry::TelemetryPacketBuilder},
};

use crate::{
    AIRBRAKES_CONFIG, AvionicsModeWatch, AirBrakesWatch, AirbrakesStateWatch, ContinuityWatch,
    DROGUE_BULKHEAD_NODE_ID, FLIGHT_PROFILE,
    FireSignal, FlightStageMutex, GPSReadingWatch, KfStateWatch,
    MAIN_BULKHEAD_NODE_ID, OZYS_1_NODE_ID, OZYS_2_NODE_ID,
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
    kf_state_watch: &'static KfStateWatch,
    airbrakes_state_watch: &'static AirbrakesStateWatch,
    amp_control_watch: &'static AmpControlWatch,
    air_brakes_watch: &'static AirBrakesWatch,
    unix_clock: &'static UnixClock,
) {
    info!("enter armed mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = (FlightStage::Armed, false);
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true,
        out2_enable: false,
        out3_enable: true,
    });

    let packet_builder = TelemetryPacketBuilder::<NoopRawMutex>::new();
    // One struct owns both estimators and the policy connecting them (the V2
    // abstain during the slow filter's Mach lockout, the airbrakes-permission
    // gate inside `airbrakes_mpc_states`) — firmware holds it behind one mutex.
    let estimators = BlockingMutex::<NoopRawMutex, _>::new(RefCell::new(FlightEstimators::new(
        FLIGHT_PROFILE.clone(),
        AIRBRAKES_CONFIG.clone(),
    )));

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

                if let Some(payload_sdrm) = can_central
                    .get_nodes::<1>(PAYLOAD_SDRM_NODE_TYPE)
                    .first()
                {
                    packet.payload_sdrm_online = payload_sdrm.is_online();
                    packet.payload_sdrm_uptime_s = payload_sdrm.status.uptime_s;
                    // Stack flags ride along in the node status heartbeat.
                    packet.payload_stack_status = payload_sdrm
                        .status
                        .custom_status::<PayloadSDRMCustomStatus>();
                } else {
                    packet.payload_sdrm_online = false;
                    packet.payload_sdrm_uptime_s = 0;
                    packet.payload_stack_status = PayloadSDRMCustomStatus::new();
                }

                estimators.lock(|s| {
                    let est = s.borrow();
                    // Coasting is the deployment estimator's burn-timer flag,
                    // orthogonal to the stage — never folded into it.
                    packet.coasting = est.deployment_estimator().is_coasting();
                    // Tilt is the airbrakes estimator's gyro dead reckoning —
                    // the field finally has a real source (0 before ignition
                    // and after its apogee latch).
                    let ab_tilt_deg = est
                        .airbrakes_estimator()
                        .tilt()
                        .map(|t| t.to_degrees())
                        .unwrap_or(0.0);

                    // The flight part of the stage mirrors `RocketState` 1:1.
                    match est.state() {
                        RocketState::OnPad => {
                            packet.altitude_agl = 0.0;
                            packet.air_speed = 0.0;
                            packet.tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Armed;
                        }
                        RocketState::Ascent {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        } => {
                            packet.altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.air_speed = vertical_velocity.abs();
                            packet.tilt_deg = ab_tilt_deg;
                            packet.flight_stage = FlightStage::Ascent;
                        }
                        RocketState::MachLockout { .. } => {
                            // The slow KF is frozen here and the state carries
                            // no altitude/velocity — report zeros rather than
                            // stale numbers; the stage tells the ground why.
                            packet.altitude_agl = 0.0;
                            packet.air_speed = 0.0;
                            packet.tilt_deg = ab_tilt_deg;
                            packet.flight_stage = FlightStage::MachLockout;
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
                            packet.flight_stage = FlightStage::DrogueChute;
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
                            packet.flight_stage = FlightStage::MainChute;
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
                            packet.flight_stage = FlightStage::FailedToReachMinApogee;
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
            packet_builder.update(|packet| match message.message {
                CanBusMessageEnum::AmpStatus(message) => {
                    packet.shared_battery_v = message.shared_battery_mv as f32 / 1000.0;
                    packet.amp_out1_overwrote = message.out1.overwrote;
                    packet.amp_out1 = message.out1.status;
                    packet.amp_out2_overwrote = message.out2.overwrote;
                    packet.amp_out2 = message.out2.status;
                    packet.amp_out3_overwrote = message.out3.overwrote;
                    packet.amp_out3 = message.out3.status;
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
                CanBusMessageEnum::CustomPayloadStatus(message) => {
                    // 0xFFFF means the payload could not read that rail; keep it as
                    // None so the ground station shows "n/a" instead of a fake 0V.
                    let rail_v = |raw_mv: u16| {
                        CustomPayloadStatusMessage::rail_mv(raw_mv).map(|mv| mv as f32 / 1000.0)
                    };
                    packet.epm_batt_v = rail_v(message.epm_batt_mv);
                    packet.epm_sys_3v3_v = rail_v(message.epm_sys_3v3_mv);
                    packet.epm_sys_5v_v = rail_v(message.epm_sys_5v_mv);
                    packet.epm_per_5v_v = rail_v(message.epm_per_5v_mv);
                    packet.epm_per_9v_v = rail_v(message.epm_per_9v_mv);
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
            // Velocity/altitude from the airbrakes estimator whenever its
            // vertical filter is running (2D velocity, tilt included) — gate
            // or not; slow-filter vertical-only state otherwise. Coasting
            // stays on the slow filter's burn timer either way.
            let (state, coasting, ab_state) = estimators.lock(|s| {
                let est = s.borrow();
                let ab = est.airbrakes_estimator();
                let ab_state = match (ab.altitude_asl(), ab.velocity()) {
                    (Some(alt), Some(velocity)) => Some((alt, velocity)),
                    _ => None,
                };
                (
                    est.state(),
                    est.deployment_estimator().is_coasting(),
                    ab_state,
                )
            });
            let message = match state {
                RocketState::Ascent {
                    vertical_velocity,
                    altitude_asl,
                    launch_pad_altitude_asl,
                } => {
                    let (velocity, altitude_agl) = match ab_state {
                        Some((ab_alt, v)) => ([v.x, v.y], ab_alt - launch_pad_altitude_asl),
                        None => (
                            [0.0, vertical_velocity],
                            altitude_asl - launch_pad_altitude_asl,
                        ),
                    };
                    Some(RocketStateMessage::new(
                        unix_clock.now_us_or_boot_time(),
                        &velocity,
                        altitude_agl,
                        coasting,
                    ))
                }
                // The slow KF is frozen (no numbers to report), but the
                // airbrakes filter may already be alive — its 2-of-3 vote
                // exits earlier than the slow filter's padded timer.
                RocketState::MachLockout {
                    launch_pad_altitude_asl,
                } => ab_state.map(|(ab_alt, v)| {
                    RocketStateMessage::new(
                        unix_clock.now_us_or_boot_time(),
                        &[v.x, v.y],
                        ab_alt - launch_pad_altitude_asl,
                        coasting,
                    )
                }),
                _ => None,
            };

            if let Some(message) = message {
                can_sender.send(message.into());
            }

            ticker.next().await;
        }
    };

    let start_airbrakes_signal = Signal::<NoopRawMutex, ()>::new();
    let update_estimators_fut = async {
        let mut airbrakes_started = false;
        let mut terminal_handled = false;
        let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();
        let kf_state_sender = kf_state_watch.sender();
        let airbrakes_state_sender = airbrakes_state_watch.sender();

        loop {
            let reading = imu_baro_sub.next_message_pure().await;
            let baro_data = reading.data.1;
            let baro_altitude_asl = baro_data.altitude_asl();
            // IMU is optional: on the rare sample without it the airbrakes
            // half is skipped and its measured-dt path bridges the gap.
            // The raw IMU reports gyro in deg/s; the estimators want rad/s —
            // convert at this edge.
            let imu = reading.data.0.as_ref().map(|imu| ImuSample {
                acc: imu.acc,
                gyro: imu.gyro * (core::f32::consts::PI / 180.0),
            });

            // One update per ~416 Hz sample; the deployment-speed feed to
            // vote V2 (abstaining during the slow filter's Mach lockout)
            // happens inside.
            let (pyro, state, coasting, mpc_states, kf_state, ab_log) = estimators.lock(|s| {
                let mut est = s.borrow_mut();
                let pyro = est.update(reading.timestamp_us, imu.as_ref(), baro_altitude_asl);

                let dep = est.deployment_estimator();
                // Raw (possibly lockout-frozen) KF numbers — logging only.
                let kf_state = (dep.kf_altitude_asl(), dep.kf_vertical_velocity());
                let coasting = dep.is_coasting();

                let ab = est.airbrakes_estimator();
                let (v1, v2, v3) = ab.lockout_votes().unwrap_or((false, false, false));
                let mut flags = 0u8;
                if v1 {
                    flags |= AB_VOTE_INERTIAL;
                }
                if v2 {
                    flags |= AB_VOTE_DEPLOYMENT;
                }
                if v3 {
                    flags |= AB_VOTE_BARO_RATE;
                }
                if ab.baro_trusted() {
                    flags |= AB_BARO_TRUSTED;
                }
                if ab.is_apogee() {
                    flags |= AB_APOGEE;
                }
                let ab_log = (
                    ab.altitude_asl().unwrap_or(f32::NAN),
                    ab.velocity().map(|v| v.y).unwrap_or(f32::NAN),
                    ab.tilt().map(|t| t.to_degrees()).unwrap_or(f32::NAN),
                    flags,
                );

                (
                    pyro,
                    est.state(),
                    coasting,
                    est.airbrakes_mpc_states(),
                    kf_state,
                    ab_log,
                )
            });
            kf_state_sender.send(kf_state);
            airbrakes_state_sender.send(ab_log);

            if let Some(pyro) = pyro {
                // Channel capacity 2: drogue+main can queue for single-at-apogee.
                #[cfg(feature = "hil-replay")]
                info!("HIL: estimator requested pyro {}", pyro);
                let _ = fire_signal.try_send(pyro);
            }

            // Act-once latch. Permission (coasting, filter alive, ascending,
            // subsonic on the airbrakes filter's own state) lives inside
            // `airbrakes_mpc_states` — Some exactly when brakes may open.
            if !airbrakes_started && mpc_states.is_some() {
                #[cfg(feature = "hil-replay")]
                info!("HIL: starting airbrakes");
                start_airbrakes_signal.signal(());
                airbrakes_started = true;
            }

            if !terminal_handled
                && matches!(
                    state,
                    RocketState::Landed | RocketState::FailedToReachMinApogee
                )
            {
                terminal_handled = true;
                #[cfg(feature = "hil-replay")]
                info!("HIL: estimator reached terminal state {}", state);
                Timer::after_secs(30).await;
                avionics_mode_watch.sender().send(AvionicsMode::Landed);
            }

            // Honest 1:1 mirror of `RocketState`; coasting rides alongside
            // as its own flag, never folded into the stage.
            let new_stage = match state {
                RocketState::OnPad => FlightStage::Armed,
                RocketState::Ascent { .. } => FlightStage::Ascent,
                RocketState::MachLockout { .. } => FlightStage::MachLockout,
                RocketState::DrogueChute { .. } => FlightStage::DrogueChute,
                RocketState::MainChute { .. } => FlightStage::MainChute,
                RocketState::Landed => FlightStage::Landed,
                RocketState::FailedToReachMinApogee => FlightStage::FailedToReachMinApogee,
            };
            #[cfg(feature = "hil-replay")]
            {
                let (prev, _) = flight_stage.lock(|r| *r.borrow());
                if prev != new_stage {
                    info!("HIL: flight_stage {} -> {}", prev, new_stage);
                }
            }
            flight_stage.lock(|r| {
                *r.borrow_mut() = (new_stage, coasting);
            });
        }
    };

    let control_airbrakes_fut = async {
        publish_airbrakes_commanded(air_brakes_watch, 0.0);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
        });

        start_airbrakes_signal.wait().await;
        let launch_pad_altitude_asl =
            estimators.lock(|s| s.borrow().deployment_estimator().launch_pad_altitude_asl());

        let airbrakes_mpc = AirBrakesMPC::new(
            ROCKET_PARAMETERS.clone(),
            launch_pad_altitude_asl
                + target_agl_watch
                    .try_get()
                    .unwrap_or(firmware_common_new::flight_storage::DEFAULT_TARGET_APOGEE_AGL),
        );

        let mut ticker = Ticker::every(Duration::from_hz(10));
        loop {
            // Run condition and MPC state are the same Option: Some exactly
            // while the airbrakes are permitted to be open (coasting, filter
            // alive, ascending, subsonic). "Permitted but no state" cannot
            // be expressed, so there is no fallback chain — the loop ends
            // (and the brakes retract) the moment permission lapses.
            let Some(s) = estimators.lock(|s| s.borrow().airbrakes_mpc_states()) else {
                break;
            };
            let airbrake_extension_percentage = airbrakes_mpc.update(s.altitude_asl, s.velocity);
            publish_airbrakes_commanded(air_brakes_watch, airbrake_extension_percentage);
            can_sender.send(AirBrakesControlMessage::new(airbrake_extension_percentage).into());
            packet_builder.update(|packet| {
                packet.air_brakes_commanded_extension_percentage = airbrake_extension_percentage;
            });

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
        join(update_estimators_fut, control_airbrakes_fut),
    );

    select(fut, wait_armed_mode_end_fut).await;
}
