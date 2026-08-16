use core::{cell::RefCell, future::join};

use air_brakes_controller_core::{
    AirBrakesMPC, FlightEstimators, ImuSample, RocketState, approximate_speed_of_sound,
};
use defmt::{info, warn};
use embassy_futures::select::select;
use embassy_sync::{
    blocking_mutex::{Mutex as BlockingMutex, raw::NoopRawMutex},
    pubsub::WaitResult,
    signal::Signal,
};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        custom_status::payload_sdrm_custom_status::PayloadSDRMCustomStatus,
        messages::{
            CanBusMessageEnum, airbrakes_control::AirBrakesControlMessage,
            amp_control::AmpControlMessage, custom_payload_status::CustomPayloadStatusMessage,
            vl_status::FlightStage,
        },
        node_types::{AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE, PAYLOAD_SDRM_NODE_TYPE},
        sender::CanSender,
    },
    flight_storage::DEFAULT_TARGET_APOGEE_AGL,
    vlp::{client::VLPAvionics, packets::telemetry::TelemetryPacketBuilder},
};

use crate::{
    AvionicsModeWatch, ContinuityWatch,
    FLIGHT_CONFIG, FireSignal, FlightEstimatorsMutex, FlightStageMutex, GPSReadingWatch,
    SetTargetWatch, VLStatusMutex,
    tasks::data_logger::{
        AirBrakesWatch, AmpStateWatch, EstimatorLogWatch, PayloadStateWatch,
        publish_airbrakes_commanded,
    },
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
    tasks::{
        amp_control_task::AmpControlWatch,
        data_logger::{FlightDataChannel, log_flight_data},
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
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
    // Flight-data logging runs here rather than as its own task; these are its
    // inputs, everything above is shared with the rest of armed mode.
    mag_pubsub: &'static MagReadingPubSub,
    amp_state_watch: &'static AmpStateWatch,
    payload_state_watch: &'static PayloadStateWatch,
    vl_status: &'static VLStatusMutex,
    flight_data_channel: &'static FlightDataChannel,
) {
    info!("enter armed mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::Armed;
    });
    amp_control_watch.sender().send(AmpControlMessage {
        out1_enable: true,
        out2_enable: false,
        out3_enable: true,
    });

    let packet_builder = TelemetryPacketBuilder::<NoopRawMutex>::new();
    // One struct owns both estimators and the policy connecting them (the
    // airbrakes-permission gate inside `airbrakes_mpc_states`) — firmware
    // holds it behind one mutex.
    let estimators: FlightEstimatorsMutex = BlockingMutex::new(RefCell::new(
        FlightEstimators::new(FLIGHT_CONFIG.clone()),
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

                // The same config value the MPC targets (operator-set, with
                // the stored default as fallback).
                packet.target_apogee_agl = target_agl_watch
                    .try_get()
                    .unwrap_or(DEFAULT_TARGET_APOGEE_AGL);

                if let Some(amp) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                    packet.amp_online = amp.is_online();
                    packet.amp_uptime_s = amp.status.uptime_s;
                } else {
                    packet.amp_online = false;
                    packet.amp_uptime_s = 0;
                }

                if let Some(icarus) = can_central.get_nodes::<1>(ICARUS_NODE_TYPE).first() {
                    packet.icarus_online = icarus.is_online();
                    packet.icarus_uptime_s = icarus.status.uptime_s;
                } else {
                    packet.icarus_online = false;
                    packet.icarus_uptime_s = 0;
                }

                if let Some(ozys) = can_central.get_nodes::<1>(OZYS_NODE_TYPE).first() {
                    packet.ozys_online = ozys.is_online();
                    packet.ozys_uptime_s = ozys.status.uptime_s;
                } else {
                    packet.ozys_online = false;
                    packet.ozys_uptime_s = 0;
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
                    // All zero once the estimator is retired at apogee, same
                    // as before it is born. The packet has no absence
                    // encoding for these, so `flight_stage` is what tells
                    // the ground which side of the flight a zero is from.
                    let ab = est.airbrakes_estimator();

                    // Airbrakes estimator health: whether the vertical filter
                    // is born. The rest of its state is SD-only.
                    packet.airbrakes_born = ab.is_some_and(|ab| ab.baro_trusted());

                    // Tilt is the airbrakes estimator's gyro dead reckoning
                    // (0 before ignition and after retirement).
                    let airbrakes_kf_tilt_deg = ab
                        .and_then(|ab| ab.tilt())
                        .map(|t| t.to_degrees())
                        .unwrap_or(0.0);

                    // The flight part of the stage mirrors `RocketState` 1:1.
                    match est.state() {
                        RocketState::OnPad => {
                            packet.deployment_kf_altitude_agl = 0.0;
                            packet.deployment_kf_vertical_velocity = 0.0;
                            packet.airbrakes_kf_tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Armed;
                        }
                        RocketState::Ascent {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                        } => {
                            packet.deployment_kf_altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.deployment_kf_vertical_velocity = vertical_velocity;
                            packet.airbrakes_kf_tilt_deg = airbrakes_kf_tilt_deg;
                            packet.flight_stage = FlightStage::Ascent;
                        }
                        RocketState::MachLockout { .. } => {
                            // Folded into `Ascent` on the wire: the rocket is
                            // ascending, and the lockout is an internal detail
                            // of the baro filter. The slow KF is frozen and the
                            // state carries no altitude/velocity, so these
                            // report zeros rather than stale numbers. Tilt is
                            // the only live number on the downlink through this
                            // window; the SD log's `airbrakes_kf_*` columns are
                            // what to read for the rest.
                            packet.deployment_kf_altitude_agl = 0.0;
                            packet.deployment_kf_vertical_velocity = 0.0;
                            packet.airbrakes_kf_tilt_deg = airbrakes_kf_tilt_deg;
                            packet.flight_stage = FlightStage::Ascent;
                        }
                        RocketState::DrogueChute {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                            ..
                        } => {
                            packet.deployment_kf_altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.deployment_kf_vertical_velocity = vertical_velocity;
                            packet.airbrakes_kf_tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::DrogueChute;
                        }
                        RocketState::MainChute {
                            vertical_velocity,
                            altitude_asl,
                            launch_pad_altitude_asl,
                            ..
                        } => {
                            packet.deployment_kf_altitude_agl = altitude_asl - launch_pad_altitude_asl;
                            packet.deployment_kf_vertical_velocity = vertical_velocity;
                            packet.airbrakes_kf_tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::MainChute;
                        }
                        RocketState::Landed => {
                            packet.deployment_kf_altitude_agl = 0.0;
                            packet.deployment_kf_vertical_velocity = 0.0;
                            packet.airbrakes_kf_tilt_deg = 0.0;
                            packet.flight_stage = FlightStage::Landed;
                        }
                        RocketState::FailedToReachMinApogee => {
                            packet.deployment_kf_altitude_agl = 0.0;
                            packet.deployment_kf_vertical_velocity = 0.0;
                            packet.airbrakes_kf_tilt_deg = 0.0;
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
            let message = can_receiver_sub.next_message_pure().await.data.message;
            packet_builder.update(|packet| match message {
                CanBusMessageEnum::AmpStatus(message) => {
                    packet.shared_battery_v = message.shared_battery_mv as f32 / 1000.0;
                    packet.amp_out1_overwrote = message.out1.overwrote;
                    packet.amp_out1 = message.out1.status;
                    packet.amp_out2_overwrote = message.out2.overwrote;
                    packet.amp_out2 = message.out2.status;
                    packet.amp_out3_overwrote = message.out3.overwrote;
                    packet.amp_out3 = message.out3.status;
                }
                CanBusMessageEnum::IcarusStatus(message) => {
                    packet.air_brakes_actual_extension_percentage =
                        message.actual_extension_percentage();
                    packet.air_brakes_servo_temp = message.servo_temperature();
                }
                CanBusMessageEnum::CustomPayloadStatus(message) => {
                    // 0xFFFF means the payload could not read that value; keep it as
                    // None so the ground station shows "n/a" instead of a fake 0.
                    packet.epm_batt_mv =
                        CustomPayloadStatusMessage::reading(message.epm_batt_mv);
                    packet.epm_rail_ma = message.rail_ma().map(CustomPayloadStatusMessage::reading);
                    packet.sem_actuator_steps = message
                        .actuator_steps()
                        .map(CustomPayloadStatusMessage::reading);
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

    // Session-scoped, like `estimators` itself: the per-sample estimator
    // state the SD logger records, published by the loop below.
    let estimator_log_watch = EstimatorLogWatch::new();

    let start_airbrakes_signal = Signal::<NoopRawMutex, ()>::new();
    let update_estimators_fut = async {
        let mut airbrakes_started = false;
        let mut terminal_handled = false;
        let mut lag_warned = false;
        let mut estimator_dropped_samples = 0u64;
        let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();

        loop {
            // Not `next_message_pure`: that silently swallows `Lagged`, and a
            // dropped sample is not a neutral event here. The deployment
            // estimator is SAMPLE-clocked — its burn timer, apogee
            // persistence, landing persistence and pyro delays are all counts
            // of samples, and its KF integrates a fixed `DT` — so every
            // sample this loop misses stretches those timers and shortens the
            // filter's idea of elapsed time, with nothing in the flight log
            // to say it happened. The data is gone either way; refusing to
            // drop it quietly is the point.
            let reading = loop {
                match imu_baro_sub.next_message().await {
                    WaitResult::Message(reading) => break reading,
                    WaitResult::Lagged(dropped) => {
                        estimator_dropped_samples =
                            estimator_dropped_samples.saturating_add(dropped);
                        if !lag_warned {
                            warn!(
                                "estimator loop lagged the sensor stream, {} samples lost — sample-clocked timers now run long",
                                estimator_dropped_samples
                            );
                            lag_warned = true;
                        }
                    }
                }
            };
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

            // One update per ~416 Hz sample; retiring the airbrakes half at
            // apogee happens inside.
            //
            // `log_sample` is taken here, inside the same lock as `update`,
            // because the baro innovation-gate outcomes it carries describe
            // exactly the sample `update` just processed and are overwritten
            // by the next one. It is published with its timestamp so the
            // logger can pair it with the matching fast record and refuse
            // anything else.
            let (pyro, state, mpc_states, log_sample) = estimators.lock(|s| {
                let mut est = s.borrow_mut();
                let pyro = est.update(reading.timestamp_us, imu.as_ref(), baro_altitude_asl);

                (
                    pyro,
                    est.state(),
                    est.airbrakes_mpc_states(),
                    est.log_sample(),
                )
            });
            estimator_log_watch
                .sender()
                .send((reading.timestamp_us, log_sample));

            if let Some(pyro) = pyro {
                // Channel capacity 2: drogue+main can queue for single-at-apogee.
                #[cfg(feature = "hil-replay")]
                info!("HIL: estimator requested pyro {}", pyro);
                let _ = fire_signal.try_send(pyro);
            }

            // Act-once latch. Permission (not retired, filter alive,
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

            // Mirror of `RocketState`, with the Mach lockout folded into
            // `Ascent` (see `FlightStage`) — the only stage that is not 1:1.
            let new_stage = match state {
                RocketState::OnPad => FlightStage::Armed,
                RocketState::Ascent { .. } | RocketState::MachLockout { .. } => FlightStage::Ascent,
                RocketState::DrogueChute { .. } => FlightStage::DrogueChute,
                RocketState::MainChute { .. } => FlightStage::MainChute,
                RocketState::Landed => FlightStage::Landed,
                RocketState::FailedToReachMinApogee => FlightStage::FailedToReachMinApogee,
            };
            #[cfg(feature = "hil-replay")]
            {
                let prev = flight_stage.lock(|r| *r.borrow());
                if prev != new_stage {
                    info!("HIL: flight_stage {} -> {}", prev, new_stage);
                }
            }
            flight_stage.lock(|r| {
                *r.borrow_mut() = new_stage;
            });
        }
    };

    let control_airbrakes_fut = async {
        publish_airbrakes_commanded(air_brakes_watch, 0.0, f32::NAN, false);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
        });

        start_airbrakes_signal.wait().await;
        let launch_pad_altitude_asl =
            estimators.lock(|s| s.borrow().deployment_estimator().launch_pad_altitude_asl());

        // The same airframe the airbrakes estimator inverts for its drag
        // check — one field, so the lockout and the apogee prediction cannot
        // be flying different rockets.
        let airbrakes_mpc = AirBrakesMPC::new(
            FLIGHT_CONFIG.airbrakes.rocket.clone(),
            launch_pad_altitude_asl
                + target_agl_watch
                    .try_get()
                    .unwrap_or(DEFAULT_TARGET_APOGEE_AGL),
        );

        let mut ticker = Ticker::every(Duration::from_hz(10));
        let mut mpc_went_full = false;
        let mut validating = false;
        loop {
            // Run condition and MPC state are the same Option: Some exactly
            // while the airbrakes are permitted to be open (coasting, filter
            // alive, ascending, subsonic). "Permitted but no state" cannot
            // be expressed, so there is no fallback chain — the loop ends
            // (and the brakes retract) the moment permission lapses.
            let Some(s) = estimators.lock(|s| s.borrow().airbrakes_mpc_states()) else {
                break;
            };
            let mpc = airbrakes_mpc.update(s.altitude_asl, s.velocity);
            mpc_went_full |= mpc.extension_percentage >= 0.99;

            // An undershoot barely moves the brakes, leaving the flight with no
            // evidence they work — so once slow enough to be harmless (full
            // extension costs ~0.1 m/s^2 at Mach 0.1), open them fully anyway.
            // Vertical velocity, like the gate's own `MAX_OPEN_MACH`, so it
            // always sweeps through before apogee — total airspeed would not on
            // a tilted flight.
            if !validating
                && !mpc_went_full
                && s.velocity.y < 0.1 * approximate_speed_of_sound(s.altitude_asl)
            {
                info!(
                    "airbrakes: MPC never reached full extension; forcing 100% for validation at {} m/s",
                    s.velocity.y
                );
                validating = true;
            }

            let airbrake_extension_percentage = if validating {
                1.0
            } else {
                mpc.extension_percentage
            };
            // The prediction belongs to the MPC's own command. While the
            // validation deploy overrides it, there is no prediction for what
            // is actually being commanded, so report absence rather than a
            // number about a different extension.
            let predicted_apogee_agl = if validating {
                f32::NAN
            } else {
                mpc.predicted_apogee_asl - launch_pad_altitude_asl
            };
            publish_airbrakes_commanded(
                air_brakes_watch,
                airbrake_extension_percentage,
                predicted_apogee_agl,
                validating,
            );
            can_sender.send(AirBrakesControlMessage::new(airbrake_extension_percentage).into());
            packet_builder.update(|packet| {
                packet.air_brakes_commanded_extension_percentage = airbrake_extension_percentage;
                // The packet has no absence encoding here; 0 while the MPC is
                // not running reads the same as before it started.
                packet.mpc_predicted_apogee_agl = if predicted_apogee_agl.is_nan() {
                    0.0
                } else {
                    predicted_apogee_agl
                };
            });

            ticker.next().await;
        }

        publish_airbrakes_commanded(air_brakes_watch, 0.0, f32::NAN, false);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
            packet.mpc_predicted_apogee_agl = 0.0;
        });
    };

    let wait_armed_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::Armed).await;
    };

    let log_flight_data_fut = log_flight_data(
        &estimator_log_watch,
        imu_baro_pubsub,
        mag_pubsub,
        gps_reading_watch,
        battery_v_watch,
        continuity_watch,
        air_brakes_watch,
        amp_state_watch,
        payload_state_watch,
        can_central,
        unix_clock,
        flight_stage,
        vl_status,
        flight_data_channel,
    );

    // ORDER MATTERS: `update_estimators_fut` must stay ahead of
    // `log_flight_data_fut`. `join!` polls in declaration order, so within the
    // poll round that delivers one sensor sample the estimator updates and
    // publishes before the logger builds that sample's record — which is what
    // puts the per-sample gate outcomes on the right row. The logger checks
    // the timestamp and warns if this is ever violated rather than logging a
    // neighbouring tick's flags.
    let fut = join!(
        update_packet_sensor_fut,
        update_packet_can_fut,
        send_telemetry_packet_fut,
        update_estimators_fut,
        control_airbrakes_fut,
        log_flight_data_fut,
    );

    select(fut, wait_armed_mode_end_fut).await;
}
