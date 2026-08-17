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
            amp_control::AmpControlMessage, vl_status::FlightStage,
        },
        node_types::{AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE, PAYLOAD_SDRM_NODE_TYPE},
        sender::CanSender,
    },
    flight_storage::DEFAULT_TARGET_APOGEE_AGL,
    vlp::{
        client::VLPAvionics,
        packets::telemetry::{DeploymentKfState, IcarusAirBrakesState, TelemetryPacketBuilder},
    },
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

/// The stage that goes on the downlink and into the SD log for a deployment
/// estimator state. Mirrors `RocketState` 1:1 apart from `MachLockout`, which
/// folds into `Ascent` (see `FlightStage`): the rocket is ascending, and the
/// lockout is an internal detail of the baro filter that the ground has no
/// stage to show it in. Whether the filter has numbers during that window is a
/// separate question, answered by `deployment_kf` being absent.
///
/// One function so the two consumers — the telemetry packet and the
/// `FlightStageMutex` the logger reads — cannot drift apart on what a state
/// looks like from outside.
fn wire_flight_stage(state: RocketState) -> FlightStage {
    match state {
        RocketState::OnPad => FlightStage::Armed,
        RocketState::Ascent { .. } | RocketState::MachLockout { .. } => FlightStage::Ascent,
        RocketState::DrogueChute { .. } => FlightStage::DrogueChute,
        RocketState::MainChute { .. } => FlightStage::MainChute,
        RocketState::Landed => FlightStage::Landed,
        RocketState::FailedToReachMinApogee => FlightStage::FailedToReachMinApogee,
    }
}

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
        out1_enable: false,
        out2_enable: true,
        out3_enable: true,
    });

    let packet_builder = TelemetryPacketBuilder::<NoopRawMutex>::new();
    // One struct owns both estimators and the policy connecting them (the
    // airbrakes-permission gate inside `airbrakes_mpc_states`) — firmware
    // holds it behind one mutex.
    let estimators: FlightEstimatorsMutex = BlockingMutex::new(RefCell::new(
        FlightEstimators::new(FLIGHT_CONFIG.clone()),
    ));

    // Session-scoped, like `estimators` itself: the per-sample estimator
    // state, published by the estimator loop below and read by BOTH the SD
    // logger and the 5 Hz packet loop. Hoisted up here because the packet
    // loop now reads it instead of taking the estimators mutex.
    let estimator_log_watch = EstimatorLogWatch::new();

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

                {
                    // Read the SAME published sample the SD logger records,
                    // rather than taking the estimators mutex at 5 Hz. The
                    // downlink and the log can therefore not disagree about a
                    // sample: there is only one, produced once per IMU sample
                    // by the estimator loop below.
                    //
                    // `None` before that loop's first sample, which is the
                    // same thing reading a freshly-built `FlightEstimators`
                    // used to produce: nothing born, nothing calibrated, no
                    // tilt, no deployment filter.
                    let sample = estimator_log_watch.try_get().map(|(_, s)| s);
                    let ab = sample.as_ref().and_then(|s| s.airbrakes.as_ref());

                    // Airbrakes estimator health: whether the vertical filter
                    // is born. The rest of its state is SD-only.
                    packet.airbrakes_born = ab.is_some_and(|ab| ab.baro_trusted);

                    // ...and whether the pad calibration exists. This is the
                    // only airbrakes bit in the downlink that can be acted on
                    // while there is still time to act: `airbrakes_born` is
                    // `baro_trusted`, which cannot go true until the Mach
                    // lockout has already been and gone. Ignition detection is
                    // gated on the calibration, so a rocket on the rail
                    // reporting false here will fly with no airbrakes and
                    // report nothing unusual while doing it.
                    //
                    // `false` once the half is retired at apogee, where the
                    // question has stopped meaning anything — the same
                    // convention `airbrakes_born` follows.
                    packet.airbrakes_calibrated = ab.is_some_and(|ab| ab.calibration_complete);

                    // Tilt is the airbrakes estimator's gyro dead reckoning:
                    // absent before ignition and again once the estimator is
                    // retired at apogee. Passed through as the `Option` it
                    // already is — a zero here would read as "pointing
                    // straight up", which is a claim, not silence.
                    packet.airbrakes_kf_tilt_deg = ab.and_then(|ab| ab.tilt_rad).map(|t| t.to_degrees());

                    // The downlink and the SD log read the deployment KF
                    // through the SAME two accessors, so the two channels
                    // cannot disagree about whether a sample exists — they
                    // agree by construction rather than by two pieces of code
                    // happening to make the same call.
                    //
                    // Both are absent for the whole first half of the flight,
                    // by design and not by fault. There is no filter on the
                    // pad at all — the barometer's only job there is the pad
                    // altitude reference — and none through the Mach lockout
                    // either: it is DROPPED at ignition detection rather than
                    // frozen, so nothing can read a pre-ignition altitude out
                    // of it while the rocket is kilometres away. One is built
                    // in flight, from the first honest reading, when the 26 s
                    // lockout ends (`hil-single` configures no lockout, so
                    // there it is built at ignition detection instead). So
                    // `deployment_kf` is `None` for the entire pad period AND
                    // the 26 s that follow ignition, and `Some` from there to
                    // landing.
                    //
                    // "n/a" on the ground display while the rocket is on the
                    // rail is therefore correct, not a dead filter, and it is
                    // not a pre-launch health check: `airbrakes_calibrated`
                    // above is the field that says something with time left to
                    // act on it. `None` here and an empty column in the log
                    // describe the same window.
                    packet.deployment_kf = sample.as_ref().and_then(|s| {
                        match (s.deployment_altitude_asl, s.deployment_vertical_velocity) {
                            (Some(altitude_asl), Some(vertical_velocity)) => {
                                Some(DeploymentKfState {
                                    altitude_agl: altitude_asl
                                        - s.deployment_launch_pad_altitude_asl,
                                    vertical_velocity,
                                })
                            }
                            _ => None,
                        }
                    });

                    // Written by the 416 Hz estimator loop below rather than
                    // recomputed here, so the packet, the SD log and the
                    // logger's own stage all come from one call to
                    // `wire_flight_stage` instead of three.
                    packet.flight_stage = flight_stage.lock(|r| *r.borrow());
                }
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
                    // Both numbers arrive in this one message, so they become
                    // present together and stay absent together until it does.
                    packet.icarus_air_brakes = Some(IcarusAirBrakesState {
                        actual_extension_percentage: message.actual_extension_percentage(),
                        servo_temp: message.servo_temperature(),
                    });
                }
                CanBusMessageEnum::CustomPayloadStatus(message) => {
                    // The accessors decode the payload's 0xFFFF "could not read
                    // this" sentinel — which exists only because the CAN frame
                    // cannot carry an `Option` — back into `None`, so the ground
                    // station shows "n/a" instead of a fake 0.
                    packet.epm_batt_mv = message.epm_batt_mv();
                    packet.epm_rail_ma = message.rail_ma();
                    packet.sem_actuator_steps = message.actuator_steps();
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
            // estimator's KF integrates a fixed `DT` per sample — deliberately,
            // so no clock can surprise the filter that fires the pyros — so
            // every sample this loop misses shortens that filter's idea of
            // elapsed time, with nothing in the flight log to say it happened.
            // The data is gone either way; refusing to drop it quietly is the
            // point.
            //
            // What a drop no longer does is stretch the timers: the Mach
            // lockout, the pyro delays and the apogee/landing persistence all
            // read the sample timestamp now, so they stay in honest seconds
            // across a gap.
            let reading = loop {
                match imu_baro_sub.next_message().await {
                    WaitResult::Message(reading) => break reading,
                    WaitResult::Lagged(dropped) => {
                        estimator_dropped_samples =
                            estimator_dropped_samples.saturating_add(dropped);
                        if !lag_warned {
                            warn!(
                                "estimator loop lagged the sensor stream, {} samples lost — the deployment KF has missed that many fixed-DT steps",
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
            // The log sample comes back OUT of `update` rather than being
            // read off the estimators afterwards: the baro innovation-gate
            // outcomes it carries describe exactly the sample `update` just
            // processed and are stored nowhere, so there is no way left to
            // read them late or attribute them to the wrong tick. It is
            // published with its timestamp so the logger can pair it with the
            // matching fast record and refuse anything else.
            let (pyro, state, mpc_states, log_sample) = estimators.lock(|s| {
                let mut est = s.borrow_mut();
                let (pyro, log_sample) =
                    est.update(reading.timestamp_us, imu.as_ref(), baro_altitude_asl);

                (pyro, est.state(), est.airbrakes_mpc_states(), log_sample)
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

            let new_stage = wire_flight_stage(state);
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
        // A real command of fully-retracted, not an absence: the CAN message
        // below carries it to Icarus. The prediction is absent because the MPC
        // has not run yet.
        publish_airbrakes_commanded(air_brakes_watch, Some(0.0), None, false);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
        });

        start_airbrakes_signal.wait().await;
        let launch_pad_altitude_asl =
            estimators.lock(|s| s.borrow().launch_pad_altitude_asl());

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
            // Two questions, one lock, one sample of the estimator, because
            // they are NOT the same question and must not be answered a tick
            // apart:
            //
            // * is the airbrakes half RETIRED — dropped for good by
            //   `FlightEstimators::update` at apogee, below the horizon, or
            //   on the deployment half's apogee call. Destructive and final:
            //   there is no state left to reopen from. This ends the loop.
            // * is the gate OPEN RIGHT NOW — Some exactly while the brakes
            //   are permitted to be open (filter alive, ascending, subsonic).
            //   This shuts for reasons that pass, so it ends a tick, not the
            //   flight.
            //
            // Ending on the gate alone was one unlucky tick away from ending
            // airbrakes control for a whole healthy flight. On the nominal
            // Osiris O3400 trajectory the filter is born at 18.892 s and its
            // vertical velocity settles at 251.8 m/s against a 0.8 Mach
            // ceiling of 250.2 m/s — 0.6% over — so the ceiling clause holds
            // the gate shut from 18.919 s to 19.079 s, 0.160 s. A 10 Hz tick
            // lands inside that window: the loop used to get 1 controlled
            // tick and then command 0% for the remaining 20 s of coast. Ending
            // on retirement instead gets all 206 of them, and still ends for
            // good at 39.59 s, on the same sample the estimator retires.
            let (mpc_states, retired) = estimators.lock(|s| {
                let est = s.borrow();
                (
                    est.airbrakes_mpc_states(),
                    est.airbrakes_estimator().is_none(),
                )
            });
            if retired {
                break;
            }

            // A shut gate is a retracted tick — a real 0.0 on the wire, and no
            // prediction, because the MPC did not run and so predicted
            // nothing. Both latches below live outside the loop on purpose and
            // are left alone here: they are statements about the flight so
            // far, not about this tick, and a gap in the middle of the coast
            // does not un-say them.
            let (airbrake_extension_percentage, predicted_apogee_agl) = match mpc_states {
                None => (0.0, None),
                Some(s) => {
                    let mpc = airbrakes_mpc.update(s.altitude_asl, s.velocity);
                    mpc_went_full |= mpc.extension_percentage >= 0.99;

                    // An undershoot barely moves the brakes, leaving the flight
                    // with no evidence they work — so once slow enough to be
                    // harmless (full extension costs ~0.1 m/s^2 at Mach 0.1),
                    // open them fully anyway. Vertical velocity, like the
                    // gate's own `max_open_mach`, so it always sweeps through
                    // before apogee — total airspeed would not on a tilted
                    // flight.
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
                    // The prediction belongs to the MPC's own command. While
                    // the validation deploy overrides it, there is no
                    // prediction for what is actually being commanded, so
                    // report absence rather than a number about a different
                    // extension. A solve that comes back NaN is absent for the
                    // same reason — it is not a prediction either, and the
                    // packet's fixed-point encoding has nothing to make of it.
                    let predicted_apogee_agl = if validating {
                        None
                    } else {
                        let predicted_apogee_agl =
                            mpc.predicted_apogee_asl - launch_pad_altitude_asl;
                        (!predicted_apogee_agl.is_nan()).then_some(predicted_apogee_agl)
                    };
                    (airbrake_extension_percentage, predicted_apogee_agl)
                }
            };
            publish_airbrakes_commanded(
                air_brakes_watch,
                Some(airbrake_extension_percentage),
                predicted_apogee_agl,
                validating,
            );
            can_sender.send(AirBrakesControlMessage::new(airbrake_extension_percentage).into());
            packet_builder.update(|packet| {
                packet.air_brakes_commanded_extension_percentage = airbrake_extension_percentage;
                // The same `Option` the SD log gets: the packet has its own
                // validity bit for this now, so "the MPC is not running" is
                // distinguishable on the downlink from "it predicts 0 m".
                packet.mpc_predicted_apogee_agl = predicted_apogee_agl;
            });

            ticker.next().await;
        }

        // The airbrakes half is retired — the flight is past apogee and there
        // is no state left to reopen from — so the brakes are commanded shut
        // (a real 0.0, not an absence) and the MPC has stopped for good, so
        // there is no longer a prediction to go with it.
        publish_airbrakes_commanded(air_brakes_watch, Some(0.0), None, false);
        can_sender.send(AirBrakesControlMessage::new(0.0).into());
        packet_builder.update(|packet| {
            packet.air_brakes_commanded_extension_percentage = 0.0;
            packet.mpc_predicted_apogee_agl = None;
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
        target_agl_watch,
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
