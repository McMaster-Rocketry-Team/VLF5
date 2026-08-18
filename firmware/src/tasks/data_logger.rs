use crate::{
    ContinuityWatch, FlightStageMutex, GPSReadingWatch, VLStatusMutex,
    can_central::CanCentral,
    tasks::{
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub},
        unix_clock::UnixClock,
    },
    utils::drain_latest,
};

use air_brakes_controller_core::EstimatorLogSample;
use defmt::warn;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex, channel::Channel, pubsub::WaitResult, watch::Watch,
};
use embassy_time::{Duration, Instant};
use firmware_common_new::{
    can_bus::node_types::{
        AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE, PAYLOAD_SDRM_NODE_TYPE,
    },
    flight_data_record::{
        AIRBRAKES_BURNOUT, AIRBRAKES_PAD_CALIBRATED, AirBrakesActuationRecord,
        AirBrakesRecord, AirbrakesEstimatorRecord, AmpRecord,
        DEPLOYMENT_BARO_GATE_REJECT, DEPLOYMENT_BARO_RESYNC, DeploymentEstimatorRecord,
        FlightDataFastRecord,
        FlightDataSlowRecord, ImuRecord, LogRecord, NodeStatusRecord, PayloadRecord,
    },
};

/// The last heartbeat from the single node of `node_type`, or `None` when the
/// node has never been heard from.
///
/// The two absences are different and the record keeps them apart: `None` is a
/// node that has never spoken at all, while a record with `online: false` is
/// one that spoke and then went quiet — `online` is a 5 s timeout, so that
/// record still carries its last-known health, which is what the flag is for.
fn node_status_record(
    can_central: &CanCentral<NoopRawMutex>,
    node_type: u8,
) -> Option<NodeStatusRecord> {
    can_central
        .get_nodes::<1>(node_type)
        .first()
        .map(|node| NodeStatusRecord::from_message(node.is_online(), &node.status))
}

/// Queue between the IMU-clocked logger and the SD writer.
pub const FLIGHT_DATA_CHANNEL_DEPTH: usize = 512;
pub type FlightDataChannel = Channel<NoopRawMutex, LogRecord, FLIGHT_DATA_CHANNEL_DEPTH>;

/// Latest airbrakes state for SD logging: commanded extension, the apogee the
/// MPC predicts at it and the one it is aiming for, and Icarus's reported
/// extension and servo temperature.
///
/// Every field is `None` until its source has spoken — the firmware for
/// `commanded_extension`, `predicted_apogee_asl` and `target_apogee_asl`, an
/// `IcarusStatus` CAN message for the other two.
/// That is the whole "is this present?" story: there are no validity flags,
/// because the `Option` says it and a 0.0 would not. An Icarus that is offline
/// or silent must not read as one reporting fully-stowed brakes at 0 C.
///
/// The default is therefore all-absent, which is exactly what an empty watch
/// means as well — nothing commanded and nothing heard — so the two are
/// interchangeable at the point of use.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct AirBrakesLogState {
    pub commanded_extension: Option<f32>,
    /// Apogee ASL the MPC predicts at `commanded_extension`. `None` whenever
    /// the MPC is not running, which includes the forced validation deploy —
    /// there the commanded extension is not the MPC's output, so pairing it
    /// with a prediction would be a lie.
    ///
    /// ASL rather than the AGL the downlink carries: the log stores altitudes
    /// in the unit they are measured in and keeps the pad reference beside
    /// them, so the subtraction happens once, on the ground, instead of being
    /// baked in irreversibly here.
    pub predicted_apogee_asl: Option<f32>,
    /// Apogee ASL the MPC is aiming at, as it latched the target at
    /// construction. `None` until the MPC exists.
    ///
    /// Published once, by the control loop that builds the MPC, rather than
    /// sampled per record off the operator's target watch: the MPC takes its
    /// target once and a `SetTargetApogee` accepted later moves the watch but
    /// not the controller. This field is about the controller.
    pub target_apogee_asl: Option<f32>,
    /// `commanded_extension` is the forced validation deploy rather than the
    /// MPC's output. The one thing about the command that an absent prediction
    /// cannot say, so it gets its own flag: 1.0 from the MPC and 1.0 from the
    /// validation deploy are otherwise the same number.
    pub validation_deploy: bool,
    pub actual_extension: Option<f32>,
    pub servo_temp: Option<f32>,
}

pub type AirBrakesWatch = Watch<NoopRawMutex, AirBrakesLogState, 2>;

/// Publish a commanded extension and the MPC's prediction for it, leaving the
/// Icarus-sourced fields and the target alone. Pass `None` for the prediction
/// when the command did not come from the MPC, and set `validation_deploy`
/// when the reason is the forced validation deploy.
pub fn publish_airbrakes_commanded(
    watch: &AirBrakesWatch,
    extension: Option<f32>,
    predicted_apogee_asl: Option<f32>,
    validation_deploy: bool,
) {
    let mut state = watch.try_get().unwrap_or_default();
    state.commanded_extension = extension;
    state.predicted_apogee_asl = predicted_apogee_asl;
    state.validation_deploy = validation_deploy;
    let _ = watch.sender().send(state);
}

/// Publish the apogee ASL the MPC has just latched as its target, leaving every
/// other field alone. Called once, when the MPC is constructed — the target
/// does not change after that, which is exactly why the log takes it from here
/// rather than resampling the operator's watch per record.
pub fn publish_airbrakes_target(watch: &AirBrakesWatch, target_apogee_asl: f32) {
    let mut state = watch.try_get().unwrap_or_default();
    state.target_apogee_asl = Some(target_apogee_asl);
    let _ = watch.sender().send(state);
}

/// Latest AMP status heartbeat for SD logging: shared battery rail voltage
/// and the three output statuses packed 2 bits per output
/// (`PowerOutputStatus` discriminants, out1 in the LSBs) — the same encoding
/// the slow record stores.
///
/// Deliberately not `Default`: both fields are always present in an
/// `AmpStatusMessage`, so there is no such thing as a partly-absent one, and a
/// zeroed stand-in for "the AMP has never spoken" would be a lie that decodes
/// cleanly — `out_status` 0 is three real `PowerOutputStatus` values. An AMP
/// that has said nothing is an absent watch, and the slow record's `amp` is
/// `None` there.
#[derive(Clone, Copy, defmt::Format)]
pub struct AmpLogState {
    pub shared_battery_v: f32,
    pub out_status: u8,
}

pub type AmpStateWatch = Watch<NoopRawMutex, AmpLogState, 2>;

/// Latest payload `CustomPayloadStatusMessage` for SD logging, kept in the
/// units it arrives in. The CAN message marks a reading the payload could not
/// take with a 0xFFFF sentinel — the bus cannot carry an `Option` — and its
/// accessors decode that back to `None`, so the sentinel never gets past the
/// receive handler and into firmware state.
///
/// Each reading is separately optional, which is what keeps a live EPM with one
/// dead rail sensor distinguishable from a payload that has said nothing at all
/// (the default: every field `None`).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct PayloadLogState {
    pub epm_batt_mv: Option<u16>,
    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub rail_ma: [Option<u16>; 6],
    /// Experiment channels 1..3.
    pub actuator_steps: [Option<u16>; 3],
    /// Fracture load per experiment channel (centinewtons, tension positive),
    /// channels 1..3. Signed, so it carries its own absence code rather than
    /// the 0xFFFF the readings above use — see `PAYLOAD_LOAD_CELL_UNAVAILABLE`.
    pub load_cell_cn: [Option<i16>; 3],
    /// Per-channel experiment state, as the packed word the payload sent.
    /// Not optional: every bit pattern is a legal state, and the default
    /// (every flag clear) is also what an unfitted channel reports.
    pub experiment_flags: u32,
}

pub type PayloadStateWatch = Watch<NoopRawMutex, PayloadLogState, 2>;

/// The estimator loop's per-sample publication to the flight-data logger:
/// `(timestamp_us, sample)`, both taken inside the same critical section as
/// the estimator update that produced them.
///
/// This exists so the log carries per-sample events — the baro innovation
/// gates — that a logger sampling the live estimators on its own clock would
/// misattribute or miss entirely. The timestamp travels with the sample so
/// the logger can refuse to pair it with a different tick.
///
/// Session-scoped: armed mode owns the `Watch` alongside the estimators
/// themselves, so a stale sample cannot outlive the session that produced it.
pub type EstimatorLogWatch = Watch<NoopRawMutex, (u64, EstimatorLogSample), 2>;

const SLOW_INTERVAL: Duration = Duration::from_millis(100);

/// The estimator half of one fast record, packed from the [`EstimatorLogSample`]
/// the estimator loop took in the same critical section as its `update` call.
///
/// The packing lives here rather than in the core crate because the `AIRBRAKES_*`
/// / `DEPLOYMENT_*` bit layout is a storage-format detail, not estimator logic.
///
/// The estimators' own `Option`s are passed through untouched: a value they
/// have not produced yet stays absent in the record, and the whole airbrakes
/// group goes absent once it is retired at apogee — absent reads the same as
/// not-yet-born, with `flight_stage` dating the transition.
///
/// The deployment half always exists (that estimator is never retired), so it
/// is returned unwrapped; only its two numbers can be absent, and only for the
/// reasons `RocketStateEstimator::kf_altitude_asl` documents.
fn pack_estimator_sample(
    sample: &EstimatorLogSample,
) -> (DeploymentEstimatorRecord, Option<AirbrakesEstimatorRecord>) {
    let mut deployment_flags = 0u8;
    if sample.deployment_baro_gate.rejected() {
        deployment_flags |= DEPLOYMENT_BARO_GATE_REJECT;
    }
    if sample.deployment_baro_gate.resynced() {
        deployment_flags |= DEPLOYMENT_BARO_RESYNC;
    }
    let deployment = DeploymentEstimatorRecord {
        kf_altitude_asl: sample.deployment_altitude_asl,
        kf_vertical_velocity: sample.deployment_vertical_velocity,
        flags: deployment_flags,
    };

    let Some(ab) = sample.airbrakes.as_ref() else {
        return (deployment, None);
    };

    let mut flags = 0u8;
    if ab.burnout_detected {
        flags |= AIRBRAKES_BURNOUT;
    }
    // Two bits of the same byte, so "where was the estimator" costs nothing
    // over the booleans it replaced.
    flags |= ab.state.to_flags();
    // The only one of these that says anything before ignition, which is
    // exactly when it matters: the estimator will not detect ignition
    // without it, so a pad segment logged with this clear is the whole
    // explanation for airbrakes that never deployed.
    if ab.calibration_complete {
        flags |= AIRBRAKES_PAD_CALIBRATED;
    }

    (
        deployment,
        Some(AirbrakesEstimatorRecord {
            kf_altitude_asl: ab.altitude_asl,
            kf_vertical_velocity: ab.vertical_velocity,
            kf_tilt_deg: ab.tilt_rad.map(|t| t.to_degrees()),
            flags,
        }),
    )
}

/// Flight-data logging, run as one of armed mode's joined futures rather than a
/// standalone task — logging exists exactly as long as an armed session does.
///
/// That is also the logging policy, now expressed by construction instead of by
/// a mode predicate: `Armed` covers the whole ascent/coast/deploy/descent until
/// the auto-switch to `Landed`, and no other mode logs, so pre-flight checks and
/// time on the ground after landing don't fill the SD card. To log in every mode
/// for bench work, spawn this as its own task from `low_prio_main` instead —
/// every input below is already `&'static` except `estimator_log_watch`, which
/// only exists while armed.
///
/// That containment is what rules out cross-session leakage *through the pubsub
/// inputs*. Every subscriber and counter below is created on entry, and an
/// embassy pubsub subscriber only receives messages published after its
/// creation, so a re-arm (Armed -> LowPower -> Armed) cannot carry pre-arm
/// samples into the new session's first records — no low-power baro backlog, and
/// no mag readings left over from the *previous* armed session (`mag_task` is
/// powered down outside Armed, so its queue would otherwise be minutes stale).
/// The estimator numbers arrive on `estimator_log_watch`, which armed mode
/// rebuilds per session alongside the estimators themselves, and each carries
/// the timestamp it was taken at — so neither a previous session's sample nor a
/// neighbouring tick's can be logged against this record.
///
/// The argument stops there, and deliberately: it is about subscribers, and the
/// `&'static` watches are not subscribers. A watch holds one value that outlives
/// the session, so `try_get` on entry returns whatever the last writer left —
/// which for a source that is unpowered between sessions is the previous
/// session's reading, with nothing about it to say so. Keeping those honest is
/// the caller's job, and `armed_mode` does it explicitly at entry: it clears
/// `payload_state_watch` and the Icarus-sourced fields of `air_brakes_watch`,
/// both fed from nodes on AMP out 2 and so genuinely dark between sessions.
/// `gps_reading_watch` and `amp_state_watch` are left alone on purpose — those
/// sources run in every mode, so their last value is current, not stale.
///
/// Cancellation-safe: records reach the SD writer through a non-blocking
/// `try_send`, so being dropped at a mode change cannot tear one.
pub async fn log_flight_data(
    estimator_log_watch: &EstimatorLogWatch,
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    mag_pubsub: &'static MagReadingPubSub,
    gps_watch: &'static GPSReadingWatch,
    battery_watch: &'static BatteryVWatch,
    continuity_watch: &'static ContinuityWatch,
    air_brakes_watch: &'static AirBrakesWatch,
    amp_state_watch: &'static AmpStateWatch,
    payload_state_watch: &'static PayloadStateWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    unix_clock: &'static UnixClock,
    flight_stage: &'static FlightStageMutex,
    vl_status: &'static VLStatusMutex,
    channel: &'static FlightDataChannel,
) {
    let mut imu_baro_sub = imu_baro_pubsub.subscriber().unwrap();
    let mut mag_sub = mag_pubsub.subscriber().unwrap();

    let mut sequence: u32 = 0;
    let mut backlog_warned = false;
    let mut estimator_skew_warned = false;
    let mut lag_warned = false;
    let mut last_slow_emit = Instant::now();

    loop {
        // Blocks on the sensor stream with no timeout: a record exists if and
        // only if a sample did. The sensor task tolerates transient bus errors
        // by skipping a tick (~2.4 ms), and anything longer than that is a
        // dead sensor, not something the log can paper over.
        //
        // Not `next_message_pure`: it swallows `Lagged`, and a sample this
        // loop never saw would otherwise leave no trace at all — the record
        // stream would read as continuous across the gap. Charging the lost
        // samples to `sequence` is what makes them visible post-flight, and
        // is exactly what that field is documented to mean.
        let (timestamp_us, imu_opt, baro_data) = loop {
            match imu_baro_sub.next_message().await {
                WaitResult::Message(reading) => {
                    let (imu, baro) = reading.data;
                    break (reading.timestamp_us, imu, baro);
                }
                WaitResult::Lagged(dropped) => {
                    sequence = sequence.wrapping_add(dropped as u32);
                    if !lag_warned {
                        warn!("data_logger: lagged the sensor stream, {} samples not logged", dropped);
                        lag_warned = true;
                    }
                }
            }
        };

        let mag_opt = drain_latest(&mut mag_sub);
        // Read straight off the watches: this loop only ever samples their
        // latest value and never awaits a change, so it needs no receiver slot
        // (they are a scarce, `unwrap`-ed resource — `GPSReadingWatch` in
        // particular sits at its capacity while armed).
        let gps_opt = gps_watch.try_get();
        let battery_opt = battery_watch.try_get();
        let continuity_opt = continuity_watch.try_get();
        let airbrakes_opt = air_brakes_watch.try_get();
        let amp_opt = amp_state_watch.try_get();
        // An empty watch and the all-absent default say the same thing — the
        // payload has reported nothing — so the default stands in for a watch
        // the payload has not written yet.
        let payload = payload_state_watch.try_get().unwrap_or_default();
        let stage = flight_stage.lock(|r| *r.borrow());
        // The estimator loop publishes this from inside the same critical
        // section as its `update`, and is polled before this future in armed
        // mode's `join!`, so `stamped.0` is this very sample's timestamp. The
        // check below is what makes that a guarantee rather than an ordering
        // assumption — a mismatch means the two loops have drifted apart and
        // the gate bits would be attributed to the wrong row.
        //
        // The pad altitude rides out of the same match for the same reason:
        // it is a field of the estimator sample, so taking it here means the
        // slow record's reference and the fast record's ASL altitudes came
        // from one estimator tick and cannot describe different pads.
        let (deployment, airbrakes, launch_pad_altitude_asl) = match estimator_log_watch.try_get()
        {
            Some((sample_timestamp_us, sample)) if sample_timestamp_us == timestamp_us => {
                let (deployment, airbrakes) = pack_estimator_sample(&sample);
                (
                    Some(deployment),
                    airbrakes,
                    Some(sample.deployment_launch_pad_altitude_asl),
                )
            }
            // Before the estimator's first sample there is no matching
            // estimator tick — log absence rather than a neighbouring tick's
            // numbers. Both groups go absent together: the gate bits belong to
            // the sample that produced them, so half a group would be a claim
            // about this tick that nothing measured.
            other => {
                if other.is_some() && imu_opt.is_some() && !estimator_skew_warned {
                    warn!("data_logger: estimator sample is not for this tick");
                    estimator_skew_warned = true;
                }
                (None, None, None)
            }
        };

        let imu = imu_opt.map(|imu| ImuRecord {
            acc: [imu.acc.x, imu.acc.y, imu.acc.z],
            gyro: [imu.gyro.x, imu.gyro.y, imu.gyro.z],
        });
        // Not optional: every published sample carries a baro reading, so
        // pressure (fast) and temperature (slow) are always present.
        let (temperature, pressure) = (baro_data.temperature, baro_data.pressure);
        let mag = mag_opt.map(|r| {
            let m = r.data.mag;
            [m.x, m.y, m.z]
        });

        let battery_voltage = battery_opt.map(|r| r.data);
        // The GPS reading is already all-`Option` where it can be absent, so it
        // passes straight through; an empty watch is the same absence one field
        // at a time. The satellite count is not optional — 0 satellites is a
        // real reading, "no fix", and that is also the right answer for a GPS
        // that has not reported.
        let (lat_lon, gps_altitude_asl, num_sats, hdop, vdop, pdop) = match gps_opt {
            Some(r) => {
                let g = r.data;
                (
                    g.lat_lon,
                    g.gps_altitude_asl,
                    g.num_of_fix_satellites,
                    g.hdop,
                    g.vdop,
                    g.pdop,
                )
            }
            None => (None, None, 0, None, None, None),
        };
        let pyro_flags = continuity_opt.map(|c| {
            (c.pyro_main_continuity as u8)
                | ((c.pyro_main_fire as u8) << 1)
                | ((c.pyro_drogue_continuity as u8) << 2)
                | ((c.pyro_drogue_fire as u8) << 3)
                | ((c.short_circuit as u8) << 4)
        });
        // Every field of `AirBrakesLogState` carries its own absence, and the
        // default is all-absent, so a watch nobody has written to logs exactly
        // like a state where nothing has been commanded and Icarus has not
        // reported — which is the truth in both cases.
        let airbrakes_state = airbrakes_opt.unwrap_or_default();
        // Sampled per fast record, which is the whole point of it being here:
        // the watch is written by the 10 Hz control loop and by every
        // `IcarusStatus` frame, and reading it at 427 Hz is what pins each of
        // those edges to a row rather than to a 100 ms bucket.
        let air_brakes_actuation = AirBrakesActuationRecord {
            commanded_extension: airbrakes_state.commanded_extension,
            actual_extension: airbrakes_state.actual_extension,
            validation_deploy: airbrakes_state.validation_deploy,
        };
        let air_brakes = AirBrakesRecord {
            predicted_apogee_asl: airbrakes_state.predicted_apogee_asl,
            servo_temp: airbrakes_state.servo_temp,
            // The MPC's own latched target, published by the control loop when
            // it built the MPC — not a per-record sample of the operator's
            // target watch. A `SetTargetApogee` accepted mid-flight moves that
            // watch and the SD config block, but the controller keeps chasing
            // the number it started with, and this column is about the
            // controller.
            target_apogee_asl: airbrakes_state.target_apogee_asl,
        };
        // No default to fall back on here: `out_status` packs `PowerOutputStatus`
        // discriminants two bits at a time and 0 is one of them, so an AMP that
        // has never spoken would otherwise decode as a genuine set of output
        // statuses. The whole record is absent instead.
        let amp = amp_opt.map(|a| AmpRecord {
            shared_battery_v: a.shared_battery_v,
            out_status: a.out_status,
        });
        // The whole heartbeat, not just liveness: the downlink can only afford
        // two bits per node, so the log is the only place a mid-flight reboot
        // or a health change is recoverable.
        let amp_node = node_status_record(can_central, AMP_NODE_TYPE);
        let icarus_node = node_status_record(can_central, ICARUS_NODE_TYPE);
        let ozys_node = node_status_record(can_central, OZYS_NODE_TYPE);
        let payload_sdrm_node = node_status_record(can_central, PAYLOAD_SDRM_NODE_TYPE);

        let sd_ok = vl_status.lock(|s| s.borrow().sd_ok);
        if !sd_ok {
            // The sample happened and simply cannot be stored. Consume its
            // sequence number anyway, so an SD outage shows up as a gap
            // rather than as a log that reads continuous straight through it.
            sequence = sequence.wrapping_add(1);
            continue;
        }

        let fast_record = LogRecord::Fast(FlightDataFastRecord {
            sequence,
            timestamp_us,
            // Absent until the GPS PPS has disciplined the unix clock.
            unix_time_us: unix_clock.convert_to_unix_us(timestamp_us),
            imu,
            pressure,
            mag,
            deployment,
            airbrakes,
            pyro_flags,
            flight_stage: stage,
            air_brakes: air_brakes_actuation,
        });
        sequence = sequence.wrapping_add(1);

        let slow_record = FlightDataSlowRecord {
            timestamp_us,
            temperature,
            battery_voltage,
            lat_lon,
            gps_altitude_asl,
            num_of_fix_satellites: num_sats,
            hdop,
            vdop,
            pdop,
            launch_pad_altitude_asl,
            air_brakes,
            amp,
            payload: PayloadRecord {
                epm_batt_mv: payload.epm_batt_mv,
                rail_ma: payload.rail_ma,
                actuator_steps: payload.actuator_steps,
                load_cell_cn: payload.load_cell_cn,
                experiment_flags: payload.experiment_flags,
            },
            amp_node,
            icarus_node,
            ozys_node,
            payload_sdrm_node,
        };

        let emit_slow = last_slow_emit.elapsed() >= SLOW_INTERVAL;

        let mut send_failed = false;
        if channel.try_send(fast_record).is_err() {
            send_failed = true;
        } else if emit_slow {
            if channel.try_send(LogRecord::Slow(slow_record)).is_err() {
                send_failed = true;
            } else {
                last_slow_emit = Instant::now();
            }
        }

        if send_failed {
            if !backlog_warned {
                warn!("data_logger: SD path busy, dropping flight record");
                backlog_warned = true;
            }
        } else if backlog_warned {
            backlog_warned = false;
        }
    }
}
