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
    can_bus::{
        messages::custom_payload_status::PAYLOAD_READING_UNAVAILABLE,
        node_types::{
            AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE, PAYLOAD_SDRM_NODE_TYPE,
        },
    },
    flight_data_record::{
        AIRBRAKES_APOGEE, AIRBRAKES_BARO_GATE_REJECT,
        AIRBRAKES_BARO_RESYNC, AIRBRAKES_BARO_TRUSTED, AIRBRAKES_BURNOUT,
        AIRBRAKES_SUBSONIC_DRAG, DEPLOYMENT_BARO_GATE_REJECT, DEPLOYMENT_BARO_RESYNC,
        FlightDataFastRecord,
        FlightDataSlowRecord, LogRecord, NodeStatusRecord, VALID_BATTERY, VALID_GPS_ALT,
        VALID_GPS_FIX, VALID_IMU, VALID_MAG,
    },
};

/// The last heartbeat from the single node of `node_type`, or `offline()` when
/// the node has never been heard from.
///
/// `online` is a 5 s timeout, so a node that has just gone quiet still reports
/// its last-known health here — that is what the flag is for.
fn node_status_record(
    can_central: &CanCentral<NoopRawMutex>,
    node_type: u8,
) -> NodeStatusRecord {
    can_central
        .get_nodes::<1>(node_type)
        .first()
        .map(|node| NodeStatusRecord::from_message(node.is_online(), &node.status))
        .unwrap_or_else(NodeStatusRecord::offline)
}

/// Queue between the IMU-clocked logger and the SD writer.
pub const FLIGHT_DATA_CHANNEL_DEPTH: usize = 512;
pub type FlightDataChannel = Channel<NoopRawMutex, LogRecord, FLIGHT_DATA_CHANNEL_DEPTH>;

/// Latest airbrakes state for SD logging: commanded extension, the apogee the
/// MPC predicts at it, and Icarus's reported extension and servo temperature.
///
/// Every field is NaN until its source has spoken — the firmware for
/// `commanded_extension` and `predicted_apogee_agl`, an `IcarusStatus` CAN
/// message for the other two.
/// That is the whole "is this present?" story: there are no validity flags,
/// because a NaN says it and a 0.0 would not. An Icarus that is offline or
/// silent must not read as one reporting fully-stowed brakes at 0 C.
#[derive(Clone, Copy, defmt::Format)]
pub struct AirBrakesLogState {
    pub commanded_extension: f32,
    /// Apogee AGL the MPC predicts at `commanded_extension`. NaN whenever the
    /// MPC is not running, which includes the forced validation deploy — there
    /// the commanded extension is not the MPC's output, so pairing it with a
    /// prediction would be a lie.
    pub predicted_apogee_agl: f32,
    /// `commanded_extension` is the forced validation deploy rather than the
    /// MPC's output. The one thing about the command that a NaN cannot say,
    /// so it gets its own flag: 1.0 from the MPC and 1.0 from the validation
    /// deploy are otherwise the same number.
    pub validation_deploy: bool,
    pub actual_extension: f32,
    pub servo_temp: f32,
}

impl Default for AirBrakesLogState {
    fn default() -> Self {
        Self {
            commanded_extension: f32::NAN,
            predicted_apogee_agl: f32::NAN,
            validation_deploy: false,
            actual_extension: f32::NAN,
            servo_temp: f32::NAN,
        }
    }
}

pub type AirBrakesWatch = Watch<NoopRawMutex, AirBrakesLogState, 2>;

/// Publish a commanded extension and the MPC's prediction for it, leaving the
/// Icarus-sourced fields alone. Pass NaN for the prediction when the command
/// did not come from the MPC, and set `validation_deploy` when the reason is
/// the forced validation deploy.
pub fn publish_airbrakes_commanded(
    watch: &AirBrakesWatch,
    extension: f32,
    predicted_apogee_agl: f32,
    validation_deploy: bool,
) {
    let mut state = watch.try_get().unwrap_or_default();
    state.commanded_extension = extension;
    state.predicted_apogee_agl = predicted_apogee_agl;
    state.validation_deploy = validation_deploy;
    let _ = watch.sender().send(state);
}

/// Latest AMP status heartbeat for SD logging: shared battery rail voltage
/// and the three output statuses packed 2 bits per output
/// (`PowerOutputStatus` discriminants, out1 in the LSBs) — the same encoding
/// the slow record stores.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct AmpLogState {
    pub shared_battery_v: f32,
    pub out_status: u8,
}

pub type AmpStateWatch = Watch<NoopRawMutex, AmpLogState, 2>;

/// Latest payload `CustomPayloadStatusMessage` for SD logging, kept in the
/// units it arrives in. `PAYLOAD_READING_UNAVAILABLE` (0xFFFF) marks a reading
/// the payload could not take — the same sentinel the slow record stores, and
/// the integer equivalent of the NaNs above.
#[derive(Clone, Copy, defmt::Format)]
pub struct PayloadLogState {
    pub epm_batt_mv: u16,
    /// Rail index order: 0 `SYS_3V3`, 1 `SYS_5V`, 2 `PER_3V3`, 3 `PER_5V`,
    /// 4 `PER_9V`, 5 `PER_12V`.
    pub rail_ma: [u16; 6],
    /// Experiment channels 1..3.
    pub actuator_steps: [u16; 3],
}

impl Default for PayloadLogState {
    fn default() -> Self {
        Self {
            epm_batt_mv: PAYLOAD_READING_UNAVAILABLE,
            rail_ma: [PAYLOAD_READING_UNAVAILABLE; 6],
            actuator_steps: [PAYLOAD_READING_UNAVAILABLE; 3],
        }
    }
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
/// NaN for any value the estimators have not produced yet, and for the whole
/// airbrakes group once it is retired at apogee — absent reads the same as
/// not-yet-born, with `flight_stage` dating the transition.
fn pack_estimator_sample(sample: &EstimatorLogSample) -> ((f32, f32, u8), (f32, f32, f32, u8)) {
    let mut deployment_flags = 0u8;
    if sample.deployment_baro_gate.rejected() {
        deployment_flags |= DEPLOYMENT_BARO_GATE_REJECT;
    }
    if sample.deployment_baro_gate.resynced() {
        deployment_flags |= DEPLOYMENT_BARO_RESYNC;
    }
    let deployment = (
        sample.deployment_altitude_asl,
        sample.deployment_vertical_velocity,
        deployment_flags,
    );

    let Some(ab) = sample.airbrakes.as_ref() else {
        return (deployment, (f32::NAN, f32::NAN, f32::NAN, 0));
    };

    let mut flags = 0u8;
    if ab.subsonic_by_drag.unwrap_or(false) {
        flags |= AIRBRAKES_SUBSONIC_DRAG;
    }
    if ab.burnout_detected {
        flags |= AIRBRAKES_BURNOUT;
    }
    if ab.baro_trusted {
        flags |= AIRBRAKES_BARO_TRUSTED;
    }
    if ab.is_apogee {
        flags |= AIRBRAKES_APOGEE;
    }
    if ab.baro_gate.rejected() {
        flags |= AIRBRAKES_BARO_GATE_REJECT;
    }
    if ab.baro_gate.resynced() {
        flags |= AIRBRAKES_BARO_RESYNC;
    }

    (
        deployment,
        (
            ab.altitude_asl.unwrap_or(f32::NAN),
            ab.vertical_velocity.unwrap_or(f32::NAN),
            ab.tilt_rad.map(|t| t.to_degrees()).unwrap_or(f32::NAN),
            flags,
        ),
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
/// That containment is what makes cross-session leakage unrepresentable. Every
/// subscriber and counter below is created on entry, and an embassy pubsub
/// subscriber only receives messages published after its creation, so a re-arm
/// (Armed -> LowPower -> Armed) cannot carry pre-arm samples into the new
/// session's first records — no low-power baro backlog, and no mag readings left
/// over from the *previous* armed session (`mag_task` is powered down outside
/// Armed, so its queue would otherwise be minutes stale). The estimator numbers
/// arrive on `estimator_log_watch`, which armed mode rebuilds per session
/// alongside the estimators themselves, and each carries the timestamp it was
/// taken at — so neither a previous session's sample nor a neighbouring tick's
/// can be logged against this record.
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
        // Defaults to the 0xFFFF sentinel until the payload's first status message.
        let payload = payload_state_watch.try_get().unwrap_or_default();
        let stage = flight_stage.lock(|r| *r.borrow());
        // The estimator loop publishes this from inside the same critical
        // section as its `update`, and is polled before this future in armed
        // mode's `join!`, so `stamped.0` is this very sample's timestamp. The
        // check below is what makes that a guarantee rather than an ordering
        // assumption — a mismatch means the two loops have drifted apart and
        // the gate bits would be attributed to the wrong row.
        let (
            (deployment_kf_altitude_asl, deployment_kf_vertical_velocity, deployment_flags),
            (
                airbrakes_kf_altitude_asl,
                airbrakes_kf_vertical_velocity,
                airbrakes_kf_tilt_deg,
                airbrakes_flags,
            ),
        ) = match estimator_log_watch.try_get() {
            Some((sample_timestamp_us, sample)) if sample_timestamp_us == timestamp_us => {
                pack_estimator_sample(&sample)
            }
            // Before the estimator's first sample there is no matching
            // estimator tick — log absence rather than a neighbouring tick's
            // numbers.
            other => {
                if other.is_some() && imu_opt.is_some() && !estimator_skew_warned {
                    warn!("data_logger: estimator sample is not for this tick");
                    estimator_skew_warned = true;
                }
                ((f32::NAN, f32::NAN, 0), (f32::NAN, f32::NAN, f32::NAN, 0))
            }
        };

        let mut fast_valid = 0u8;
        let mut slow_valid = 0u8;
        let (acc, gyro) = match imu_opt {
            Some(imu) => {
                fast_valid |= VALID_IMU;
                (
                    [imu.acc.x, imu.acc.y, imu.acc.z],
                    [imu.gyro.x, imu.gyro.y, imu.gyro.z],
                )
            }
            None => ([0.0; 3], [0.0; 3]),
        };
        // No validity bit: every published sample carries a baro reading, so
        // pressure (fast) and temperature (slow) are always present.
        let (temperature, pressure) = (baro_data.temperature, baro_data.pressure);
        let mag = match mag_opt {
            Some(r) => {
                fast_valid |= VALID_MAG;
                let m = r.data.mag;
                [m.x, m.y, m.z]
            }
            None => [0.0; 3],
        };

        let battery_voltage = match battery_opt {
            Some(r) => {
                slow_valid |= VALID_BATTERY;
                r.data
            }
            None => 0.0,
        };
        let (lat_lon, gps_altitude_asl, num_sats, hdop, vdop, pdop) = match gps_opt {
            Some(r) => {
                let g = r.data;
                if g.lat_lon.is_some() {
                    slow_valid |= VALID_GPS_FIX;
                }
                if g.gps_altitude_asl.is_some() {
                    slow_valid |= VALID_GPS_ALT;
                }
                (
                    g.lat_lon.unwrap_or((0.0, 0.0)),
                    g.gps_altitude_asl.unwrap_or(0.0),
                    g.num_of_fix_satellites,
                    g.hdop.unwrap_or(0.0),
                    g.vdop.unwrap_or(0.0),
                    g.pdop.unwrap_or(0.0),
                )
            }
            None => ((0.0, 0.0), 0.0, 0, 0.0, 0.0, 0.0),
        };
        let pyro_flags = match continuity_opt {
            Some(c) => {
                (c.pyro_main_continuity as u8)
                    | ((c.pyro_main_fire as u8) << 1)
                    | ((c.pyro_drogue_continuity as u8) << 2)
                    | ((c.pyro_drogue_fire as u8) << 3)
                    | ((c.short_circuit as u8) << 4)
            }
            None => 0,
        };
        // NaN carries "not present" for all four floats (see
        // `AirBrakesLogState`), so nothing here needs a valid bit — an empty
        // watch and an all-NaN state log identically, which is the truth in
        // both cases.
        let (
            air_brakes_commanded_extension,
            air_brakes_actual_extension,
            air_brakes_servo_temp,
            air_brakes_validation_deploy,
            mpc_predicted_apogee_agl,
        ) = match airbrakes_opt {
            Some(ab) => (
                ab.commanded_extension,
                ab.actual_extension,
                ab.servo_temp,
                ab.validation_deploy,
                ab.predicted_apogee_agl,
            ),
            None => (f32::NAN, f32::NAN, f32::NAN, false, f32::NAN),
        };
        let (amp_shared_battery_v, amp_out_status) = match amp_opt {
            Some(a) => (a.shared_battery_v, a.out_status),
            None => (0.0, 0),
        };
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
            // 0 until the GPS PPS has disciplined the unix clock.
            unix_time_us: unix_clock.convert_to_unix_us(timestamp_us).unwrap_or(0),
            acc,
            gyro,
            pressure,
            mag,
            deployment_kf_altitude_asl,
            deployment_kf_vertical_velocity,
            deployment_flags,
            airbrakes_kf_altitude_asl,
            airbrakes_kf_vertical_velocity,
            airbrakes_kf_tilt_deg,
            airbrakes_flags,
            pyro_flags,
            flight_stage: stage,
            valid: fast_valid,
        });
        sequence = sequence.wrapping_add(1);

        let slow_record = FlightDataSlowRecord {
            timestamp_us,
            temperature,
            battery_voltage,
            lat_lon,
            gps_altitude_asl,
            num_of_fixed_satalites: num_sats,
            hdop,
            vdop,
            pdop,
            air_brakes_commanded_extension,
            air_brakes_actual_extension,
            air_brakes_servo_temp,
            air_brakes_validation_deploy,
            mpc_predicted_apogee_agl,
            amp_node,
            icarus_node,
            ozys_node,
            payload_sdrm_node,
            amp_out_status,
            amp_shared_battery_v,
            payload_epm_batt_mv: payload.epm_batt_mv,
            payload_rail_ma: payload.rail_ma,
            payload_actuator_steps: payload.actuator_steps,
            valid: slow_valid,
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
