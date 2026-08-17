#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]
#![feature(try_blocks)]
#![feature(future_join)]

use core::cell::RefCell;

use {defmt_rtt_pipe as _, panic_probe as _};

// `boot-armed` removes the operator from the arming decision, so it must never
// be reachable from a flight build. HIL builds fake the barometer, which is
// what makes an unattended Armed boot a bench exercise rather than a live
// rocket.
#[cfg(all(feature = "boot-armed", not(feature = "hil-replay")))]
compile_error!("boot-armed is a HIL-only bench shortcut; it requires hil-dual or hil-single");

use air_brakes_controller_core::{
    DeploymentProfile, FlightConfig, FlightEstimators, FlightProfile, RocketParameters,
    airbrakes_estimator::AirbrakesConfig,
};
#[cfg(not(feature = "hil-single"))]
use air_brakes_controller_core::airbrakes_estimator::MachLockoutConfig;
use binary_macros::base64;
use cortex_m::singleton;
use cortex_m_rt::entry;
use embassy_executor::{Executor, InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::{
    Peri, Peripherals,
    can::{CanRx, CanTx},
    gpio::{Level, Output, Speed},
    interrupt::{self, InterruptExt as _, Priority},
    peripherals::{PA2, PA7},
};
use embassy_sync::{
    blocking_mutex::{
        Mutex as BlockingMutex,
        raw::{CriticalSectionRawMutex, NoopRawMutex},
    },
    channel::Channel,
    mutex::Mutex,
    pubsub::PubSubBehavior as _,
    watch::{self, Watch},
};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use firmware_common_new::{
    can_bus::{
        custom_status::vl_custom_status::VLCustomStatus,
        messages::{
            baro_measurement::BaroMeasurementMessage,
            imu_measurement::IMUMeasurementMessage, mag_measurement::MagMeasurementMessage,
            vl_status::FlightStage,
        },
        sender::CanSender,
    },
    gps::GPSData,
    sensor_reading::SensorReading,
    time::BootTimestamp,
    vlp::{client::VLPAvionics, packets::fire_pyro::PyroSelect},
};
use firmware_common_new::vlp::lora_config::LoraConfig;

use crate::{
    avionics_mode::AvionicsMode,
    watchdog::watchdog_task,
    can::{can_bus_broadcast_unix_time_task, init_can_bus, start_can_bus_low_prio_tasks},
    clock_config::vlf5_clock_config,
    modes::{
        armed_mode::armed_mode, demo_mode::demo_mode, landed_mode::landed_mode,
        low_power_mode::low_power_mode, self_test_mode::self_test_mode,
    },
    tasks::{
        amp_control_task::{AmpControlWatch, amp_control_task},
        buzzer_task::{BuzzerPubSub, BuzzerTone, DISABLE_BUZZER, buzzer_task},
        data_logger::{AirBrakesWatch, AmpStateWatch, PayloadStateWatch},
        pyro_task::ContinuityUpdate,
        sensor_tasks::{BatteryVWatch, IMUBaroReadingPubSub, MagReadingPubSub, adc_task, mag_task},
        unix_clock::{UnixClock, unix_clock_task},
    },
};
use crate::tasks::{
    gps_task::gps_task, pyro_task::pyro_task, sensor_tasks::imu_baro_task,
    vlp_avionics_daemon_task::vlp_avionics_daemon_task,
};
use receive_vlp_task::{fire_pyro_countdown_task, receive_vlp_task};

mod avionics_mode;
mod can;
mod can_central;
mod clock_config;
mod drivers;
#[cfg(feature = "hil-replay")]
mod hil;
mod modes;
mod receive_vlp_task;
mod tasks;
mod time;
mod utils;
mod usb_handler;
mod watchdog;

static VLP_KEY: &[u8] = base64!("file:vlp.key");
const LORA_CONFIG: LoraConfig = LoraConfig {
    frequency: 920_000_000,
    sf: 12,
    bw: 250000,
    cr: 8,
    power: 22,
};
/// Osiris on the CTI O3400, from `2026_06_26 - Osiris LC FDR.ork` (all 12
/// simulations in that file re-run headlessly on OpenRocket 24.12, so the
/// numbers below are spreads over launch conditions, not one sim) and the
/// airbrakes CFD table in the Osiris FDR.
///
/// Motor-config sims (burnout 6.322 s, liftoff 29.968 kg): apogee 9347–9489 m
/// AGL at 39.29–39.70 s, peak Mach 1.91–1.92, back below Mach 0.8 at
/// 17.52–17.57 s and below Mach 0.75 at 18.62–18.67 s. The N3301/N2900 backup
/// motors go subsonic earlier (15.7–16.1 s) and apogee earlier (37.7–38.2 s),
/// which only costs control window here — every bound below is set from the
/// O3400 and stays safe on the backups.
///
/// The `hil-dual` bench build flies THIS config, not a HIL-specific one, and
/// replays the same OpenRocket trajectory it was derived from (see
/// `hil::osiris`). A bench profile with its own softened numbers would prove
/// only that the bench profile works.
#[cfg(not(feature = "hil-single"))]
pub const FLIGHT_CONFIG: FlightConfig = FlightConfig {
    profile: FlightProfile {
        // Mach 0.75 at 18.67 s worst case, x1.4; ends 13 s before the
        // earliest simulated apogee.
        mach_lockout_duration_us: Some(26_000_000),
        // The ONLY ignition detector this half has. The barometric one it
        // replaced completed at ignition+1.12 s on this trajectory while the
        // airframe crosses Mach 0.8 at +1.88 s — it was deciding the start
        // of the Mach lockout 0.75 s before the static port it reads began
        // lying. This latches at +0.15 s instead.
        //
        // 8 g: boost holds 14 g for the whole 6.3 s burn, so on this motor
        // 8 g costs only 15-22 ms over 4 g while doubling the margin over
        // anything that can happen to a rocket on a rail. Measured latch
        // times after true ignition, sustain included — O3400 0.136 s,
        // N2900 0.148 s (`ignition_latch_time_by_threshold`). Cheap here,
        // but airframe-specific: on LC'25's softer curve 8 g costs 0.45 s
        // and 10 g never latches at all.
        //
        // The airbrakes half below runs the same detector at the same
        // number. They are two settings, but "the motor lit" is one event,
        // so on this airframe they get one answer — and the diagnostic
        // asserts the two halves latch on the same sample.
        ignition_detection_acc_threshold: 8.0 * 9.81,
        deployment: DeploymentProfile::Dual {
            drogue_chute_minimum_altitude_agl: 2000.0,
            drogue_chute_delay_us: 1_000_000,
            // 1500 ft AGL, FDR R4.3.1 / CONOPS.
            main_chute_altitude_agl: 457.2,
            main_chute_delay_us: 0,
        },
    },
    airbrakes: AirbrakesConfig {
        // Same 8 g as the pyro half above, and for the same reason. This was
        // 4 g while this half had no sustain, which meant the looser
        // threshold and the weaker transient rejection sat on the same
        // detector; sharing the implementation removed the second half of
        // that and matching the number removes the first.
        //
        // The cost lands entirely on this half — it used to latch ~0.02 s
        // after ignition and now latches at 0.136 s with the rest. That lag
        // is the error in the origin of the two timers below, and 0.115 s
        // against a 17.5 s floor is not a number either of them can feel.
        ignition_detection_acc_threshold: 8.0 * 9.81,
        mach_lockout: Some(MachLockoutConfig {
            // Earliest simulated time below Mach 0.8: 17.52 s. This floor,
            // not the velocity ceiling, is what actually keeps an early
            // birth out: on the O3400 sim a 5x-wrong Cd moves birth only
            // from 18.89 s to 18.52 s, because the check cannot speak before
            // this.
            earliest_subsonic_after_ignition_us: 17_500_000,
            // Latest (17.57 s) x1.4; still 14 s before the earliest apogee.
            force_birth_after_ignition_us: 25_000_000,
            // The O3400 crosses Mach 0.8 at 6734 m in the sim the two timers
            // above come from; this is that, rounded up, since erring HIGH
            // reads density low and airspeed high and so only ever delays
            // the exit. The N2900 backup crosses 1151 m lower (5583 m), and
            // taking the higher of the two is the same one-sided choice —
            // flying the backup then makes the check vote at Mach 0.736
            // instead of 0.796, which `mach_lockout_timers_bracket_every_simulation`
            // asserts for both motors.
            subsonic_crossing_altitude_asl: 6800.0,
        }),
        // The Mach the CFD Cd table below is tabulated at (FDR Table 10 is a
        // Mach 0.8, 4000 m sweep), which is also the speed below which the
        // flaps may open — one fact, so one number. The drag check votes at
        // it and the MPC gate refuses above it. Measured birth lands at Mach
        // 0.726 nominal and 0.743 with a 5x-wrong Cd, so the gate is slack
        // on a healthy flight without a second, higher constant.
        max_open_mach: 0.8,
        rocket: RocketParameters {
            burnout_mass: 18.696,
            // FDR Table 10 — STAR-CCM+ drag at Mach 0.8 and 4000 m for 0/25/
            // 50/75/100% flap extension (167/190/220/263/306 N), divided by
            // `q * reference_area` at ISA 4000 m (rho 0.8191, a 324.6 m/s,
            // q 27615 Pa). The stowed 0.614 sits 3.6% above OpenRocket's own
            // Mach-0.8 Cd of 0.592, which is the cross-check that the CFD
            // numbers are whole-body and share this reference area.
            cd: [0.61365, 0.69816, 0.8084, 0.96641, 1.12441],
            // pi/4 * (0.112017 m)^2 — OpenRocket's reference area for this
            // airframe, taken from the widest transition, not the 105.7 mm
            // body tube.
            reference_area: 0.009854945,
        },
    },
};

#[cfg(feature = "hil-single")]
pub const FLIGHT_CONFIG: FlightConfig = FlightConfig {
    profile: FlightProfile {
        mach_lockout_duration_us: None,
        // 4 g, not the Osiris profile's 8 g: this bench's scripted motor is
        // a constant 80 m/s^2 net, so the accelerometer sees 9.15 g and 8 g
        // would leave barely a g of margin. Different airframe, different
        // number — which is why this is per-profile config.
        ignition_detection_acc_threshold: 4.0 * 9.81,
        deployment: DeploymentProfile::Single {
            minimum_deployment_altitude_agl: 2000.0,
            delay_us: 0,
        },
    },
    airbrakes: AirbrakesConfig {
        ignition_detection_acc_threshold: 4.0 * 9.81,
        mach_lockout: None,
        // No lockout on this bench profile, so the drag check never reads
        // this; it is only the MPC gate's ceiling here. Kept at the flight
        // value so the gate behaves the same on the bench.
        max_open_mach: 0.8,
        rocket: RocketParameters {
            burnout_mass: 17.607,
            cd: [0.47044, 0.5082, 0.57784, 0.665, 0.74313],
            reference_area: 0.008982476,
        },
    },
};

pub type AvionicsModeWatch = Watch<CriticalSectionRawMutex, AvionicsMode, 10>;
pub type GPSReadingWatch = Watch<CriticalSectionRawMutex, SensorReading<BootTimestamp, GPSData>, 4>;
pub type VLStatusMutex = BlockingMutex<CriticalSectionRawMutex, RefCell<VLCustomStatus>>;
pub type FlightStageMutex = BlockingMutex<NoopRawMutex, RefCell<FlightStage>>;
pub type FlightEstimatorsMutex = BlockingMutex<NoopRawMutex, RefCell<FlightEstimators>>;
pub type ContinuityWatch = Watch<NoopRawMutex, ContinuityUpdate, 2>;
pub type FireSignal = Channel<NoopRawMutex, PyroSelect, 2>;
pub type SetTargetWatch = Watch<NoopRawMutex, f32, 1>;

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(vlf5_clock_config());

    let buzzer_pubsub = singleton!(: BuzzerPubSub = BuzzerPubSub::new()).unwrap();
    let avionics_mode = singleton!(: AvionicsModeWatch = AvionicsModeWatch::new()).unwrap();
    #[cfg(feature = "hil-dual")]
    defmt::warn!(
        "HIL-DUAL build: baro is SIMULATED; IMU/GPS/mag/CAN/SD/LoRa are REAL and the real pyro task drives real GPIO — do NOT connect e-matches"
    );
    #[cfg(feature = "hil-single")]
    defmt::warn!(
        "HIL-SINGLE build: baro is SIMULATED; IMU/GPS/mag/CAN/SD/LoRa are REAL and the real pyro task drives real GPIO — do NOT connect e-matches"
    );
    // Flight and HIL boot identically into SelfTest; the operator arms over the radio.
    // Boot mode. Flight builds always come up in SelfTest and are armed by an
    // operator over the radio — arming is a human decision and stays one.
    //
    // `boot-armed` skips that, for a bench with no ground station attached.
    // It is gated to HIL builds at the top of this file because the thing it
    // removes is the only thing standing between power-on and the pyro task
    // energizing the drogue and main FETs on its own, roughly a minute later.
    // NEVER power a `boot-armed` board with e-matches connected.
    #[cfg(feature = "boot-armed")]
    {
        defmt::warn!(
            "boot-armed: entering Armed with no operator — pyro GPIO WILL fire at apogee"
        );
        avionics_mode.sender().send(AvionicsMode::Armed);
    }
    #[cfg(not(feature = "boot-armed"))]
    avionics_mode.sender().send(AvionicsMode::SelfTest);
    let imu_baro_reading_pubsub =
        singleton!(: IMUBaroReadingPubSub = IMUBaroReadingPubSub::new()).unwrap();
    let mag_reading_pubsub = singleton!(: MagReadingPubSub = MagReadingPubSub::new()).unwrap();

    let gps_reading_watch = singleton!(: GPSReadingWatch = GPSReadingWatch::new()).unwrap();
    let vl_status =
        singleton!(: VLStatusMutex = BlockingMutex::new(RefCell::new(VLCustomStatus::new())))
            .unwrap();
    let unix_clock = singleton!(: UnixClock = UnixClock::new()).unwrap();

    let (can_tx, can_rx) = init_can_bus(p.FDCAN2, p.PB5, p.PB6);

    static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
    #[embassy_stm32::interrupt]
    unsafe fn USART2() {
        unsafe { EXECUTOR_HIGH.on_interrupt() }
    }

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::USART2);
    spawner.spawn(high_prio_main(
        spawner,
        can_tx,
        buzzer_pubsub,
        avionics_mode,
        imu_baro_reading_pubsub,
        mag_reading_pubsub,
        gps_reading_watch,
        vl_status,
        unix_clock,
    ).unwrap());

    let executor_low = singleton!(: Executor = Executor::new()).unwrap();
    executor_low.run(|spawner| {
        spawner.spawn(low_prio_main(
            spawner,
            can_tx,
            can_rx,
            buzzer_pubsub,
            avionics_mode,
            imu_baro_reading_pubsub,
            mag_reading_pubsub,
            gps_reading_watch,
            vl_status,
            unix_clock,
        ).unwrap());
    })
}

#[embassy_executor::task]
async fn high_prio_main(
    spawner: SendSpawner,
    can_tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    buzzer_pubsub: &'static BuzzerPubSub,
    avionics_mode_watch: &'static AvionicsModeWatch,
    imu_baro_reading_pubsub: &'static IMUBaroReadingPubSub,
    mag_reading_pubsub: &'static MagReadingPubSub,
    gps_reading_watch: &'static GPSReadingWatch,
    vl_status: &'static VLStatusMutex,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    spawner.spawn(buzzer_task(p.PC15, buzzer_pubsub).unwrap());
    spawner.spawn(imu_baro_task(
        p.SPI4,
        p.PE2,
        p.PE6,
        p.PE5,
        p.PC13,
        p.DMA2_CH1,
        p.DMA2_CH0,
        p.PC14,
        p.EXTI14,
        p.SPI1,
        p.PA5,
        p.PD7,
        p.PA6,
        p.PC6,
        p.DMA1_CH4,
        p.DMA1_CH5,
        imu_baro_reading_pubsub,
        vl_status,
        avionics_mode_watch,
    ).unwrap());
    spawner.spawn(mag_task(
        p.I2C2,
        p.PB10,
        p.PB11,
        p.DMA1_CH7,
        p.DMA1_CH6,
        mag_reading_pubsub,
        vl_status,
        avionics_mode_watch,
    ).unwrap());
    spawner.spawn(unix_clock_task(
        p.PA15,
        p.EXTI15,
        unix_clock,
        gps_reading_watch,
    ).unwrap());
    spawner.spawn(can_bus_broadcast_unix_time_task(can_tx, unix_clock).unwrap());
}

#[embassy_executor::task]
async fn low_prio_main(
    spawner: Spawner,
    can_tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    can_rx: CanRx<'static>,
    buzzer_pubsub: &'static BuzzerPubSub,
    avionics_mode_watch: &'static AvionicsModeWatch,
    imu_baro_reading_pubsub: &'static IMUBaroReadingPubSub,
    mag_reading_pubsub: &'static MagReadingPubSub,
    gps_reading_watch: &'static GPSReadingWatch,
    vl_status: &'static VLStatusMutex,
    unix_clock: &'static UnixClock,
) {
    let p = unsafe { Peripherals::steal() };
    let mut ps = Output::new(p.PA3, Level::Low, Speed::Low);

    let vlp_avionics_client = singleton!(: VLPAvionics<NoopRawMutex> = VLPAvionics::new()).unwrap();
    let flight_stage =
        singleton!(: FlightStageMutex = BlockingMutex::new(RefCell::new(FlightStage::Armed)))
            .unwrap();
    let battery_v_watch = singleton!(: BatteryVWatch = BatteryVWatch::new()).unwrap();
    let air_brakes_watch = singleton!(: AirBrakesWatch = AirBrakesWatch::new()).unwrap();
    let amp_state_watch = singleton!(: AmpStateWatch = AmpStateWatch::new()).unwrap();
    let payload_state_watch =
        singleton!(: PayloadStateWatch = PayloadStateWatch::new()).unwrap();

    let continuity_watch = singleton!(: ContinuityWatch = ContinuityWatch::new()).unwrap();
    let fire_signal = singleton!(: FireSignal = FireSignal::new()).unwrap();
    // Manual `fire-pyro` requests are handed here so the buzzer countdown runs off
    // `receive_vlp_task` (see `fire_pyro_countdown_task`); autonomous deploy still
    // uses `fire_signal` directly.
    let fire_pyro_request = singleton!(: FireSignal = FireSignal::new()).unwrap();
    let amp_control_watch = singleton!(: AmpControlWatch = AmpControlWatch::new()).unwrap();
    let target_agl_signal = singleton!(:SetTargetWatch= SetTargetWatch::new()).unwrap();

    let storage_cmd = singleton!(
        : tasks::sd_card_writer::StorageCmdSignal = tasks::sd_card_writer::StorageCmdSignal::new()
    )
    .unwrap();
    let storage_resp = singleton!(
        : tasks::sd_card_writer::StorageRespChannel = tasks::sd_card_writer::StorageRespChannel::new()
    )
    .unwrap();
    spawner.spawn(usb_handler::setup_usb_handler(
        p.USB_OTG_FS,
        p.PA12,
        p.PA11,
        storage_cmd,
        storage_resp,
        spawner,
    ).unwrap());

    spawner.spawn(power_led_task(
        p.PA2,
        p.PA7,
        gps_reading_watch.receiver().unwrap(),
    ).unwrap());
    spawner.spawn(adc_task(p.ADC1, p.PB0, battery_v_watch).unwrap());
    spawner.spawn(pyro_task(
        p.PE9,
        p.PE13,
        p.EXTI13,
        p.PD8,
        p.PD13,
        p.PD9,
        p.PE12,
        p.EXTI12,
        continuity_watch,
        fire_signal,
    ).unwrap());
    spawner.spawn(gps_task(
        p.USART1,
        p.PA10,
        p.PB14,
        gps_reading_watch.sender(),
        vl_status,
    ).unwrap());
    // VLP always runs over the real LoRa radio + GCM, in both flight and HIL builds.
    spawner.spawn(vlp_avionics_daemon_task(
        vlp_avionics_client,
        VLP_KEY.try_into().unwrap(),
        LORA_CONFIG.clone(),
        p.SPI3,
        p.PB3,
        p.PD6,
        p.PB4,
        p.PC7,
        p.PD5,
        p.PD4,
        p.EXTI4,
        p.PD1,
        p.EXTI1,
        p.PD0,
        p.PA8,
        p.DMA1_CH3,
        p.DMA1_CH2,
    ).unwrap());
    let (can_sender, can_receiver, can_central) = start_can_bus_low_prio_tasks(
        &spawner,
        can_tx,
        can_rx,
        flight_stage,
        battery_v_watch,
        air_brakes_watch,
        amp_state_watch,
        payload_state_watch,
        vl_status,
    )
    .await;

    spawner.spawn(receive_vlp_task(
        vlp_avionics_client,
        avionics_mode_watch,
        fire_pyro_request,
        target_agl_signal,
        can_sender,
        can_central,
        storage_cmd,
    ).unwrap());
    spawner.spawn(
        fire_pyro_countdown_task(fire_pyro_request, fire_signal, buzzer_pubsub).unwrap(),
    );

    spawner.spawn(broadcast_imu_baro_measurement_task(
        imu_baro_reading_pubsub,
        can_sender,
        unix_clock,
    ).unwrap());
    spawner.spawn(broadcast_mag_measurement_task(
        mag_reading_pubsub,
        can_sender,
        unix_clock,
    ).unwrap());
    spawner.spawn(amp_control_task(can_sender, amp_control_watch).unwrap());

    let flight_data_channel = singleton!(
        
        : tasks::data_logger::FlightDataChannel = tasks::data_logger::FlightDataChannel::new()
    )
    .unwrap();
    // No logger task: flight-data logging runs as one of armed mode's joined
    // futures (`tasks::data_logger::log_flight_data`), so it exists exactly as
    // long as an armed session. The SD writer stays a task — it owns SDMMC1 —
    // and is fed over this channel.
    spawner.spawn(tasks::sd_card_writer::sd_card_writer(
        p.SDMMC1,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        flight_data_channel,
        storage_cmd,
        storage_resp,
        vl_status,
        target_agl_signal,
    ).unwrap());

    if cfg!(not(debug_assertions)) {
        spawner.spawn(watchdog_task(p.IWDG1).unwrap());
    } else {
        defmt::warn!("Watchdog is disabled in debug build");
    }

    if !DISABLE_BUZZER {
        buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
        buzzer_pubsub.publish_immediate(BuzzerTone::Mid(250, 100));
        // 500 ms of silence, not the usual 100: it separates the fixed
        // power-on burst from the continuity report that follows, so the two
        // do not run together as one uniform five-note run.
        buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 500));

        // Then report igniter continuity: two highs if both channels are
        // good, two lows otherwise. `pyro_task` publishes its first reading
        // ~10 ms after boot (it waits for the pyro supply to come up), but
        // boot must not hang on it — a timeout counts as "not good", which is
        // the honest answer when continuity cannot be read at all.
        let continuity = {
            let mut receiver = continuity_watch.receiver().unwrap();
            with_timeout(Duration::from_millis(500), receiver.get())
                .await
                .ok()
        };
        let both_good = continuity
            .map(|c| c.pyro_main_continuity && c.pyro_drogue_continuity)
            .unwrap_or(false);
        if both_good {
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
        } else {
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
        }
    }

    loop {
        match avionics_mode_watch.try_get().unwrap() {
            AvionicsMode::Armed => {
                ps.set_low();
                armed_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_sender,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    continuity_watch,
                    fire_signal,
                    target_agl_signal,
                    can_receiver.subscriber().unwrap(),
                    flight_stage,
                    amp_control_watch,
                    air_brakes_watch,
                    unix_clock,
                    mag_reading_pubsub,
                    amp_state_watch,
                    payload_state_watch,
                    vl_status,
                    flight_data_channel,
                )
                .await;
            }
            AvionicsMode::SelfTest => {
                ps.set_high();
                self_test_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    vl_status,
                    amp_control_watch,
                    flight_stage,
                    continuity_watch,
                    can_receiver.subscriber().unwrap(),
                    buzzer_pubsub,
                )
                .await
            }
            AvionicsMode::LowPower => {
                ps.set_high();
                low_power_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    flight_stage,
                )
                .await
            }
            AvionicsMode::Landed => {
                ps.set_high();
                landed_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    flight_stage,
                )
                .await
            }
            AvionicsMode::Demo => {
                ps.set_high();
                demo_mode(
                    vlp_avionics_client,
                    avionics_mode_watch,
                    can_sender,
                    can_central,
                    gps_reading_watch,
                    battery_v_watch,
                    imu_baro_reading_pubsub,
                    can_receiver.subscriber().unwrap(),
                    amp_control_watch,
                    air_brakes_watch,
                    flight_stage,
                )
                .await
            }
        }
    }
}

#[embassy_executor::task]
async fn power_led_task(
    blue_led: Peri<'static, PA2>,
    green_led: Peri<'static, PA7>,
    mut gps_reading: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        4,
    >,
) {
    let mut blue_led = Output::new(blue_led, Level::High, Speed::Low);
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let gps_fixed = if let Some(gps_reading) = gps_reading.try_get() {
            gps_reading.data.lat_lon.is_some()
        } else {
            false
        };
        let led = if gps_fixed {
            &mut green_led
        } else {
            &mut blue_led
        };
        led.set_low();
        Timer::after_millis(50).await;
        led.set_high();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn broadcast_imu_baro_measurement_task(
    imu_baro_pubsub: &'static IMUBaroReadingPubSub,
    can_sender: &'static CanSender<NoopRawMutex>,
    unix_clock: &'static UnixClock,
) {
    let mut sub = imu_baro_pubsub.subscriber().unwrap();
    let mut last_send_time_us = Instant::now().as_micros();
    loop {
        let reading = sub.next_message_pure().await;
        let now_us = Instant::now().as_micros();
        if now_us - last_send_time_us < 9000 {
            continue;
        }
        last_send_time_us = now_us;

        if let Some(imu_data) = reading.data.0 {
            can_sender.send(
                IMUMeasurementMessage::new(
                    unix_clock
                        .convert_to_unix_us(reading.timestamp_us)
                        .unwrap_or(reading.timestamp_us),
                    &imu_data.acc,
                    &imu_data.gyro,
                )
                .into(),
            );
        }

        let baro_data = reading.data.1;
        can_sender.send(
            BaroMeasurementMessage::new(
                unix_clock
                    .convert_to_unix_us(reading.timestamp_us)
                    .unwrap_or(reading.timestamp_us),
                baro_data.pressure,
                baro_data.temperature,
            )
            .into(),
        );
    }
}

#[embassy_executor::task]
async fn broadcast_mag_measurement_task(
    mag_pubsub: &'static MagReadingPubSub,
    can_sender: &'static CanSender<NoopRawMutex>,
    unix_clock: &'static UnixClock,
) {
    let mut sub = mag_pubsub.subscriber().unwrap();
    loop {
        let reading = sub.next_message_pure().await;

        can_sender.send(
            MagMeasurementMessage::new(
                unix_clock
                    .convert_to_unix_us(reading.timestamp_us)
                    .unwrap_or(reading.timestamp_us),
                &reading.data.mag.into(),
            )
            .into(),
        );
    }
}
