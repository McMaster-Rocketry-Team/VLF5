//! Scripted VLP uplinks + downlink logger for HIL (bypasses SX126x).

use defmt::info;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::vlp::{
    client::VLPAvionics,
    packets::{
        change_mode::{ChangeModePacket, Mode},
        set_target_apogee::SetTargetApogeePacket,
        VLPDownlinkPacket, VLPUplinkPacket,
    },
};

/// Consumes downlinks that armed/low-power modes try to send over LoRa and logs them.
#[embassy_executor::task]
pub async fn hil_vlp_bridge_task(vlp: &'static VLPAvionics<NoopRawMutex>) {
    info!("HIL: vlp_bridge started (logging downlinks, no radio)");
    loop {
        let packet = vlp.wait_downlink().await;
        match &packet {
            VLPDownlinkPacket::Telemetry(t) => {
                info!(
                    "HIL downlink Telemetry: stage={} alt_agl={} max_agl={} air_speed={} airbrakes_cmd={}",
                    t.flight_stage(),
                    t.altitude_agl(),
                    t.max_altitude_agl(),
                    t.air_speed(),
                    t.air_brakes_commanded_extension_percentage(),
                );
            }
            VLPDownlinkPacket::LowPowerTelemetry(_) => {
                info!("HIL downlink LowPowerTelemetry");
            }
            VLPDownlinkPacket::LandedTelemetry(_) => {
                info!("HIL downlink LandedTelemetry");
            }
            VLPDownlinkPacket::SelfTestResult(_) => {
                info!("HIL downlink SelfTestResult");
            }
            VLPDownlinkPacket::GPSBeacon(_) => {
                info!("HIL downlink GPSBeacon");
            }
            VLPDownlinkPacket::Ack(_) => {
                info!("HIL downlink Ack");
            }
        }
        // Drop the packet — nothing ACKs over the air in HIL.
        let _ = packet;
    }
}

/// Fixed timeline of ground-station uplinks injected into `VLPAvionics`.
#[embassy_executor::task]
pub async fn hil_script_task(vlp: &'static VLPAvionics<NoopRawMutex>) {
    #[cfg(feature = "hil-dual")]
    info!("HIL: script started (dual-deploy plot)");
    #[cfg(feature = "hil-single")]
    info!("HIL: script started (single-deploy plot)");
    let t0 = Instant::now();

    Timer::after(Duration::from_secs(2)).await;
    info!(
        "HIL script t={} ms: SetTargetApogee 4000 m",
        (Instant::now() - t0).as_millis()
    );
    vlp.inject_uplink(VLPUplinkPacket::SetTargetApogee(SetTargetApogeePacket::new(
        4000.0,
    )));

    Timer::after(Duration::from_secs(1)).await;
    info!(
        "HIL script t={} ms: ChangeMode Armed",
        (Instant::now() - t0).as_millis()
    );
    vlp.inject_uplink(VLPUplinkPacket::ChangeMode(ChangeModePacket {
        mode: Mode::Armed,
    }));

    // Script done; sensor profile + estimator drive the rest of the flight.
    #[cfg(feature = "hil-dual")]
    info!("HIL script: expect PyroDrogue near apogee, PyroMain near 457 m AGL");
    #[cfg(feature = "hil-single")]
    info!("HIL script: expect PyroDrogue then PyroMain back-to-back at apogee");
    core::future::pending::<()>().await;
}
