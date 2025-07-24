use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, watch::Watch};
use firmware_common_new::vlp::{client::VLPAvionics, packets::low_power_telemetry::LowPowerTelemetryPacketBuilder};

use crate::avionics_mode::AvionicsMode;

#[embassy_executor::task]
async fn low_power_mode(
    spawner: &'static Spawner,
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode: &'static Watch<CriticalSectionRawMutex, AvionicsMode, 10>,
){
    let mut packet_builder = LowPowerTelemetryPacketBuilder::<NoopRawMutex>::new();

}