//! Fake GPS publisher so mode loops that wait on GPS do not stall.

use defmt::info;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch,
};
use embassy_time::{Duration, Instant, Ticker};
use firmware_common_new::{
    gps::GPSData,
    sensor_reading::SensorReading,
    time::BootTimestamp,
};

use crate::{GPSReadingWatch, VLStatusMutex};

#[embassy_executor::task]
pub async fn hil_gps_stub(
    gps_reading_watch: &'static GPSReadingWatch,
    vl_status: &'static VLStatusMutex,
) {
    info!("HIL: gps_stub started");
    vl_status.lock(|s| {
        s.borrow_mut().gps_ok = true;
    });

    let sender: watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorReading<BootTimestamp, GPSData>,
        4,
    > = gps_reading_watch.sender();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        ticker.next().await;
        sender.send(SensorReading::new(
            Instant::now().as_micros(),
            GPSData {
                timestamp: Some(1_700_000_000),
                lat_lon: Some((43.26, -79.92)),
                altitude: Some(200.0),
                num_of_fix_satellites: 8,
                hdop: Some(1.0),
                vdop: Some(1.0),
                pdop: Some(1.5),
            },
        ));
    }
}
