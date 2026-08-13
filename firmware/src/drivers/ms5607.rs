use crate::sleep;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use firmware_common_new::readings::BaroData;
use firmware_common_new::sensor_reading::SensorReading;
use firmware_common_new::time::BootTimestamp;

/// Pressure conversion at OSR=512 (1.17 ms). The whole read must fit inside one
/// LSM6DSM data-ready period (~2.34 ms) so the IMU+baro loop publishes on every
/// IMU sample — OSR=1024's 2.28 ms conversion overran it and halved the
/// effective sample rate (Void Lake flight logged 213.5 Hz instead of ~416 Hz).
const PRESSURE_CONVERT_CMD: u8 = 0x42;
const PRESSURE_CONVERT_TIME_US: u64 = 1170;

/// Temperature conversion at OSR=256 (0.60 ms), refreshed once every
/// [`TEMP_DECIMATION`] pressure reads (~75 ms at 416 Hz). Die temperature moves
/// far slower than that; skipping it keeps most read cycles pressure-only.
const TEMP_CONVERT_CMD: u8 = 0x50;
const TEMP_CONVERT_TIME_US: u64 = 600;
const TEMP_DECIMATION: u8 = 32;

#[derive(Clone, Copy)]
struct Coefficients {
    sens_t1: u16,
    off_t1: u16,
    tcs: u16,
    tco: u16,
    t_ref: u16,
    tempsens: u16,
}

pub struct MS5607<'a, B: SpiDevice> {
    spi: B,
    coefficients: Option<Coefficients>,
    buffer: &'a mut [u8],
    /// Last raw temperature ADC value (D2); reused between temperature refreshes.
    last_d2: Option<u32>,
    reads_since_temp: u8,
}

macro_rules! create_buffer {
    ($self: expr, $write_data: expr) => {{
        let read_length = $write_data.len(); // read length is equal to the length of the write data
        let (read_buffer, write_buffer) = $self.buffer.split_at_mut(read_length);
        let write_buffer = &mut write_buffer[..$write_data.len()];
        write_buffer.copy_from_slice(&$write_data);
        (read_buffer, write_buffer)
    }};
}

impl<'a, B: SpiDevice> MS5607<'a, B> {
    pub fn new(spi_device: B, buffer: &'a mut [u8]) -> Self {
        defmt::assert!(buffer.len() == 8, "Buffer length must be 8");
        Self {
            spi: spi_device,
            coefficients: None,
            buffer,
            last_d2: None,
            reads_since_temp: 0,
        }
    }

    pub async fn reset(&mut self) -> Result<(), B::Error> {
        let (read_buffer, write_buffer) = create_buffer!(self, [0x1E]);
        // reset
        self.spi
            .transfer(read_buffer, write_buffer)
            .await?;

        sleep!(20);

        // read coefficients
        let mut coefficients = [0u16; 6];
        for addr in 1..=6 {
            let (read_buffer, write_buffer) = create_buffer!(self, [0xA0 | (addr << 1), 0, 0]);
            self.spi
                .transfer(read_buffer, write_buffer)
                .await?;

            coefficients[(addr - 1) as usize] =
                ((read_buffer[1] as u16) << 8) | (read_buffer[2] as u16);
        }
        self.coefficients = Some(Coefficients {
            sens_t1: coefficients[0],
            off_t1: coefficients[1],
            tcs: coefficients[2],
            tco: coefficients[3],
            t_ref: coefficients[4],
            tempsens: coefficients[5],
        });
        self.last_d2 = None;
        self.reads_since_temp = 0;

        Ok(())
    }

    pub async fn read(&mut self) -> Result<SensorReading<BootTimestamp, BaroData>, B::Error> {
        // request pressure measurement; stamp the middle of its integration window
        let timestamp = Instant::now().as_micros() + PRESSURE_CONVERT_TIME_US / 2;
        let (read_buffer, write_buffer) = create_buffer!(self, [PRESSURE_CONVERT_CMD]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await?;
        Timer::after(Duration::from_micros(PRESSURE_CONVERT_TIME_US)).await;

        // read pressure measurement
        let (read_buffer, write_buffer) = create_buffer!(self, [0x00, 0, 0, 0]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await?;
        let d1 = ((read_buffer[1] as u32) << 16)
            | ((read_buffer[2] as u32) << 8)
            | (read_buffer[3] as u32);

        let d2 = if self.last_d2.is_none() || self.reads_since_temp >= TEMP_DECIMATION {
            // request temperature measurement
            let (read_buffer, write_buffer) = create_buffer!(self, [TEMP_CONVERT_CMD]);
            self.spi
                .transfer(read_buffer, write_buffer)
                .await?;
            Timer::after(Duration::from_micros(TEMP_CONVERT_TIME_US)).await;

            // read temperature measurement
            let (read_buffer, write_buffer) = create_buffer!(self, [0x00, 0, 0, 0]);
            self.spi
                .transfer(read_buffer, write_buffer)
                .await?;
            let d2 = ((read_buffer[1] as u32) << 16)
                | ((read_buffer[2] as u32) << 8)
                | (read_buffer[3] as u32);
            self.last_d2 = Some(d2);
            self.reads_since_temp = 0;
            d2
        } else {
            self.reads_since_temp += 1;
            self.last_d2.unwrap()
        };

        let coeffs = self.coefficients.unwrap();

        // temperature calculation
        let dt: i64 = ((d2 as i32) - ((coeffs.t_ref as i32) << 8)) as i64;
        let mut temperature = (2000i64 + ((dt * coeffs.tempsens as i64) >> 23)) as i32;

        // compensated pressure calculation
        let mut off = ((coeffs.off_t1 as i64) << 17) + (((coeffs.tco as i64) * dt) >> 6);
        let mut sens = ((coeffs.sens_t1 as i64) << 16) + (((coeffs.tcs as i64) * dt) >> 7);

        // second order temperature compensation
        if temperature < 2000 {
            let t2 = ((dt * dt) >> 31) as i32;
            let mut off2 = (61 * (temperature - 2000) * (temperature - 2000)) >> 4;
            let mut sens2 = 2 * (temperature - 2000) * (temperature - 2000);

            if temperature < -1500 {
                off2 += 15 * (temperature + 1500) * (temperature + 1500);
                sens2 += 8 * (temperature + 1500) * (temperature + 1500);
            }

            temperature -= t2;
            off -= off2 as i64;
            sens -= sens2 as i64;
        }

        // pressure calculation
        let pressure = (((((d1 as i64) * sens) >> 21) - off) >> 15) as i32;

        Ok(SensorReading::new(
            timestamp,
            BaroData {
                temperature: temperature as f32 / 100.0,
                pressure: pressure as f32,
            },
        ))
    }
}
