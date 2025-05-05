use crate::sleep;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::Error;
use embedded_hal_async::spi::{ErrorKind, SpiDevice};
use firmware_common_new::readings::BaroData;
use firmware_common_new::sensor_reading::SensorReading;
use firmware_common_new::time::BootTimestamp;

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
        }
    }

    pub async fn reset(&mut self) -> Result<(), ErrorKind> {
        let (read_buffer, write_buffer) = create_buffer!(self, [0x1E]);
        // reset
        self.spi
            .transfer(read_buffer, write_buffer)
            .await
            .map_err(|e| e.kind())?;

        sleep!(20);

        // read coefficients
        let mut coefficients = [0u16; 6];
        for addr in 1..=6 {
            let (read_buffer, write_buffer) = create_buffer!(self, [0xA0 | (addr << 1), 0, 0]);
            self.spi
                .transfer(read_buffer, write_buffer)
                .await
                .map_err(|e| e.kind())?;

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

        Ok(())
    }

    pub async fn read(&mut self) -> Result<SensorReading<BootTimestamp, BaroData>, ErrorKind> {
        // request measurement pressure with OSR=1024
        let timestamp = Instant::now().as_micros() as f64 / 1000.0 + 1.0; // timestamp of the pressure measurement
        let (read_buffer, write_buffer) = create_buffer!(self, [0x44]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await
            .map_err(|e| e.kind())?;
        Timer::after(Duration::from_micros(2280)).await;

        // read pressure measurement
        let (read_buffer, write_buffer) = create_buffer!(self, [0x00, 0, 0, 0]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await
            .map_err(|e| e.kind())?;
        let d1 = ((read_buffer[1] as u32) << 16)
            | ((read_buffer[2] as u32) << 8)
            | (read_buffer[3] as u32);

        // request measurement temperature with OSR=256
        let (read_buffer, write_buffer) = create_buffer!(self, [0x50]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await
            .map_err(|e| e.kind())?;
        Timer::after(Duration::from_micros(600)).await;

        // read temerature measurement
        let (read_buffer, write_buffer) = create_buffer!(self, [0x00, 0, 0, 0]);
        self.spi
            .transfer(read_buffer, write_buffer)
            .await
            .map_err(|e| e.kind())?;
        let d2 = ((read_buffer[1] as u32) << 16)
            | ((read_buffer[2] as u32) << 8)
            | (read_buffer[3] as u32);

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
