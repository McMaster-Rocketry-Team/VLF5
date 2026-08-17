use embassy_time::{Instant, Timer};
use embedded_hal_async::i2c::I2c;
use firmware_common_new::{readings::MagData, sensor_reading::SensorReading, time::BootTimestamp};
use nalgebra::Vector3;

const ADDRESS: u8 = 0b0011110;
const WHO_AM_I: u8 = 0x4F;
const CFG_REG_A: u8 = 0x60;
const CFG_REG_B: u8 = 0x61;
const CFG_REG_C: u8 = 0x62;
const OUTX_L_REG: u8 = 0x68;

pub struct LIS2MDL<B: I2c> {
    i2c: B,
}

impl<B: I2c> LIS2MDL<B> {
    pub fn new(i2c: B) -> Self {
        Self { i2c }
    }

    async fn read_register(&mut self, address: u8) -> Result<u8, B::Error> {
        let mut data = [0u8; 1];
        self.i2c.write_read(ADDRESS, &[address], &mut data).await?;
        Ok(data[0])
    }

    async fn write_register(&mut self, address: u8, value: u8) -> Result<(), B::Error> {
        self.i2c.write(ADDRESS, &[address, value]).await?;
        Ok(())
    }

    pub async fn power_up(&mut self) -> Result<(), B::Error> {
        // 100Hz ODR
        self.write_register(CFG_REG_A, 0b1000_11_00).await?;

        // enable offset cancellation
        self.write_register(CFG_REG_B, 0b0000_0010).await?;

        // enable BDU (block data update)
        self.write_register(CFG_REG_C, 0b0001_0000).await?;

        Ok(())
    }

    pub async fn power_down(&mut self) -> Result<(), B::Error> {
        self.write_register(CFG_REG_A, 0b1000_11_11).await?;

        Ok(())
    }

    /// Reset the part and confirm it is really there.
    ///
    /// `Ok(false)` means the I2C transfers were acknowledged but `WHO_AM_I`
    /// did not read back this part's ID. Callers MUST treat `Ok(false)` as
    /// fatal for this sensor rather than carrying on with a device that is not
    /// the LIS2MDL.
    pub async fn reset(&mut self) -> Result<bool, B::Error> {
        Timer::after_millis(15).await; // wait for initialize

        // reset
        self.write_register(CFG_REG_A, 0b01000000).await?;
        Timer::after_millis(15).await; // wait for initialize

        let id = self.read_register(WHO_AM_I).await?;
        if id != 0b01000000 {
            return Ok(false);
        }

        Ok(true)
    }

    pub async fn read(&mut self) -> Result<SensorReading<BootTimestamp, MagData>, B::Error> {
        let now = Instant::now().as_micros();
        let mut data = [0u8; 6];
        self.i2c
            .write_read(ADDRESS, &[OUTX_L_REG], &mut data)
            .await?;

        let mag_x = i16::from_le_bytes([data[0], data[1]]);
        let mag_y = i16::from_le_bytes([data[2], data[3]]);
        let mag_z = i16::from_le_bytes([data[4], data[5]]);

        let scale = 49.152f32 / 32768.0;

        Ok(SensorReading::new(
            now,
            MagData {
                mag: Vector3::new(
                    mag_x as f32 * scale,
                    mag_y as f32 * scale,
                    mag_z as f32 * scale,
                ),
            },
        ))
    }
}
