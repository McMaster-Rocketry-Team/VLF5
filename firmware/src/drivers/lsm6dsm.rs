use embassy_time::{Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use firmware_common_new::readings::IMUData;
use firmware_common_new::sensor_reading::SensorReading;
use firmware_common_new::time::BootTimestamp;
use nalgebra::Vector3;

const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const CTRL4_C: u8 = 0x13;
const CTRL6_C: u8 = 0x15;
const DRDY_PULSE_CFG: u8 = 0x0B;
const INT1_CTRL: u8 = 0x0D;
const OUTX_L_G: u8 = 0x22;

pub struct LSM6DSM<B: SpiDevice> {
    spi: B,
}

impl<B: SpiDevice> LSM6DSM<B> {
    pub fn new(spi_device: B) -> Self {
        Self { spi: spi_device }
    }

    async fn read_register(&mut self, address: u8) -> Result<u8, B::Error> {
        let mut buffer = [0u8; 2];

        self.spi
            .transfer(&mut buffer, &[address | 0b10000000, 0x00])
            .await?;

        Ok(buffer[1])
    }

    async fn write_register(&mut self, address: u8, value: u8) -> Result<(), B::Error> {
        self.spi
            .transfer(&mut [0u8; 2], &[address & !0b10000000, value])
            .await?;
        Ok(())
    }

    pub async fn power_down(&mut self) -> Result<(), B::Error> {
        self.write_register(CTRL1_XL, 0).await?;
        self.write_register(CTRL2_G, 0).await?;

        Ok(())
    }

    pub async fn power_up(&mut self) -> Result<(), B::Error> {
        // set acc ODR to 416Hz, bandwidth 104Hz, full scale to +-16g
        self.write_register(CTRL1_XL, 0b0110_01_1_0).await?;
        // set gyro ODR to 416Hz, full scale to +-2000dps
        self.write_register(CTRL2_G, 0b0110_11_0_0).await?;
        // enable gyro filter
        self.write_register(CTRL4_C, 0b0000_0001).await?;
        // gyro bandwidth 121Hz
        self.write_register(CTRL6_C, 0b0000_0010).await?;

        // pulse int1 when data ready
        self.write_register(DRDY_PULSE_CFG, 0b1000_0000).await?;
        // enable gyro and acc data ready on int1
        self.write_register(INT1_CTRL, 0b0000_0011).await?;

        Timer::after_millis(1).await;

        Ok(())
    }

    pub async fn reset(&mut self) -> Result<bool, B::Error> {
        Timer::after_millis(20).await; // wait for initialize

        // reset
        self.write_register(CTRL3_C, 0b10000101).await?;
        Timer::after_millis(20).await; // wait for initialize

        let id = self.read_register(WHO_AM_I).await?;
        if id != 0x6A {
            return Ok(false);
        }

        Ok(true)
    }

    pub async fn read(&mut self) -> Result<SensorReading<BootTimestamp, IMUData>, B::Error> {
        let mut buffer = [0u8; 13];
        self.spi
            .transfer(
                &mut buffer,
                &[OUTX_L_G | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            )
            .await?;

        let buffer = &buffer[1..];
        let gyro_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let gyro_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let gyro_z = i16::from_le_bytes([buffer[4], buffer[5]]);

        let acc_x = i16::from_le_bytes([buffer[6], buffer[7]]);
        let acc_y = i16::from_le_bytes([buffer[8], buffer[9]]);
        let acc_z = i16::from_le_bytes([buffer[10], buffer[11]]);

        let acc_scale = 16.0 / 32768.0 * 9.81; // ±16g range
        let gyro_scale = 2000.0 / 32768.0; // ±2000dps range

        Ok(SensorReading::new(
            Instant::now().as_micros(),
            IMUData {
                acc: Vector3::new(
                    acc_x as f32 * acc_scale,
                    acc_y as f32 * acc_scale,
                    acc_z as f32 * acc_scale,
                ),
                gyro: Vector3::new(
                    gyro_x as f32 * gyro_scale,
                    gyro_y as f32 * gyro_scale,
                    gyro_z as f32 * gyro_scale,
                ),
            },
        ))
    }
}
