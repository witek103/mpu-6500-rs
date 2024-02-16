#![no_std]

pub mod raw_data;
pub mod register_map;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use raw_data::RawData;
use register_map::{AccelFullScaleRate, DigitalLowPassFilter, GyroFullScaleRate, RegisterMap};

const I2C_ADDR_AD0_LOW: SevenBitAddress = 0b1101000;
const I2C_ADDR_AD0_HIGH: SevenBitAddress = 0b1101001;

const WHO_AM_I: u8 = 0x70;

pub struct Mpu6500<T> {
    dev: T,
    address: SevenBitAddress,
}

impl<T> Mpu6500<T>
where
    T: I2c,
{
    /// Use driver with default I2C address (AD0 line low)
    pub fn new(dev: T) -> Self {
        Self {
            dev,
            address: I2C_ADDR_AD0_LOW,
        }
    }

    /// AD0 line is high, adjust device I2C address accordingly
    pub fn with_ad0_line_high(self) -> Self {
        Self {
            dev: self.dev,
            address: I2C_ADDR_AD0_HIGH,
        }
    }

    /// Check `WHO_AM_I` response
    pub async fn detected(&mut self) -> Result<bool, T::Error> {
        let value = self.read_register(RegisterMap::WhoAmI).await?;

        Ok(value == WHO_AM_I)
    }

    /// Initialize IMU with reasonable default settings
    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), T::Error> {
        self.reset(delay).await?;
        self.wake_up().await?;
        self.set_gyro_full_scale(GyroFullScaleRate::Dps2000).await?;
        self.set_accel_full_scale(AccelFullScaleRate::G2).await?;
        self.set_gyro_digital_low_pass_filter(DigitalLowPassFilter::Filter1)
            .await?;
        self.set_accel_digital_low_pass_filter(DigitalLowPassFilter::Filter1)
            .await?;
        self.set_sample_rate_divider(4).await?;
        self.disable_interrupts().await?;
        self.configure_interrupt_pin().await?;
        self.set_clock_source(delay).await?;

        #[cfg(feature = "defmt")]
        defmt::trace!("MPU6500 initialized");

        Ok(())
    }

    pub async fn read_register(&mut self, register: RegisterMap) -> Result<u8, T::Error> {
        let mut buf = [0; 1];

        self.dev
            .write_read(self.address, &[register as u8], &mut buf)
            .await?;

        Ok(buf[0])
    }

    pub async fn write_register(
        &mut self,
        register: RegisterMap,
        value: u8,
    ) -> Result<(), T::Error> {
        self.dev.write(self.address, &[register as u8, value]).await
    }

    /// Reset routine
    pub async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), T::Error> {
        self.write_register(RegisterMap::PwrMgmt1, 1 << 7).await?;

        delay.delay_ms(100).await;

        self.write_register(RegisterMap::SignalPathReset, 0b111)
            .await?;

        delay.delay_ms(100).await;

        Ok(())
    }

    pub async fn wake_up(&mut self) -> Result<(), T::Error> {
        self.write_register(RegisterMap::PwrMgmt1, 0).await
    }

    pub async fn set_gyro_full_scale(&mut self, scale: GyroFullScaleRate) -> Result<(), T::Error> {
        let value = self.read_register(RegisterMap::GyroConfig).await?;

        let value = (value & !(0b11 << 3)) | ((scale as u8) << 3);

        self.write_register(RegisterMap::GyroConfig, value).await
    }

    pub async fn set_accel_full_scale(
        &mut self,
        scale: AccelFullScaleRate,
    ) -> Result<(), T::Error> {
        let value = self.read_register(RegisterMap::AccelConfig).await?;

        let value = (value & !(0b11 << 3)) | ((scale as u8) << 3);

        self.write_register(RegisterMap::AccelConfig, value).await
    }

    pub async fn set_gyro_digital_low_pass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), T::Error> {
        let value = self.read_register(RegisterMap::Config).await?;

        let value = (value & !0b111) | (filter as u8);

        self.write_register(RegisterMap::Config, value).await
    }

    pub async fn set_accel_digital_low_pass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), T::Error> {
        let value = self.read_register(RegisterMap::AccelConfig2).await?;

        let value = (value & !0b111) | (filter as u8);

        self.write_register(RegisterMap::AccelConfig2, value).await
    }

    /// Divider is working only when `Fs = 1kHz`, so to set desired sampling rate:
    ///
    /// `divider = 1000 / SAMPLE_RATE_HZ - 1`
    ///
    /// Sampling rate must be between 4Hz and 1kHz.
    pub async fn set_sample_rate_divider(&mut self, divider: u8) -> Result<(), T::Error> {
        self.write_register(RegisterMap::SmplRtDiv, divider).await
    }

    pub async fn disable_interrupts(&mut self) -> Result<(), T::Error> {
        self.write_register(RegisterMap::IntEnable, 0).await
    }

    /// Drive interrupt pit when new measurement data is ready
    pub async fn enable_data_ready_interrupt(&mut self) -> Result<(), T::Error> {
        self.write_register(RegisterMap::IntEnable, 1).await
    }

    /// Interrupt pin active low, latched, cleared on any measurement data read
    pub async fn configure_interrupt_pin(&mut self) -> Result<(), T::Error> {
        self.write_register(RegisterMap::IntPinCfg, 1 << 7 | 1 << 4 | 1 << 5)
            .await
    }

    /// Auto select best clock source - tries PLL, then internal oscillator
    pub async fn set_clock_source(&mut self, delay: &mut impl DelayNs) -> Result<(), T::Error> {
        self.write_register(RegisterMap::PwrMgmt1, 1).await?;

        delay.delay_ms(50).await;

        Ok(())
    }

    /// Read raw accelerometer measurement data
    pub async fn read_accel_data(&mut self) -> Result<RawData, T::Error> {
        let mut data = [0; RawData::SIZE];

        self.dev
            .write_read(self.address, &[RegisterMap::AccelXOutH as u8], &mut data)
            .await?;

        Ok(data.into())
    }

    /// Read raw gyroscope measurement data
    pub async fn read_gyro_data(&mut self) -> Result<RawData, T::Error> {
        let mut data = [0; RawData::SIZE];

        self.dev
            .write_read(self.address, &[RegisterMap::GyroXOutH as u8], &mut data)
            .await?;

        Ok(data.into())
    }
}
