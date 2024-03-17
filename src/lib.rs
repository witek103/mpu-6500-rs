#![no_std]

pub mod calibration;
pub mod raw_data;
pub mod register_map;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{SevenBitAddress, Write, WriteRead};
use raw_data::RawData;
use register_map::{AccelFullScaleRate, DigitalLowPassFilter, GyroFullScaleRate, RegisterMap};

const I2C_ADDR_AD0_LOW: SevenBitAddress = 0b1101000;
const I2C_ADDR_AD0_HIGH: SevenBitAddress = 0b1101001;

const WHO_AM_I: u8 = 0x70;

pub struct Mpu6500<T> {
    dev: T,
    address: SevenBitAddress,
}

impl<T, E> Mpu6500<T>
where
    T: WriteRead<Error = E> + Write<Error = E>,
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
    pub fn detected(&mut self) -> Result<bool, E> {
        let value = self.read_register(RegisterMap::WhoAmI)?;

        Ok(value == WHO_AM_I)
    }

    /// Initialize IMU with reasonable default settings
    pub fn init(&mut self, delay: &mut impl DelayMs<u32>) -> Result<(), E> {
        self.reset(delay)?;
        self.wake_up()?;
        self.set_gyro_full_scale(GyroFullScaleRate::Dps2000)?;
        self.set_accel_full_scale(AccelFullScaleRate::G2)?;
        self.set_gyro_digital_low_pass_filter(DigitalLowPassFilter::Filter1)?;
        self.set_accel_digital_low_pass_filter(DigitalLowPassFilter::Filter1)?;
        self.set_sample_rate_divider(4)?;
        self.disable_interrupts()?;
        self.configure_interrupt_pin()?;
        self.set_clock_source(delay)?;

        #[cfg(feature = "defmt")]
        defmt::trace!("MPU6500 initialized");

        Ok(())
    }

    pub fn read_register(&mut self, register: RegisterMap) -> Result<u8, E> {
        let mut buf = [0; 1];

        self.dev
            .write_read(self.address, &[register as u8], &mut buf)?;

        Ok(buf[0])
    }

    pub fn read_registers(&mut self, register: RegisterMap, buf: &mut [u8]) -> Result<(), E> {
        self.dev.write_read(self.address, &[register as u8], buf)?;

        Ok(())
    }

    pub fn write_register(&mut self, register: RegisterMap, value: u8) -> Result<(), E> {
        self.dev.write(self.address, &[register as u8, value])
    }

    /// Reset routine
    pub fn reset(&mut self, delay: &mut impl DelayMs<u32>) -> Result<(), E> {
        self.write_register(RegisterMap::PwrMgmt1, 1 << 7)?;

        delay.delay_ms(100);

        self.write_register(RegisterMap::SignalPathReset, 0b111)?;

        delay.delay_ms(100);

        Ok(())
    }

    pub fn wake_up(&mut self) -> Result<(), E> {
        self.write_register(RegisterMap::PwrMgmt1, 0)
    }

    pub fn set_gyro_full_scale(&mut self, scale: GyroFullScaleRate) -> Result<(), E> {
        let value = self.read_register(RegisterMap::GyroConfig)?;

        let value = (value & !(0b11 << 3)) | ((scale as u8) << 3);

        self.write_register(RegisterMap::GyroConfig, value)
    }

    pub fn set_accel_full_scale(&mut self, scale: AccelFullScaleRate) -> Result<(), E> {
        let value = self.read_register(RegisterMap::AccelConfig)?;

        let value = (value & !(0b11 << 3)) | ((scale as u8) << 3);

        self.write_register(RegisterMap::AccelConfig, value)
    }

    pub fn set_gyro_digital_low_pass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), E> {
        let value = self.read_register(RegisterMap::Config)?;

        let value = (value & !0b111) | (filter as u8);

        self.write_register(RegisterMap::Config, value)
    }

    pub fn set_accel_digital_low_pass_filter(
        &mut self,
        filter: DigitalLowPassFilter,
    ) -> Result<(), E> {
        let value = self.read_register(RegisterMap::AccelConfig2)?;

        let value = (value & !0b111) | (filter as u8);

        self.write_register(RegisterMap::AccelConfig2, value)
    }

    /// Divider is working only when `Fs = 1kHz`, so to set desired sampling rate:
    ///
    /// `divider = 1000 / SAMPLE_RATE_HZ - 1`
    ///
    /// Sampling rate must be between 4Hz and 1kHz.
    pub fn set_sample_rate_divider(&mut self, divider: u8) -> Result<(), E> {
        self.write_register(RegisterMap::SmplRtDiv, divider)
    }

    pub fn disable_interrupts(&mut self) -> Result<(), E> {
        self.write_register(RegisterMap::IntEnable, 0)
    }

    /// Drive interrupt pit when new measurement data is ready
    pub fn enable_data_ready_interrupt(&mut self) -> Result<(), E> {
        self.write_register(RegisterMap::IntEnable, 1)
    }

    /// Interrupt pin active low, latched, cleared on any measurement data read
    pub fn configure_interrupt_pin(&mut self) -> Result<(), E> {
        self.write_register(RegisterMap::IntPinCfg, 1 << 7 | 1 << 4 | 1 << 5)
    }

    /// Auto select best clock source - tries PLL, then internal oscillator
    pub fn set_clock_source(&mut self, delay: &mut impl DelayMs<u32>) -> Result<(), E> {
        self.write_register(RegisterMap::PwrMgmt1, 1)?;

        delay.delay_ms(50);

        Ok(())
    }

    /// Read raw accelerometer measurement data
    pub fn read_accel_data(&mut self) -> Result<RawData, E> {
        let mut data = [0; RawData::SIZE];

        self.dev
            .write_read(self.address, &[RegisterMap::AccelXOutH as u8], &mut data)?;

        Ok(data.into())
    }

    /// Read raw gyroscope measurement data
    pub fn read_gyro_data(&mut self) -> Result<RawData, E> {
        let mut data = [0; RawData::SIZE];

        self.dev
            .write_read(self.address, &[RegisterMap::GyroXOutH as u8], &mut data)?;

        Ok(data.into())
    }
}
