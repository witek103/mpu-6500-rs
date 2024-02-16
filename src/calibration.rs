use core::ops::AddAssign;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::{DigitalLowPassFilter, GyroFullScaleRate, Mpu6500, RawData, RegisterMap};

struct CalibrationData {
    x: i32,
    y: i32,
    z: i32,
}

impl CalibrationData {
    const SAMPLE_COUNT: usize = 200;

    fn new() -> Self {
        Self { x: 0, y: 0, z: 0 }
    }

    /// Calculate bias based on gathered data.
    /// Currently, it approximates with mean values.
    fn estimate_bias(&self) -> RawData {
        let x = (self.x / Self::SAMPLE_COUNT as i32) as i16;
        let y = (self.y / Self::SAMPLE_COUNT as i32) as i16;
        let z = (self.z / Self::SAMPLE_COUNT as i32) as i16;

        RawData::new(x, y, z)
    }
}

impl AddAssign<RawData> for CalibrationData {
    fn add_assign(&mut self, rhs: RawData) {
        self.x += rhs.x() as i32;
        self.y += rhs.y() as i32;
        self.z += rhs.z() as i32;
    }
}

/// Apply calculated bias to gyro offset registers.
/// This should be done after initialization.
pub async fn set_gyro_offset<T>(dev: &mut Mpu6500<T>, bias: RawData) -> Result<(), T::Error>
where
    T: I2c,
{
    #[cfg(feature = "defmt")]
    defmt::trace!("Apply given bias: {:?} to gyro offset", bias);

    let x = (-bias.x).to_be_bytes();

    dev.write_register(RegisterMap::XgOffsetH, x[0]).await?;
    dev.write_register(RegisterMap::XgOffsetL, x[1]).await?;

    let y = (-bias.y).to_be_bytes();

    dev.write_register(RegisterMap::YgOffsetH, y[0]).await?;
    dev.write_register(RegisterMap::YgOffsetL, y[1]).await?;

    let z = (-bias.z).to_be_bytes();

    dev.write_register(RegisterMap::ZgOffsetH, z[0]).await?;
    dev.write_register(RegisterMap::ZgOffsetL, z[1]).await
}

/// Calculate gyro bias.
pub async fn get_gyro_bias<T>(
    dev: &mut Mpu6500<T>,
    delay: &mut impl DelayNs,
) -> Result<RawData, T::Error>
where
    T: I2c,
{
    #[cfg(feature = "defmt")]
    defmt::trace!("Gyro calibration started");

    reset(dev, delay).await?;
    prepare_gyro_readout(dev, delay).await?;

    let mut calibration_data = CalibrationData::new();

    let mut sample_index = 0;

    while sample_index < CalibrationData::SAMPLE_COUNT {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "Processing samples [{}/{}]",
            sample_index,
            CalibrationData::SAMPLE_COUNT
        );

        delay.delay_ms(10).await;

        let ready_samples_count = get_ready_samples_count(dev, sample_index).await?;

        process_ready_samples(dev, ready_samples_count, &mut calibration_data).await?;

        sample_index += ready_samples_count;
    }

    #[cfg(feature = "defmt")]
    defmt::trace!("Processed {} samples", sample_index);

    let bias = calibration_data.estimate_bias();

    #[cfg(feature = "defmt")]
    defmt::trace!("Estimated bias: {:?}", bias);

    Ok(bias)
}

async fn reset<T>(dev: &mut Mpu6500<T>, delay: &mut impl DelayNs) -> Result<(), T::Error>
where
    T: I2c,
{
    // initialize
    dev.write_register(RegisterMap::PwrMgmt1, 1).await?;
    dev.write_register(RegisterMap::PwrMgmt2, 0).await?;

    delay.delay_ms(200).await;
    // reset settings
    dev.write_register(RegisterMap::XgOffsetH, 0).await?;
    dev.write_register(RegisterMap::XgOffsetL, 0).await?;
    dev.write_register(RegisterMap::YgOffsetH, 0).await?;
    dev.write_register(RegisterMap::YgOffsetL, 0).await?;
    dev.write_register(RegisterMap::ZgOffsetH, 0).await?;
    dev.write_register(RegisterMap::ZgOffsetL, 0).await?;
    dev.disable_interrupts().await?;
    dev.write_register(RegisterMap::FifoEn, 0).await?;
    dev.write_register(RegisterMap::PwrMgmt1, 0).await?;
    dev.write_register(RegisterMap::I2CMstCtrl, 0).await?;
    dev.write_register(RegisterMap::UserCtrl, 0).await?;
    // reset DMP and FIFO
    dev.write_register(RegisterMap::UserCtrl, 1 << 2 | 1 << 3)
        .await?;

    delay.delay_ms(15).await;

    Ok(())
}

async fn prepare_gyro_readout<T>(
    dev: &mut Mpu6500<T>,
    delay: &mut impl DelayNs,
) -> Result<(), T::Error>
where
    T: I2c,
{
    // setup parameters
    dev.set_gyro_digital_low_pass_filter(DigitalLowPassFilter::Filter2)
        .await?;
    dev.set_sample_rate_divider(0).await?;
    dev.set_gyro_full_scale(GyroFullScaleRate::Dps1000).await?;
    // wait for sensors to stabilize
    delay.delay_ms(200).await;
    // enable fifo for gyro XYZ readout
    dev.write_register(RegisterMap::UserCtrl, 1 << 6).await?;
    dev.write_register(RegisterMap::FifoEn, 0b111 << 4).await
}

async fn get_ready_samples_count<T>(
    dev: &mut Mpu6500<T>,
    sample_index: usize,
) -> Result<usize, T::Error>
where
    T: I2c,
{
    let mut data = [0; 2];

    dev.read_registers(RegisterMap::FifoCountH, &mut data)
        .await?;

    let fifo_count = u16::from_be_bytes([data[0], data[1]]) as usize;

    let ready_samples_count = match fifo_count / RawData::SIZE {
        s if CalibrationData::SAMPLE_COUNT - sample_index < s => {
            CalibrationData::SAMPLE_COUNT - sample_index
        }
        s => s,
    };

    Ok(ready_samples_count)
}

async fn process_ready_samples<T>(
    dev: &mut Mpu6500<T>,
    ready_samples_count: usize,
    calibration_data: &mut CalibrationData,
) -> Result<(), T::Error>
where
    T: I2c,
{
    let mut data = [0; 512];

    dev.read_registers(
        RegisterMap::FifoRW,
        &mut data[0..ready_samples_count * RawData::SIZE],
    )
    .await?;

    for si in 0..ready_samples_count {
        let sample_index = si * RawData::SIZE;

        let raw_sample: [u8; RawData::SIZE] = (&data[sample_index..sample_index + RawData::SIZE])
            .try_into()
            .unwrap();

        *calibration_data += raw_sample.into();
    }

    Ok(())
}
