# `mpu-6500-async`

Async I2C driver for the TDK InvenSense MPU-6500.

## Demo project for STM32F3
Example project demonstrating the crate in action can be found
[here](https://github.com/witek103/stm32f3_embassy_async_i2c).

### Shout out to [mpu-6050-dmp](https://github.com/barafael/mpu6050-dmp-rs)
MPU-6500 and MPU-6050 are similar chips from the same manufacturer and there
are many common parts (i.e. most of the registers are the same), this crate was
heavily influenced by the mentioned one.
