#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegisterMap {
    XgOffsetH = 0x13,
    XgOffsetL = 0x14,
    YgOffsetH = 0x15,
    YgOffsetL = 0x16,
    ZgOffsetH = 0x17,
    ZgOffsetL = 0x18,
    SmplRtDiv = 0x19,
    Config = 0x1A,
    GyroConfig = 0x1B,
    AccelConfig = 0x1C,
    AccelConfig2 = 0x1D,
    FifoEn = 0x23,
    I2CMstCtrl = 0x24,
    IntPinCfg = 0x37,
    IntEnable = 0x38,
    AccelXOutH = 0x3B,
    GyroXOutH = 0x43,
    SignalPathReset = 0x68,
    UserCtrl = 0x6A,
    PwrMgmt1 = 0x6B,
    PwrMgmt2 = 0x6C,
    FifoCountH = 0x72,
    FifoRW = 0x74,
    WhoAmI = 0x75,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroFullScaleRate {
    Dps250 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFullScaleRate {
    G2 = 0b00,
    G4 = 0b01,
    G8 = 0b10,
    G16 = 0b11,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DigitalLowPassFilter {
    Filter0 = 0,
    Filter1 = 1,
    Filter2 = 2,
    Filter3 = 3,
    Filter4 = 4,
    Filter5 = 5,
    Filter6 = 6,
    Filter7 = 7,
}
