#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawData {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) z: i16,
}

impl RawData {
    pub const SIZE: usize = 6;

    pub fn x(&self) -> i16 {
        self.x
    }

    pub fn y(&self) -> i16 {
        self.y
    }

    pub fn z(&self) -> i16 {
        self.z
    }

    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }
}

impl From<[u8; Self::SIZE]> for RawData {
    fn from(value: [u8; Self::SIZE]) -> Self {
        Self {
            x: i16::from_be_bytes([value[0], value[1]]),
            y: i16::from_be_bytes([value[2], value[3]]),
            z: i16::from_be_bytes([value[4], value[5]]),
        }
    }
}
