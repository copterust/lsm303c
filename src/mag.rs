pub const ADDRESS: u8 = 0x1E;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    MAG_WHO_AM_I = 0x0F,
    CTRL1 = 0x20,
    CTRL2 = 0x21,
    CTRL3 = 0x22,
    CTRL4 = 0x23,
    CTRL5 = 0x24,
    STATUS_REG = 0x27,
    OUTX_L = 0x28,
    OUTX_H = 0x29,
    OUTY_L = 0x2A,
    OUTY_H = 0x2B,
    OUTZ_L = 0x2C,
    OUTZ_H = 0x2D,
    TEMP_OUT_L = 0x2E,
    TEMP_OUT_H = 0x2F,
    INT_CFG = 0x30,
    INT_SRC = 0x31,
    INT_THS_L = 0x32,
    INT_THS_H = 0x33,
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}
