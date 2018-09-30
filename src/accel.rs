pub const ADDRESS: u8 = 0x1D;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    TEMP_L = 0x0B,
    TEMP_H = 0x0C,
    ACT_TSH = 0x1E,
    ACT_DUR = 0x1F,
    WHO_AM_I = 0x0F,
    CTRL1 = 0x20,
    CTRL2 = 0x21,
    CTRL3 = 0x22,
    CTRL4 = 0x23,
    CTRL5 = 0x24,
    CTRL6 = 0x25,
    CTRL7 = 0x26,
    STATUS = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL = 0x2E,
    FIFO_SRC = 0x2F,
    IG_CFG1 = 0x30,
    IG_SRC1 = 0x31,
    IG_THS_X1 = 0x32,
    IG_THS_Y1 = 0x33,
    IG_THS_Z1 = 0x34,
    IG_DUR1 = 0x35,
    IG_CFG2 = 0x36,
    IG_SRC2 = 0x37,
    IG_THS2 = 0x38,
    IG_DUR2 = 0x39,
    XL_REFERENCE = 0x3A,
    XH_REFERENCE = 0x3B,
    YL_REFERENCE = 0x3C,
    YH_REFERENCE = 0x3D,
    ZL_REFERENCE = 0x3E,
}

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}
