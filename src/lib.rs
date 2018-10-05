//! A platform agnostic driver to interface with the LSM303C (accelerometer +
//! compass)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal

#![deny(missing_docs)]
#![deny(warnings)]
#![allow(unused)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;
extern crate nalgebra;

mod accel;
mod mag;

use core::mem;

use cast::{f32, u16};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

pub use nalgebra::Vector3;

use hal::blocking::i2c::{Write, WriteRead};

const TEMP_RESOLUTION: f32 = 0.125;
const TEMP_ZERO_OFFSET: f32 = 25.0;

/// LSM303C driver
pub struct Lsm303c<I2C> {
    i2c: I2C,
}

impl<I2C, E> Lsm303c<I2C> where I2C: WriteRead<Error = E> + Write<Error = E>
{
    /// Creates a new driver from a I2C peripheral
    pub fn new<W: core::fmt::Write>(i2c: I2C, l: &mut W) -> Result<Self, E> {
        let mut lsm303c = Lsm303c { i2c, };

        // TODO reset all the registers / the device

        write!(l, "let's write\r\n");
        // configure the accelerometer to operate at 400 Hz
        lsm303c.write_accel_register_with_mask(accel::Register::CTRL1,
                                               AccelOdr::default())?;
        write!(l, "freq done\r\n");

        // configure the magnetometer to operate in continuous mode
        lsm303c.write_mag_register_with_mask(mag::Register::CTRL3,
                                             MagMode::default())?;
        write!(l, "cont mode done\r\n");

        // enable the temperature sensor
        lsm303c.write_mag_register_with_mask(mag::Register::CTRL1,
                                             TemperatureControl::default())?;
        write!(l, "enable temp done\r\n");

        Ok(lsm303c)
    }

    /// Accelerometer measurements
    pub fn accel(&mut self) -> Result<Vector3<i16>, E> {
        let buffer: GenericArray<u8, U6> =
            self.read_accel_registers(accel::Register::OUT_X_L)?;

        Ok(Vector3::new((u16(buffer[0]) + (u16(buffer[1]) << 8)) as i16,
                        (u16(buffer[2]) + (u16(buffer[3]) << 8)) as i16,
                        (u16(buffer[4]) + (u16(buffer[5]) << 8)) as i16))
    }

    /// Sets the accelerometer output data rate
    pub fn accel_odr(&mut self, odr: AccelOdr) -> Result<(), E> {
        self.write_accel_register_with_mask(accel::Register::CTRL1, odr)
    }

    /// Magnetometer measurements
    pub fn mag(&mut self) -> Result<Vector3<i16>, E> {
        let buffer: GenericArray<u8, U6> =
            self.read_mag_registers(mag::Register::OUTX_L)?;

        Ok(Vector3::new((u16(buffer[1]) + (u16(buffer[0]) << 8)) as i16,
                        (u16(buffer[5]) + (u16(buffer[4]) << 8)) as i16,
                        (u16(buffer[3]) + (u16(buffer[2]) << 8)) as i16))
    }

    /// Sets the magnetometer output data rate
    pub fn mag_odr(&mut self, odr: MagOdr) -> Result<(), E> {
        self.write_mag_register_with_mask(mag::Register::CTRL1, odr)
    }

    /// Temperature sensor measurement in Celcius
    ///
    /// - Range: [-40, +85] C
    pub fn temp(&mut self) -> Result<f32, E> {
        let rt = self.raw_temp()?;
        Ok(f32(rt) * TEMP_RESOLUTION + TEMP_ZERO_OFFSET)
    }

    /// Raw temperature sensor measurement
    ///
    /// - Resolution: 12-bit
    pub fn raw_temp(&mut self) -> Result<i16, E> {
        let buffer = self.read_mag_registers::<U2>(mag::Register::TEMP_OUT_L)?;
        let t = ((u16(buffer[1]) << 8) | u16(buffer[0])) as i16;
        Ok(t >> 4)
    }

    /// Sets accelerometer full scalue
    pub fn set_accel_scale(&mut self,
                           accel_scale: AccelScale)
                           -> Result<(), E> {
        self.write_accel_register_with_mask(accel::Register::CTRL4, accel_scale)
    }

    fn modify_accel_register<F>(&mut self,
                                reg: accel::Register,
                                f: F)
                                -> Result<(), E>
        where F: FnOnce(u8) -> u8
    {
        let r = self.read_accel_register(reg)?;
        self.write_accel_register(reg, f(r))?;
        Ok(())
    }

    fn write_accel_register_with_mask<RB>(&mut self,
                                          reg: accel::Register,
                                          v: RB)
                                          -> Result<(), E>
        where RB: RegisterBits
    {
        self.modify_accel_register(reg, |r| (r & !RB::mask()) | v.value())
    }

    fn write_mag_register_with_mask<RB>(&mut self,
                                        reg: mag::Register,
                                        v: RB)
                                        -> Result<(), E>
        where RB: RegisterBits
    {
        self.modify_mag_register(reg, |r| (r & !RB::mask()) | v.value())
    }

    fn modify_mag_register<F>(&mut self,
                              reg: mag::Register,
                              f: F)
                              -> Result<(), E>
        where F: FnOnce(u8) -> u8
    {
        let r = self.read_mag_register(reg)?;
        self.write_mag_register(reg, f(r))?;
        Ok(())
    }

    fn read_accel_registers<N>(&mut self,
                               reg: accel::Register)
                               -> Result<GenericArray<u8, N>, E>
        where N: ArrayLength<u8>
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::uninitialized() };

        {
            let buffer: &mut [u8] = &mut buffer;

            const MULTI: u8 = 1 << 7;
            self.i2c
                .write_read(accel::ADDRESS, &[reg.addr() | MULTI], buffer)?;
        }

        Ok(buffer)
    }

    fn read_accel_register(&mut self, reg: accel::Register) -> Result<u8, E> {
        self.read_accel_registers::<U1>(reg).map(|b| b[0])
    }

    fn read_mag_register(&mut self, reg: mag::Register) -> Result<u8, E> {
        let buffer: GenericArray<u8, U1> = self.read_mag_registers(reg)?;
        Ok(buffer[0])
    }

    // NOTE has weird address increment semantics; use only with `OUT_X_H`
    fn read_mag_registers<N>(&mut self,
                             reg: mag::Register)
                             -> Result<GenericArray<u8, N>, E>
        where N: ArrayLength<u8>
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::uninitialized() };

        {
            let buffer: &mut [u8] = &mut buffer;

            self.i2c.write_read(mag::ADDRESS, &[reg.addr()], buffer)?;
        }

        Ok(buffer)
    }

    fn write_accel_register(&mut self,
                            reg: accel::Register,
                            byte: u8)
                            -> Result<(), E> {
        self.i2c.write(accel::ADDRESS, &[reg.addr(), byte])
    }

    fn write_mag_register(&mut self,
                          reg: mag::Register,
                          byte: u8)
                          -> Result<(), E> {
        self.i2c.write(mag::ADDRESS, &[reg.addr(), byte])
    }
}

trait RegisterBits {
    fn mask() -> u8;
    fn value(&self) -> u8;
}

/// Accelerometer Output Data Rate
#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum AccelOdr {
    /// 10 Hz
    _10_Hz = 0x10,
    /// 50 Hz
    _50_Hz = 0x20,
    /// 100 Hz
    _100_Hz = 0x30,
    /// 200 Hz
    _200_Hz = 0x40,
    /// 400 Hz
    _400_Hz = 0x50,
    /// 800 Hz
    _800_Hz = 0x60,
}

impl Default for AccelOdr {
    fn default() -> Self {
        AccelOdr::_100_Hz
    }
}

impl RegisterBits for AccelOdr {
    fn mask() -> u8 {
        AccelOdr::_800_Hz.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

/// Acceleration scale/sensitivity
#[derive(Clone, Copy)]
pub enum AccelScale {
    /// +/- 2g
    _2G,
    /// +/- 4g
    _4G,
    /// +/- 8g
    _8G,
}

impl Default for AccelScale {
    fn default() -> Self {
        AccelScale::_2G
    }
}

impl RegisterBits for AccelScale {
    fn mask() -> u8 {
        AccelScale::_8G.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

impl AccelScale {
    fn resolution(&self) -> f32 {
        match self {
            AccelScale::_2G => 2.0 / 32768.0,
            AccelScale::_4G => 4.0 / 32768.0,
            AccelScale::_8G => 8.0 / 32768.0,
        }
    }
}

/// Magnetometer Output Data Rate
#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum MagOdr {
    /// 0.625Hz
    _0_625_Hz = 0x00,
    /// 1.25Hz
    _1_25_Hz = 0x04,
    /// 2.5Hz
    _2_5_Hz = 0x08,
    /// 5Hz
    _5_Hz = 0x0C,
    /// 10Hz
    _10_Hz = 0x10,
    /// 20Hz
    _20_Hz = 0x14,
    /// 40Hz
    _40_Hz = 0x18,
    /// 80Hz
    _80_Hz = 0x1C,
}

impl Default for MagOdr {
    fn default() -> Self {
        MagOdr::_40_Hz
    }
}

impl RegisterBits for MagOdr {
    fn mask() -> u8 {
        MagOdr::_80_Hz.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

/// Magnetometer full scale
#[derive(Debug, Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum MagScale {
    /// +/- 4 gauss
    _4_Ga = 0x00,
    /// +/- 8 gauss
    _8_Ga = 0x20,
    /// +/- 12 gauss
    _12_Ga = 0x40,
    /// +/- 16 gauss
    _16_Ga = 0x60,
}
// LSM303C only supports 16 ga
// mgauss/LSB

impl Default for MagScale {
    fn default() -> Self {
        MagScale::_16_Ga
    }
}

impl RegisterBits for MagScale {
    fn mask() -> u8 {
        MagScale::_16_Ga.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

impl MagScale {
    fn resolution(&self) -> f32 {
        match self {
            MagScale::_4_Ga => 4.0 / 32768.0,
            MagScale::_8_Ga => 8.0 / 32768.0,
            MagScale::_12_Ga => 12.0 / 32768.0,
            MagScale::_16_Ga => 16.0 / 32768.0,
        }
    }
}

/// Magnetrometer mode
#[derive(Debug, Clone, Copy)]
pub enum MagMode {
    /// Continous mode
    Continous = 0x00,
    /// Single
    Single = 0x01,
    /// Pwr Down 1
    PowerDown1 = 0x02,
    /// Pwr Down 2
    PowerDown2 = 0x03,
}

impl RegisterBits for MagMode {
    fn mask() -> u8 {
        MagMode::PowerDown2.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Default for MagMode {
    fn default() -> Self {
        MagMode::Continous
    }
}

#[derive(Debug, Clone, Copy)]
enum TemperatureControl {
    Enable = 0x80,
    Disable = 0x00,
}

impl Default for TemperatureControl {
    fn default() -> Self {
        TemperatureControl::Enable
    }
}

impl RegisterBits for TemperatureControl {
    fn mask() -> u8 {
        TemperatureControl::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}
