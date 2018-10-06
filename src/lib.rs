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
    // connections
    i2c: I2C,
    // configuration
    mag_scale: MagScale,
    accel_scale: AccelScale,
    accel_odr: AccelOdr,
    accel_bdu: AccelBdu,
    mag_odr: MagOdr,
    mag_mode: MagMode,
    mag_bdu: MagBdu,
    temp_control: TempControl,
}

impl<I2C, E> Lsm303c<I2C> where I2C: WriteRead<Error = E> + Write<Error = E>
{
    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// default [`Accel scale`], [`Mag scale`], [`AccelOdr`],
    /// [`MagOdr`], [`MagMode`], [`AccelBdu`], [`MagBdu`],
    /// and [`TempControl`].
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`AccelOdr`]: ./enum.AccelOdr.html
    /// [`MagOdr`]: ./enum.MagOdr.html
    /// [`MagMode`]: ./enum.MagMode.html
    /// [`MagBdu`]: ./enum.MagBdu.html
    /// [`AccelBdu`]: ./enum.AccelBdu.html
    /// [`TempControl`]: ./enum.TempControl.html
    pub fn default(i2c: I2C) -> Result<Self, E> {
        Lsm303c::new(i2c, None, None, None, None, None, None, None, None)
    }

    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// optionally provided [`Accel scale`], [`Mag scale`],
    /// [`AccelOdr`], [`MagOdr`], [`AccelBdu`], [`MagBdu`],
    /// [`MagMode`], and [`TempControl`].
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    /// [`Mag scale`]: ./enum.MagScale.html
    /// [`AccelOdr`]: ./enum.AccelOdr.html
    /// [`MagOdr`]: ./enum.MagOdr.html
    /// [`AccelBdu`]: ./enum.AccelBdu.html
    /// [`MagBdu`]: ./enum.MagBdu.html
    /// [`MagMode`]: ./enum.MagMode.html
    /// [`TempControl`]: ./enum.TempControl.html
    pub fn new(i2c: I2C,
               accel_scale: Option<AccelScale>,
               mag_scale: Option<MagScale>,
               accel_odr: Option<AccelOdr>,
               mag_odr: Option<MagOdr>,
               accel_bdu: Option<AccelBdu>,
               mag_bdu: Option<MagBdu>,
               mag_mode: Option<MagMode>,
               temp_control: Option<TempControl>)
               -> Result<Self, E> {
        let mut lsm303c =
            Lsm303c { i2c,
                      accel_scale: accel_scale.unwrap_or_default(),
                      mag_scale: mag_scale.unwrap_or_default(),
                      accel_odr: accel_odr.unwrap_or_default(),
                      mag_odr: mag_odr.unwrap_or_default(),
                      accel_bdu: accel_bdu.unwrap_or_default(),
                      mag_bdu: mag_bdu.unwrap_or_default(),
                      mag_mode: mag_mode.unwrap_or_default(),
                      temp_control: temp_control.unwrap_or_default(), };
        lsm303c.init_lsm()?;

        Ok(lsm303c)
    }

    fn init_lsm(&mut self) -> Result<(), E> {
        // TODO reset all the registers / the device
        self._mag_odr()?;
        self._mag_scale()?;
        self._mag_block_data_update()?;
        // MAG_XYZ_AxOperativeMode
        self._mag_mode()?;

        self._accel_scale()?;
        self._accel_block_data_update()?;
        // _accel_enable_axis()?;
        self._accel_odr()?;

        self._temp_control()?;

        Ok(())
    }

    /// Reads and returns unscaled accelerometer measurements (LSB).
    pub fn unscaled_accel(&mut self) -> Result<Vector3<i16>, E> {
        let buffer: GenericArray<u8, U6> =
            self.read_accel_registers(accel::Register::OUT_X_L)?;

        Ok(self.to_vector(buffer, 0))
    }

    /// Raw magnetometer measurements
    pub fn unscaled_mag(&mut self) -> Result<Vector3<i16>, E> {
        let buffer: GenericArray<u8, U6> =
            self.read_mag_registers(mag::Register::OUTX_L)?;

        Ok(self.to_vector(buffer, 0))
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

    /// Sets the accelerometer output data rate
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    pub fn accel_odr(&mut self, odr: AccelOdr) -> Result<(), E> {
        self.accel_odr = odr;
        self._accel_odr()
    }

    fn _accel_odr(&mut self) -> Result<(), E> {
        let ao = self.accel_odr;
        self.write_accel_register_with_mask(accel::Register::CTRL1, ao)
    }

    /// Sets the magnetometer output data rate  ([`Mag Odr`]).
    ///
    /// [`Mag Odr`]: ./enum.MagOdr.html
    pub fn mag_odr(&mut self, odr: MagOdr) -> Result<(), E> {
        self.mag_odr = odr;
        self._mag_odr()
    }

    fn _mag_odr(&mut self) -> Result<(), E> {
        let mo = self.mag_odr;
        self.write_mag_register_with_mask(mag::Register::CTRL1, mo)
    }

    /// Sets accelerometer full reading scale ([`Accel scale`]).
    ///
    /// [`Accel scale`]: ./enum.AccelScale.html
    pub fn accel_scale(&mut self, accel_scale: AccelScale) -> Result<(), E> {
        self.accel_scale = accel_scale;
        self._accel_scale()
    }

    fn _accel_scale(&mut self) -> Result<(), E> {
        let asc = self.accel_scale;
        self.write_accel_register_with_mask(accel::Register::CTRL4, asc)
    }

    /// Sets accelerometer block data update ([`Accel Bdu`]).
    ///
    /// [`Accel Bdu`]: ./enum.AccelBdu.html
    pub fn accel_block_data_update(&mut self, bdu: AccelBdu) -> Result<(), E> {
        self.accel_bdu = bdu;
        self._accel_block_data_update()
    }

    fn _accel_block_data_update(&mut self) -> Result<(), E> {
        let ab = self.accel_bdu;
        self.write_accel_register_with_mask(accel::Register::CTRL1, ab)
    }

    /// Sets temperature control ([`Temp control`]).
    ///
    /// [`Temp control`]: ./enum.TempControl.html
    pub fn temp_control(&mut self, control: TempControl) -> Result<(), E> {
        self.temp_control = control;
        self._temp_control()
    }

    fn _temp_control(&mut self) -> Result<(), E> {
        let tc = self.temp_control;
        self.write_mag_register_with_mask(mag::Register::CTRL1, tc)
    }

    /// Sets magnetometer mode ([`Mag mode`]).
    ///
    /// [`Mag mode`]: ./enum.MagMode.html
    pub fn mag_mode(&mut self, mode: MagMode) -> Result<(), E> {
        self.mag_mode = mode;
        self._mag_mode()
    }

    fn _mag_mode(&mut self) -> Result<(), E> {
        let mm = self.mag_mode;
        self.write_mag_register_with_mask(mag::Register::CTRL3, mm)
    }

    /// Sets magnetometer full reading scale ([`Mag scale`]).
    ///
    /// [`Mag scale`]: ./enum.MagScale.html
    pub fn mag_scale(&mut self, mag_scale: MagScale) -> Result<(), E> {
        self.mag_scale = mag_scale;
        self._mag_scale()
    }

    fn _mag_scale(&mut self) -> Result<(), E> {
        let ms = self.mag_scale;
        self.write_mag_register_with_mask(mag::Register::CTRL2, ms)
    }

    /// Sets magnetometer block data update ([`Mag Bdu`]).
    ///
    /// [`Mag Bdu`]: ./enum.MagBdu.html
    pub fn mag_block_data_update(&mut self, bdu: MagBdu) -> Result<(), E> {
        self.mag_bdu = bdu;
        self._mag_block_data_update()
    }

    fn _mag_block_data_update(&mut self) -> Result<(), E> {
        let mb = self.mag_bdu;
        self.write_mag_register_with_mask(mag::Register::CTRL5, mb)
    }

    fn to_vector<N>(&self,
                    buffer: GenericArray<u8, N>,
                    offset: usize)
                    -> Vector3<i16>
        where N: ArrayLength<u8>
    {
        Vector3::new(((u16(buffer[offset + 0]) << 8) | u16(buffer[offset + 1]))
                     as i16,
                     ((u16(buffer[offset + 2]) << 8) | u16(buffer[offset + 3]))
                     as i16,
                     ((u16(buffer[offset + 4]) << 8) | u16(buffer[offset + 5]))
                     as i16)
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
/// Controls (disable or enable) temperature sensor.
pub enum TempControl {
    /// Enable temperature sensor
    Enable = 0x80,
    /// Disable temperature sensor
    Disable = 0x00,
}

impl Default for TempControl {
    fn default() -> Self {
        TempControl::Enable
    }
}

impl RegisterBits for TempControl {
    fn mask() -> u8 {
        TempControl::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

#[derive(Debug, Clone, Copy)]
/// Block data update for accel data.
pub enum MagBdu {
    /// output registers not updated until MSB and LSB have been read
    Enable = 0x40,
    /// continuous update
    Disable = 0x00,
}

impl Default for MagBdu {
    fn default() -> Self {
        MagBdu::Enable
    }
}

impl RegisterBits for MagBdu {
    fn mask() -> u8 {
        MagBdu::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

#[derive(Debug, Clone, Copy)]
/// Block data update for accel data.
pub enum AccelBdu {
    /// output registers not updated until MSB and LSB have been read
    Enable = 0x08,
    /// continuous update
    Disable = 0x00,
}

impl Default for AccelBdu {
    fn default() -> Self {
        AccelBdu::Enable
    }
}

impl RegisterBits for AccelBdu {
    fn mask() -> u8 {
        AccelBdu::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}
