//! A platform agnostic driver to interface with the LSM303C (accelerometer +
//! compass)
//!
//! This driver is built using [`embedded-hal`] traits.
//!
//! # Usage
//!
//! Use embedded-hal implementation to get I2C, then create lsm handle
//!
//! ```
//! // to create sensor with default configuration:
//! let mut lsm = Lsm303c::default(i2c)?;
//! // to get all supported measurements:
//! let all = lsm.all()?;
//! println!("{:?}", all);
//! // One can also use conf module to supply configuration:
//! let mut lsm =
//!     LsmConfig::new(i2c,
//!                    LsmConfig::new().mag_scale(conf::MagScale::_16_Ga))?;
//! ```
//!
//! More examples (for stm32) in [Proving ground] repo.
//!
//! # References
//!
//! - [Register Map][1]
//!
//! [1]: http://www.farnell.com/datasheets/1878201.pdf
//!
//! - [Proving ground][2]
//!
//! [2]: https://github.com/copterust/proving-ground
//!
//! - [`embedded-hal`][3]
//!
//! [3]: https://github.com/rust-embedded/embedded-hal

#![deny(missing_docs)]
#![deny(warnings)]
#![allow(unused)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;
extern crate nalgebra;

mod accel;
mod conf;
mod mag;

use core::mem;

use cast::{f32, u16};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

pub use nalgebra::Vector3;

use hal::blocking::i2c::{Write, WriteRead};

pub use conf::*;

const TEMP_RESOLUTION: f32 = 0.125;
const TEMP_ZERO_OFFSET: f32 = 25.0;

/// LSM303C driver
pub struct Lsm303c<I2C> {
    // connections
    i2c: I2C,
    // configuration
    mag_scale: MagScale,
    accel_scale: AccelScale,
    accel_data_rate: AccelDataRate,
    accel_block_data_update: AccelBlockDataUpdate,
    mag_data_rate: MagDataRate,
    mag_mode: MagMode,
    mag_xy_operative_mode: MagXYOperativeMode,
    mag_z_operative_mode: MagZOperativeMode,
    mag_block_data_update: MagBlockDataUpdate,
    temp_control: TempControl,
}

impl<I2C, E> Lsm303c<I2C> where I2C: WriteRead<Error = E> + Write<Error = E>
{
    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// default configuration.
    pub fn default(i2c: I2C) -> Result<Self, E> {
        Lsm303c::new(i2c, &mut LsmConfig::new())
    }

    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// provided [`LsmConfig`].
    ///
    /// [`LsmConfig`]: ./conf/enum.LsmConfig.html
    pub fn new(i2c: I2C, config: &mut LsmConfig) -> Result<Self, E> {
        let mut lsm303c =
            Lsm303c { i2c,
                      accel_scale: config.accel_scale.unwrap_or_default(),
                      mag_scale: config.mag_scale.unwrap_or_default(),
                      accel_data_rate: config.accel_data_rate
                                             .unwrap_or_default(),
                      mag_data_rate: config.mag_data_rate
                                           .unwrap_or_default(),
                      accel_block_data_update:
                          config.accel_block_data_update.unwrap_or_default(),
                      mag_block_data_update: config.mag_block_data_update
                                                   .unwrap_or_default(),
                      mag_mode: config.mag_mode.unwrap_or_default(),
                      mag_xy_operative_mode: config.mag_xy_operative_mode
                                                   .unwrap_or_default(),
                      mag_z_operative_mode: config.mag_z_operative_mode
                                                  .unwrap_or_default(),
                      temp_control: config.temp_control.unwrap_or_default(), };
        lsm303c.init_lsm()?;

        Ok(lsm303c)
    }

    fn init_lsm(&mut self) -> Result<(), E> {
        // TODO reset all the registers / the device
        self._mag_data_rate()?;
        self._mag_scale()?;
        self._mag_block_data_update()?;
        self._mag_xy_operative_mode()?;
        self._mag_z_operative_mode()?;
        self._mag_mode()?;

        self._accel_scale()?;
        self._accel_block_data_update()?;
        // _accel_enable_axis()?;
        self._accel_data_rate()?;

        self._temp_control()?;

        Ok(())
    }

    /// Configures device using provided [`MpuConfig`].
    pub fn config(&mut self, config: &mut LsmConfig) -> Result<(), E> {
        transpose(config.mag_scale.map(|v| self.mag_scale(v)))?;
        transpose(config.accel_scale.map(|v| self.accel_scale(v)))?;
        transpose(config.accel_data_rate.map(|v| self.accel_data_rate(v)))?;
        transpose(config.mag_data_rate.map(|v| self.mag_data_rate(v)))?;
        transpose(config.mag_mode.map(|v| self.mag_mode(v)))?;
        transpose(config.mag_block_data_update
                        .map(|v| self.mag_block_data_update(v)))?;
        transpose(config.accel_block_data_update
                        .map(|v| self.accel_block_data_update(v)))?;
        transpose(config.mag_xy_operative_mode
                        .map(|v| self.mag_xy_operative_mode(v)))?;
        transpose(config.mag_z_operative_mode
                        .map(|v| self.mag_z_operative_mode(v)))?;

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

    /// Configures the accelerometer output data rate
    ///
    /// [`Accel scale`]: ./conf/enum.AccelScale.html
    pub fn accel_data_rate(&mut self,
                           data_rate: AccelDataRate)
                           -> Result<(), E> {
        self.accel_data_rate = data_rate;
        self._accel_data_rate()
    }

    fn _accel_data_rate(&mut self) -> Result<(), E> {
        let dr = self.accel_data_rate;
        self.write_accel_register_with_mask(accel::Register::CTRL1, dr)
    }

    /// Configures the magnetometer output data rate  ([`MagDataRate`]).
    ///
    /// [`MagDataRate`]: ./conf/enum.MagDataRate.html
    pub fn mag_data_rate(&mut self, data_rate: MagDataRate) -> Result<(), E> {
        self.mag_data_rate = data_rate;
        self._mag_data_rate()
    }

    fn _mag_data_rate(&mut self) -> Result<(), E> {
        let dr = self.mag_data_rate;
        self.write_mag_register_with_mask(mag::Register::CTRL1, dr)
    }

    /// Configures accelerometer full reading scale ([`AccelScale`]).
    ///
    /// [`AccelScale`]: ./conf/enum.AccelScale.html
    pub fn accel_scale(&mut self, accel_scale: AccelScale) -> Result<(), E> {
        self.accel_scale = accel_scale;
        self._accel_scale()
    }

    fn _accel_scale(&mut self) -> Result<(), E> {
        let asc = self.accel_scale;
        self.write_accel_register_with_mask(accel::Register::CTRL4, asc)
    }

    /// Configures accelerometer block data update ([`AccelBlockDataUpdate`]).
    ///
    /// [`AccelBlockDataUpdate`]: ./conf/enum.AccelBlockDataUpdate.html
    pub fn accel_block_data_update(&mut self,
                                   bdu: AccelBlockDataUpdate)
                                   -> Result<(), E> {
        self.accel_block_data_update = bdu;
        self._accel_block_data_update()
    }

    fn _accel_block_data_update(&mut self) -> Result<(), E> {
        let ab = self.accel_block_data_update;
        self.write_accel_register_with_mask(accel::Register::CTRL1, ab)
    }

    /// Configures temperature control ([`TempControl`]).
    ///
    /// [`TempControl`]: ./conf/enum.TempControl.html
    pub fn temp_control(&mut self, control: TempControl) -> Result<(), E> {
        self.temp_control = control;
        self._temp_control()
    }

    fn _temp_control(&mut self) -> Result<(), E> {
        let tc = self.temp_control;
        self.write_mag_register_with_mask(mag::Register::CTRL1, tc)
    }

    /// Configures magnetometer mode ([`MagMode`]).
    ///
    /// [`MagMode`]: ./conf/enum.MagMode.html
    pub fn mag_mode(&mut self, mode: MagMode) -> Result<(), E> {
        self.mag_mode = mode;
        self._mag_mode()
    }

    fn _mag_mode(&mut self) -> Result<(), E> {
        let mm = self.mag_mode;
        self.write_mag_register_with_mask(mag::Register::CTRL3, mm)
    }

    /// Configures magnetometer x and y axes operative mode
    /// ([`MagXYOperativeMode`]).
    ///
    /// [`MagXYOperativeMode`]: ./conf/enum.MagXYOperativeMode.html
    pub fn mag_xy_operative_mode(&mut self,
                                 operative_mode: MagXYOperativeMode)
                                 -> Result<(), E> {
        self.mag_xy_operative_mode = operative_mode;
        self._mag_xy_operative_mode()
    }

    fn _mag_xy_operative_mode(&mut self) -> Result<(), E> {
        let om = self.mag_xy_operative_mode;
        self.write_mag_register_with_mask(mag::Register::CTRL1, om)
    }

    /// Configures magnetometer z axis operative mode ([`MagXOperativeMode`]).
    ///
    /// [`MagZOperativeMode`]: ./conf/enum.MagZOperativeMode.html
    pub fn mag_z_operative_mode(&mut self,
                                operative_mode: MagZOperativeMode)
                                -> Result<(), E> {
        self.mag_z_operative_mode = operative_mode;
        self._mag_z_operative_mode()
    }

    fn _mag_z_operative_mode(&mut self) -> Result<(), E> {
        let om = self.mag_z_operative_mode;
        self.write_mag_register_with_mask(mag::Register::CTRL4, om)
    }

    /// Configures magnetometer full reading scale ([`MagScale`]).
    ///
    /// [`MagScale`]: ./conf/enum.MagScale.html
    pub fn mag_scale(&mut self, mag_scale: MagScale) -> Result<(), E> {
        self.mag_scale = mag_scale;
        self._mag_scale()
    }

    fn _mag_scale(&mut self) -> Result<(), E> {
        let ms = self.mag_scale;
        self.write_mag_register_with_mask(mag::Register::CTRL2, ms)
    }

    /// Configures magnetometer block data update ([`MagBlockDataUpdate`]).
    ///
    /// [`MagBlockDataUpdate`]: ./conf/enum.MagBlockDataUpdate.html
    pub fn mag_block_data_update(&mut self,
                                 bdu: MagBlockDataUpdate)
                                 -> Result<(), E> {
        self.mag_block_data_update = bdu;
        self._mag_block_data_update()
    }

    fn _mag_block_data_update(&mut self) -> Result<(), E> {
        let mb = self.mag_block_data_update;
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

fn transpose<T, E>(o: Option<Result<T, E>>) -> Result<Option<T>, E> {
    match o {
        Some(Ok(x)) => Ok(Some(x)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}
