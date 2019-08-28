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
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;

mod accel;
mod conf;
mod mag;

use cast::{f32, u16};

use hal::blocking::i2c::{Write, WriteRead};

pub use conf::*;

/// G constant
pub const G: f32 = 9.807;
const TEMP_RESOLUTION: f32 = 0.125;
const TEMP_ZERO_OFFSET: f32 = 25.0;

const LSM_ACC_WHO_AM_I: u8 = 0x41;
const LSM_MAG_WHO_AM_I: u8 = 0x3d;

/// LSM Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Accelerometer WHO_AM_I returned invalid value (returned value is
    /// argument).
    InvalidAccDevice(u8),
    /// Magnetometer WHO_AM_I returned invalid value (returned value is
    /// argument).
    InvalidMagDevice(u8),
    /// Underlying bus error.
    BusError(E),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

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
    accel_axes_control: AccelAxesControl,
    temp_control: TempControl,
}

impl<I2C, E> Lsm303c<I2C> where I2C: WriteRead<Error = E> + Write<Error = E>
{
    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// default configuration.
    pub fn default(i2c: I2C) -> Result<Self, Error<E>> {
        Lsm303c::new(i2c, &mut LsmConfig::new())
    }

    /// Creates a new [`Lsm303c`] driver from a I2C peripheral with
    /// provided [`LsmConfig`].
    ///
    /// [`LsmConfig`]: ./conf/enum.LsmConfig.html
    pub fn new(i2c: I2C, config: &mut LsmConfig) -> Result<Self, Error<E>> {
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
                      accel_axes_control: config.accel_axes_control
                                                .unwrap_or_default(),
                      temp_control: config.temp_control.unwrap_or_default() };
        lsm303c.init_lsm()?;
        let acc_wai = lsm303c.acc_who_am_i()?;
        if acc_wai == LSM_ACC_WHO_AM_I {
            let mag_wai = lsm303c.mag_who_am_i()?;
            if mag_wai == LSM_MAG_WHO_AM_I {
                Ok(lsm303c)
            } else {
                Err(Error::InvalidMagDevice(mag_wai))
            }
        } else {
            Err(Error::InvalidAccDevice(acc_wai))
        }
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
        self._accel_axes_control()?;
        self._accel_data_rate()?;

        self._temp_control()?;

        Ok(())
    }

    /// Reads the accelerometer WHO_AM_I register
    pub fn acc_who_am_i(&mut self) -> Result<u8, E> {
        self.read_accel_register(accel::Register::WHO_AM_I)
    }

    /// Reads the magnetometer WHO_AM_I register
    pub fn mag_who_am_i(&mut self) -> Result<u8, E> {
        self.read_mag_register(mag::Register::WHO_AM_I)
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
        transpose(config.accel_axes_control
                        .map(|v| self.accel_axes_control(v)))?;

        Ok(())
    }

    /// Reads and returns unscaled accelerometer measurements (LSB).
    pub fn unscaled_accel<T: From<[i16; 3]>>(&mut self) -> Result<T, E> {
        let buffer = &mut [0; 6];
        self.read_accel_registers(accel::Register::OUT_X_L, buffer)?;

        Ok(self.to_vector(buffer, 0))
    }

    /// Reads and returns accelerometer measurements converted to g.
    pub fn accel<T: From<[f32; 3]>>(&mut self) -> Result<T, E> {
        let resolution = self.accel_scale.resolution();
        let scale = G * resolution;
        let res = convert(self.unscaled_accel()?, scale);
        Ok(res.into())
    }

    /// Reads and returns magnetometer measurements converted to mgauss.
    pub fn mag<T: From<[f32; 3]>>(&mut self) -> Result<T, E> {
        let resolution = self.mag_scale.resolution();
        let scale = resolution;
        let res = convert(self.unscaled_mag()?, scale);
        Ok(res.into())
    }

    /// Reads and returns unscaled magnetometer measurements (LSB).
    pub fn unscaled_mag<T: From<[i16; 3]>>(&mut self) -> Result<T, E> {
        let buffer = &mut [0; 6];
        self.read_mag_registers(mag::Register::OUTX_L, buffer)?;
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
        let buffer = &mut [0; 2];
        self.read_mag_registers(mag::Register::TEMP_OUT_L, buffer)?;
        let t = ((u16(buffer[1]) << 8) | u16(buffer[0])) as i16;
        Ok(t >> 4)
    }

    /// Reads and returns raw unscaled Accelerometer + Magnetometer +
    /// Thermometer measurements (LSB).
    pub fn unscaled_all<T: From<[i16; 3]>>(&mut self) -> Result<UnscaledMeasurements<T>, E>
    {
        let accel = self.unscaled_accel()?;
        let mag = self.unscaled_mag()?;
        let temp = self.raw_temp()?;

        Ok(UnscaledMeasurements { accel,
                                  mag,
                                  temp })
    }

    /// Reads and returns Accelerometer + Magnetometer + Thermometer
    /// measurements scaled and converted to respective units.
    pub fn all<T: From<[f32; 3]>>(&mut self) -> Result<Measurements<T>, E> {
        let accel = self.accel()?;
        let mag = self.mag()?;
        let temp = self.temp()?;

        Ok(Measurements { accel,
                          mag,
                          temp })
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

    /// Enables or disables accelerometer axes ([`AccelAxesControl`]).
    ///
    /// [`AccelAxesControl`]: ./conf/enum.AccelAxesControl.html
    pub fn accel_axes_control(&mut self,
                              axes_control: AccelAxesControl)
                              -> Result<(), E> {
        self.accel_axes_control = axes_control;
        self._accel_axes_control()
    }

    fn _accel_axes_control(&mut self) -> Result<(), E> {
        let ac = self.accel_axes_control;
        self.write_accel_register_with_mask(accel::Register::CTRL1, ac)
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

    fn to_vector<T>(&self,
                    buffer: &[u8; 6],
                    offset: usize)
                    -> T
    where T: From<[i16; 3]>
    {

        [
            ((u16(buffer[offset + 1]) << 8) | u16(buffer[offset + 0])) as i16,
            ((u16(buffer[offset + 3]) << 8) | u16(buffer[offset + 2])) as i16,
            ((u16(buffer[offset + 5]) << 8) | u16(buffer[offset + 4])) as i16
        ].into()
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

    fn read_accel_registers(&mut self,
                               reg: accel::Register,
                               buffer: &mut [u8],
                               )
                               -> Result<(), E>
    {
        const I2C_AUTO_INCREMENT: u8 = 1 << 7;
        self.i2c.write_read(accel::ADDRESS,
                             &[reg.addr() | I2C_AUTO_INCREMENT],
                             buffer)?;

        Ok(())
    }

    fn read_accel_register(&mut self, reg: accel::Register) -> Result<u8, E> {
        let buffer = &mut [0];
        self.read_accel_registers(reg, buffer)?;
        Ok(buffer[0])
    }

    fn read_mag_register(&mut self, reg: mag::Register) -> Result<u8, E> {
        let buffer = &mut [0; 1];
        self.read_mag_registers(reg, buffer)?;
        Ok(buffer[0])
    }

    // NOTE has weird address increment semantics; use only with `OUT_X_H`
    fn read_mag_registers(&mut self,
                             reg: mag::Register,
                             buffer: &mut [u8])
                             -> Result<(), E>
    {
        const I2C_AUTO_INCREMENT: u8 = 1 << 7;
        self.i2c.write_read(mag::ADDRESS,
                             &[reg.addr() | I2C_AUTO_INCREMENT],
                             buffer)?;

        Ok(())
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

/// Unscaled measurements (LSB)
#[derive(Copy, Clone, Debug)]
pub struct UnscaledMeasurements<T> {
    /// Accelerometer measurements (LSB)
    pub accel: T,
    /// Magnetometer measurements (LSB)
    pub mag: T,
    /// Temperature sensor measurement (LSB)
    pub temp: i16,
}

/// Measurements scaled with respective scales and converted
/// to appropriate units.
#[derive(Copy, Clone, Debug)]
pub struct Measurements<T> {
    /// Accelerometer measurements (g)
    pub accel: T,
    /// Magnetometer measurements (ga, gausses)
    pub mag: T,
    /// Temperature sensor measurement (C)
    pub temp: f32,
}

fn transpose<T, E>(o: Option<Result<T, E>>) -> Result<Option<T>, E> {
    match o {
        Some(Ok(x)) => Ok(Some(x)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}

fn convert(v: [i16; 3], scale: f32) -> [f32; 3] {
    [
        v[0] as f32 * scale,
        v[1] as f32 * scale,
        v[2] as f32 * scale,
    ]
}
