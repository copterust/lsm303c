pub(crate) trait RegisterBits {
    fn mask() -> u8;
    fn value(&self) -> u8;
}

/// Accelerometer Output Data Rate
#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum AccelDataRate {
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

impl Default for AccelDataRate {
    fn default() -> Self {
        AccelDataRate::_100_Hz
    }
}

impl RegisterBits for AccelDataRate {
    fn mask() -> u8 {
        AccelDataRate::_800_Hz.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

/// Acceleration scale/sensitivity
#[derive(Clone, Copy, Debug)]
#[allow(non_camel_case_types)]
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
#[derive(Clone, Copy, Debug)]
#[allow(non_camel_case_types)]
pub enum MagDataRate {
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

impl Default for MagDataRate {
    fn default() -> Self {
        MagDataRate::_40_Hz
    }
}

impl RegisterBits for MagDataRate {
    fn mask() -> u8 {
        MagDataRate::_80_Hz.value()
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

/// Magnetrometer X and Y axes operative mode selection.
/// Default: High performance.
#[derive(Debug, Clone, Copy)]
pub enum MagXYOperativeMode {
    /// Low power
    LowPower = 0x00,
    /// Medium performance
    MediumPerformance = 0x20,
    /// High performance
    HighPerformance = 0x40,
    /// Ultra high performance
    UltraHighPerformance = 0x60,
}

impl RegisterBits for MagXYOperativeMode {
    fn mask() -> u8 {
        MagXYOperativeMode::UltraHighPerformance.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Default for MagXYOperativeMode {
    fn default() -> Self {
        MagXYOperativeMode::HighPerformance
    }
}

/// Magnetrometer Z axis operative mode selection
/// Default value: HighPerformance.
#[derive(Debug, Clone, Copy)]
pub enum MagZOperativeMode {
    /// Low power
    LowPower = 0x00,
    /// Medium performance
    MediumPerformance = 0x04,
    /// High performance
    HighPerformance = 0x08,
    /// Ultra high performance
    UltraHighPerformance = 0x0C,
}

impl RegisterBits for MagZOperativeMode {
    fn mask() -> u8 {
        MagZOperativeMode::UltraHighPerformance.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Default for MagZOperativeMode {
    fn default() -> Self {
        MagZOperativeMode::HighPerformance
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
pub enum MagBlockDataUpdate {
    /// output registers not updated until MSB and LSB have been read
    Enable = 0x40,
    /// continuous update
    Disable = 0x00,
}

impl Default for MagBlockDataUpdate {
    fn default() -> Self {
        MagBlockDataUpdate::Enable
    }
}

impl RegisterBits for MagBlockDataUpdate {
    fn mask() -> u8 {
        MagBlockDataUpdate::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

#[derive(Debug, Clone, Copy)]
/// Block data update for accel data.
pub enum AccelBlockDataUpdate {
    /// output registers not updated until MSB and LSB have been read
    Enable = 0x08,
    /// continuous update
    Disable = 0x00,
}

impl Default for AccelBlockDataUpdate {
    fn default() -> Self {
        AccelBlockDataUpdate::Enable
    }
}

impl RegisterBits for AccelBlockDataUpdate {
    fn mask() -> u8 {
        AccelBlockDataUpdate::Enable.value()
    }

    fn value(&self) -> u8 {
        *self as u8
    }
}

/// Configuration of Lsm303c
#[derive(Copy, Clone, Debug)]
pub struct LsmConfig {
    pub(crate) accel_scale: Option<AccelScale>,
    pub(crate) mag_scale: Option<MagScale>,
    pub(crate) accel_data_rate: Option<AccelDataRate>,
    pub(crate) mag_data_rate: Option<MagDataRate>,
    pub(crate) accel_block_data_update: Option<AccelBlockDataUpdate>,
    pub(crate) mag_block_data_update: Option<MagBlockDataUpdate>,
    pub(crate) mag_mode: Option<MagMode>,
    pub(crate) mag_xy_operative_mode: Option<MagXYOperativeMode>,
    pub(crate) mag_z_operative_mode: Option<MagZOperativeMode>,
    pub(crate) temp_control: Option<TempControl>,
}

impl LsmConfig {
    /// Creates Lsm303c configuration with default
    /// [`AccelScale`], [`MagScale`], [`AccelDataRate`],
    /// [`MagDataRate`], [`MagMode`], [`AccelBlockDataUpdate`],
    /// [`MagBlockDataUpdate`], and [`TempControl`].
    ///
    /// [`AccelScale`]: ./enum.AccelScale.html
    /// [`MagScale`]: ./enum.MagScale.html
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    /// [`MagDataRate`]: ./enum.MagDataRate.html
    /// [`MagMode`]: ./enum.MagMode.html
    /// [`MagBlockDataUpdate`]: ./enum.MagBlockDataUpdate.html
    /// [`AccelBlockDataUpdate`]: ./enum.AccelBlockDataUpdate.html
    /// [`TempControl`]: ./enum.TempControl.html
    pub fn new() -> Self {
        LsmConfig { accel_scale: None,
                    mag_scale: None,
                    accel_data_rate: None,
                    mag_data_rate: None,
                    accel_block_data_update: None,
                    mag_block_data_update: None,
                    mag_mode: None,
                    mag_xy_operative_mode: None,
                    mag_z_operative_mode: None,
                    temp_control: None, }
    }

    /// Sets accelerometer full reading scale ([`AccelScale`])
    ///
    /// [`AccelScale`]: ./enum.AccelScale.html
    pub fn accel_scale(&mut self, accel_scale: AccelScale) -> &mut Self {
        self.accel_scale = Some(accel_scale);
        self
    }

    /// Sets magnetrometer full reading scale ([`MagScale`])
    ///
    /// [`MagScale`]: ./enum.MagScale.html
    pub fn mag_scale(&mut self, mag_scale: MagScale) -> &mut Self {
        self.mag_scale = Some(mag_scale);
        self
    }

    /// Sets accelerometer output data rate ([`AccelDataRate`])
    ///
    /// [`AccelDataRate`]: ./enum.AccelDataRate.html
    pub fn accel_data_rate(&mut self,
                           accel_data_rate: AccelDataRate)
                           -> &mut Self {
        self.accel_data_rate = Some(accel_data_rate);
        self
    }

    /// Sets magnetrometer output data rate ([`MagDataRate`])
    ///
    /// [`MagDataRate`]: ./enum.MagDataRate.html
    pub fn mag_data_rate(&mut self, mag_data_rate: MagDataRate) -> &mut Self {
        self.mag_data_rate = Some(mag_data_rate);
        self
    }

    /// Sets accelerometer block data update ([`AccelBlockDataUpdate`])
    ///
    /// [`AccelBlockDataUpdate`]: ./enum.AccelBlockDataUpdate.html
    pub fn accel_block_data_update(&mut self,
                                   accel_block_data_update: AccelBlockDataUpdate)
                                   -> &mut Self {
        self.accel_block_data_update = Some(accel_block_data_update);
        self
    }

    /// Sets magnetrometer block data update ([`MagBlockDataUpdate`])
    ///
    /// [`MagBlockDataUpdate`]: ./enum.MagBlockDataUpdate.html
    pub fn mag_block_data_update(&mut self,
                                 mag_block_data_update: MagBlockDataUpdate)
                                 -> &mut Self {
        self.mag_block_data_update = Some(mag_block_data_update);
        self
    }

    /// Sets magnetrometer mode ([`MagMode`])
    ///
    /// [`MagMode`]: ./enum.MagMode.html
    pub fn mag_mode(&mut self, mag_mode: MagMode) -> &mut Self {
        self.mag_mode = Some(mag_mode);
        self
    }

    /// Sets magnetrometer x and y axes operative mode ([`MagXYOperativeMode`])
    ///
    /// [`MagXYOperativeMode`]: ./enum.MagXYOperativeMode.html
    pub fn mag_xy_operative_mode(&mut self,
                                 operative_mode: MagXYOperativeMode)
                                 -> &mut Self {
        self.mag_xy_operative_mode = Some(operative_mode);
        self
    }

    /// Sets magnetrometer z axis operative mode ([`MagZOperativeMode`])
    ///
    /// [`MagZOperativeMode`]: ./enum.MagZOperativeMode.html
    pub fn mag_z_operative_mode(&mut self,
                                operative_mode: MagZOperativeMode)
                                -> &mut Self {
        self.mag_z_operative_mode = Some(operative_mode);
        self
    }

    /// Sets temperature control ([`TempControl`])
    ///
    /// [`TempControl`]: ./enum.TempControl.html
    pub fn temp_control(&mut self, temp_control: TempControl) -> &mut Self {
        self.temp_control = Some(temp_control);
        self
    }
}
