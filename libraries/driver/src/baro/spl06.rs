use embassy_stm32::i2c::{I2c, Instance};
use embassy_time::{Timer, Duration};
use crate::baro::{BarometerChip, I2cDevice};
use core::marker::PhantomData;

// SPL06 I2C addresses
pub const SPL06_I2C_ADDR_PRIMARY: u8 = 0x76;
pub const SPL06_I2C_ADDR_SECONDARY: u8 = 0x77;

// SPL06 register addresses
const SPL06_REG_ID: u8 = 0x0D;
const SPL06_REG_PSR_B2: u8 = 0x00;
const SPL06_REG_PSR_B1: u8 = 0x01;
const SPL06_REG_PSR_B0: u8 = 0x02;
const SPL06_REG_TMP_B2: u8 = 0x03;
const SPL06_REG_TMP_B1: u8 = 0x04;
const SPL06_REG_TMP_B0: u8 = 0x05;
const SPL06_REG_PRS_CFG: u8 = 0x06;
const SPL06_REG_TMP_CFG: u8 = 0x07;
const SPL06_REG_MEAS_CFG: u8 = 0x08;
const SPL06_REG_CFG_REG: u8 = 0x09;
const SPL06_REG_INT_STS: u8 = 0x0A;
const SPL06_REG_FIFO_STS: u8 = 0x0B;
const SPL06_REG_RESET: u8 = 0x0C;
const SPL06_REG_COEF_C0: u8 = 0x10;

// SPL06 sensor constants
const SPL06_CHIP_ID: u8 = 0x10;
const SPL06_RESET_CMD: u8 = 0x89;

// SPL06 measurement modes
pub const SPL06_MEAS_STANDBY: u8 = 0x00;
pub const SPL06_MEAS_PRESSURE_ONLY: u8 = 0x01;
pub const SPL06_MEAS_TEMPERATURE_ONLY: u8 = 0x02;
pub const SPL06_MEAS_CONTINUOUS_PRESSURE: u8 = 0x05;
pub const SPL06_MEAS_CONTINUOUS_TEMP: u8 = 0x06;
pub const SPL06_MEAS_CONTINUOUS_BOTH: u8 = 0x07;

// SPL06 pressure measurement rate (samples per second)
pub const SPL06_PM_RATE_1: u8 = 0x00;
pub const SPL06_PM_RATE_2: u8 = 0x10;
pub const SPL06_PM_RATE_4: u8 = 0x20;
pub const SPL06_PM_RATE_8: u8 = 0x30;
pub const SPL06_PM_RATE_16: u8 = 0x40;
pub const SPL06_PM_RATE_32: u8 = 0x50;
pub const SPL06_PM_RATE_64: u8 = 0x60;
pub const SPL06_PM_RATE_128: u8 = 0x70;

// SPL06 temperature measurement rate (samples per second)
pub const SPL06_TMP_RATE_1: u8 = 0x00;
pub const SPL06_TMP_RATE_2: u8 = 0x10;
pub const SPL06_TMP_RATE_4: u8 = 0x20;
pub const SPL06_TMP_RATE_8: u8 = 0x30;
pub const SPL06_TMP_RATE_16: u8 = 0x40;
pub const SPL06_TMP_RATE_32: u8 = 0x50;
pub const SPL06_TMP_RATE_64: u8 = 0x60;
pub const SPL06_TMP_RATE_128: u8 = 0x70;

// SPL06 oversampling rates
pub const SPL06_OVERSAMPLE_1: u8 = 0x00;
pub const SPL06_OVERSAMPLE_2: u8 = 0x01;
pub const SPL06_OVERSAMPLE_4: u8 = 0x02;
pub const SPL06_OVERSAMPLE_8: u8 = 0x03;
pub const SPL06_OVERSAMPLE_16: u8 = 0x04;
pub const SPL06_OVERSAMPLE_32: u8 = 0x05;
pub const SPL06_OVERSAMPLE_64: u8 = 0x06;
pub const SPL06_OVERSAMPLE_128: u8 = 0x07;

// SPL06 temperature sensor source
pub const SPL06_TMP_SRC_INTERNAL: u8 = 0x00;
pub const SPL06_TMP_SRC_EXTERNAL: u8 = 0x80;

// SPL06 interrupt pin settings
pub const SPL06_INTERRUPT_DISABLE: u8 = 0x00;
pub const SPL06_INTERRUPT_PRESSURE: u8 = 0x01;
pub const SPL06_INTERRUPT_TEMPERATURE: u8 = 0x02;
pub const SPL06_INTERRUPT_BOTH: u8 = 0x03;
pub const SPL06_INTERRUPT_FIFO: u8 = 0x04;

// SPL06 FIFO settings
pub const SPL06_FIFO_DISABLE: u8 = 0x00;
pub const SPL06_FIFO_ENABLE: u8 = 0x08;

/// SPL06 configuration structure
#[derive(Debug, Clone, Copy)]
pub struct Spl06Config {
    /// I2C address of the sensor
    pub i2c_addr: u8,
    
    /// Pressure sensor oversampling rate
    pub pressure_oversample: u8,
    
    /// Pressure measurement rate
    pub pressure_rate: u8,
    
    /// Temperature sensor oversampling rate
    pub temperature_oversample: u8,
    
    /// Temperature measurement rate
    pub temperature_rate: u8,
    
    /// Temperature sensor source (internal or external)
    pub temperature_source: u8,
    
    /// Measurement mode (standby, continuous, etc.)
    pub measurement_mode: u8,
    
    /// FIFO mode
    pub fifo_mode: u8,
    
    /// Interrupt settings
    pub interrupt_settings: u8,
}

impl Default for Spl06Config {
    fn default() -> Self {
        Self {
            i2c_addr: SPL06_I2C_ADDR_PRIMARY,
            pressure_oversample: SPL06_OVERSAMPLE_8,
            pressure_rate: SPL06_PM_RATE_8,
            temperature_oversample: SPL06_OVERSAMPLE_8,
            temperature_rate: SPL06_TMP_RATE_8,
            temperature_source: SPL06_TMP_SRC_INTERNAL,
            measurement_mode: SPL06_MEAS_CONTINUOUS_BOTH,
            fifo_mode: SPL06_FIFO_DISABLE,
            interrupt_settings: SPL06_INTERRUPT_DISABLE,
        }
    }
}

/// SPL06 calibration coefficients
#[derive(Debug, Default)]
struct Spl06CalibData {
    c0: i16,     // 12-bit
    c1: i16,     // 12-bit
    c00: i32,    // 20-bit
    c10: i32,    // 20-bit
    c01: i16,    // 16-bit
    c11: i16,    // 16-bit
    c20: i16,    // 16-bit
    c21: i16,    // 16-bit
    c30: i16,    // 16-bit
    
    // Scaling factors
    pressure_scale: f32,
    temperature_scale: f32,
}

/// Raw temperature and pressure data
#[derive(Debug, Clone, Copy)]
struct Spl06RawData {
    raw_temp: i32,
    raw_pressure: i32,
    scaled_temp: f32,
    t_ready: bool,
    p_ready: bool,
}

impl Default for Spl06RawData {
    fn default() -> Self {
        Self {
            raw_temp: 0,
            raw_pressure: 0,
            scaled_temp: 0.0,
            t_ready: false,
            p_ready: false,
        }
    }
}

/// SPL06 barometer driver
pub struct Spl06<I: I2cDevice> {
    /// Configuration for the sensor
    config: Spl06Config,
    /// I2C device for communication
    i2c: Option<I>,
    /// Calibration data for the sensor
    cal_data: Spl06CalibData,
    /// Most recent measurement data
    raw_data: Spl06RawData,
    /// Whether the sensor is initialized
    initialized: bool,
}

// Implement Send if the I2C device is Send
unsafe impl<I: I2cDevice + Send> Send for Spl06<I> {}

impl<I: I2cDevice> Spl06<I> {
    /// Create a new SPL06 instance
    pub fn new() -> Self {
        Self {
            config: Spl06Config::default(),
            i2c: None,
            cal_data: Spl06CalibData::default(),
            raw_data: Spl06RawData::default(),
            initialized: false,
        }
    }
    
    /// Create a new SPL06 instance with custom configuration
    pub fn new_with_config(config: Spl06Config) -> Self {
        Self {
            config,
            i2c: None,
            cal_data: Spl06CalibData::default(),
            raw_data: Spl06RawData::default(),
            initialized: false,
        }
    }
    
    /// Set the I2C device for communication
    pub fn set_i2c(&mut self, i2c: I) {
        self.i2c = Some(i2c);
    }
    
    /// Read a register
    async fn read_reg(&mut self, reg: u8) -> Result<u8, ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.read_reg(self.config.i2c_addr, reg).await
        } else {
            Err(())
        }
    }
    
    /// Write to a register
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.write_reg(self.config.i2c_addr, reg, value).await
        } else {
            Err(())
        }
    }
    
    /// Read multiple registers
    async fn read_regs(&mut self, reg: u8, data: &mut [u8]) -> Result<(), ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.read_regs(self.config.i2c_addr, reg, data).await
        } else {
            Err(())
        }
    }
    
    /// Read the chip ID
    async fn read_chip_id(&mut self) -> Result<u8, ()> {
        self.read_reg(SPL06_REG_ID).await
    }
    
    /// Perform a soft reset of the sensor
    async fn soft_reset(&mut self) -> Result<(), ()> {
        self.write_reg(SPL06_REG_RESET, SPL06_RESET_CMD).await?;
        Timer::after(Duration::from_millis(40)).await;
        Ok(())
    }
    
    /// Read calibration coefficients from the sensor
    /// 
    /// The SPL06 calibration data is stored in registers 0x10 to 0x21
    async fn read_calibration_coefficients(&mut self) -> Result<Spl06CalibData, ()> {
        // Read all calibration registers at once
        let mut coef_buf = [0u8; 18];
        self.read_regs(SPL06_REG_COEF_C0, &mut coef_buf).await?;
        
        // Calculate c0, c1 (12-bit values, using first 3 bytes)
        let mut c0 = ((coef_buf[0] as u16) << 4) | ((coef_buf[1] as u16) >> 4);
        if c0 & 0x800 != 0 {
            c0 |= 0xF000; // Sign extension for negative values
        }
        
        let mut c1 = (((coef_buf[1] as u16) & 0x0F) << 8) | (coef_buf[2] as u16);
        if c1 & 0x800 != 0 {
            c1 |= 0xF000; // Sign extension
        }
        
        // Calculate c00, c10 (20-bit values, next 5 bytes)
        let mut c00 = ((coef_buf[3] as u32) << 12) | ((coef_buf[4] as u32) << 4) | ((coef_buf[5] as u32) >> 4);
        if c00 & 0x80000 != 0 {
            c00 |= 0xFFF00000; // Sign extension
        }
        
        let mut c10 = (((coef_buf[5] as u32) & 0x0F) << 16) | ((coef_buf[6] as u32) << 8) | (coef_buf[7] as u32);
        if c10 & 0x80000 != 0 {
            c10 |= 0xFFF00000; // Sign extension
        }
        
        // Calculate remaining coefficients (16-bit values)
        let c01 = ((coef_buf[8] as i16) << 8) | (coef_buf[9] as i16);
        let c11 = ((coef_buf[10] as i16) << 8) | (coef_buf[11] as i16);
        let c20 = ((coef_buf[12] as i16) << 8) | (coef_buf[13] as i16);
        let c21 = ((coef_buf[14] as i16) << 8) | (coef_buf[15] as i16);
        let c30 = ((coef_buf[16] as i16) << 8) | (coef_buf[17] as i16);
        
        // Calculate scaling factors based on oversampling rates
        let pressure_scale = match self.config.pressure_oversample {
            SPL06_OVERSAMPLE_1 => 524288.0,
            SPL06_OVERSAMPLE_2 => 1572864.0,
            SPL06_OVERSAMPLE_4 => 3670016.0,
            SPL06_OVERSAMPLE_8 => 7864320.0,
            SPL06_OVERSAMPLE_16 => 253952.0,
            SPL06_OVERSAMPLE_32 => 516096.0,
            SPL06_OVERSAMPLE_64 => 1040384.0,
            SPL06_OVERSAMPLE_128 => 2088960.0,
            _ => 524288.0, // Default to single measurement scaling
        };
        
        let temperature_scale = match self.config.temperature_oversample {
            SPL06_OVERSAMPLE_1 => 524288.0,
            SPL06_OVERSAMPLE_2 => 1572864.0,
            SPL06_OVERSAMPLE_4 => 3670016.0,
            SPL06_OVERSAMPLE_8 => 7864320.0,
            SPL06_OVERSAMPLE_16 => 253952.0,
            SPL06_OVERSAMPLE_32 => 516096.0,
            SPL06_OVERSAMPLE_64 => 1040384.0,
            SPL06_OVERSAMPLE_128 => 2088960.0,
            _ => 524288.0, // Default to single measurement scaling
        };
        
        // Create and return the calibration data structure
        let cal_data = Spl06CalibData {
            c0: c0.try_into().unwrap_or(0),
            c1: c1.try_into().unwrap_or(0),
            c00: c00 as i32,
            c10: c10 as i32,
            c01,
            c11,
            c20,
            c21,
            c30,
            pressure_scale,
            temperature_scale,
        };
        
        Ok(cal_data)
    }
    
    /// Read raw temperature value
    async fn read_raw_temperature(&mut self) -> Result<i32, ()> {
        // If we're in standby mode, trigger a measurement
        if self.config.measurement_mode == SPL06_MEAS_STANDBY {
            // Configure for a single temperature measurement
            self.write_reg(SPL06_REG_MEAS_CFG, SPL06_MEAS_TEMPERATURE_ONLY).await?;
            
            // Wait for measurement to complete (time depends on oversampling)
            let wait_time_ms = match self.config.temperature_oversample {
                SPL06_OVERSAMPLE_1 => 4,
                SPL06_OVERSAMPLE_2 => 5,
                SPL06_OVERSAMPLE_4 => 8,
                SPL06_OVERSAMPLE_8 => 14,
                SPL06_OVERSAMPLE_16 => 24,
                SPL06_OVERSAMPLE_32 => 45,
                SPL06_OVERSAMPLE_64 => 85,
                SPL06_OVERSAMPLE_128 => 165,
                _ => 4,
            };
            Timer::after(Duration::from_millis(wait_time_ms)).await;
            
            // Check if measurement is ready
            let status = self.read_reg(SPL06_REG_MEAS_CFG).await?;
            if status & 0x20 == 0 {
                return Err(()); // Temperature measurement not ready
            }
        }
        
        // Read the temperature registers
        let mut temp_bytes = [0u8; 3];
        self.read_regs(SPL06_REG_TMP_B2, &mut temp_bytes).await?;
        
        // Combine bytes into a 24-bit value
        let mut raw_temp = ((temp_bytes[0] as i32) << 16) | 
                           ((temp_bytes[1] as i32) << 8) | 
                           (temp_bytes[2] as i32);
        
        // Sign extend if negative (MSB is 1)
        if raw_temp & 0x800000 != 0 {
            raw_temp |= -0x1000000;
        }
        
        // Mark temperature as ready in raw data
        self.raw_data.t_ready = true;
        self.raw_data.raw_temp = raw_temp;
        
        Ok(raw_temp)
    }
    
    /// Read raw pressure value
    async fn read_raw_pressure(&mut self) -> Result<i32, ()> {
        // If we're in standby mode, trigger a measurement
        if self.config.measurement_mode == SPL06_MEAS_STANDBY {
            // Configure for a single pressure measurement
            self.write_reg(SPL06_REG_MEAS_CFG, SPL06_MEAS_PRESSURE_ONLY).await?;
            
            // Wait for measurement to complete (time depends on oversampling)
            let wait_time_ms = match self.config.pressure_oversample {
                SPL06_OVERSAMPLE_1 => 4,
                SPL06_OVERSAMPLE_2 => 5,
                SPL06_OVERSAMPLE_4 => 8,
                SPL06_OVERSAMPLE_8 => 14,
                SPL06_OVERSAMPLE_16 => 24,
                SPL06_OVERSAMPLE_32 => 45,
                SPL06_OVERSAMPLE_64 => 85,
                SPL06_OVERSAMPLE_128 => 165,
                _ => 4,
            };
            Timer::after(Duration::from_millis(wait_time_ms)).await;
            
            // Check if measurement is ready
            let status = self.read_reg(SPL06_REG_MEAS_CFG).await?;
            if status & 0x10 == 0 {
                return Err(()); // Pressure measurement not ready
            }
        }
        
        // Read the pressure registers
        let mut pressure_bytes = [0u8; 3];
        self.read_regs(SPL06_REG_PSR_B2, &mut pressure_bytes).await?;
        
        // Combine bytes into a 24-bit value
        let mut raw_pressure = ((pressure_bytes[0] as i32) << 16) | 
                               ((pressure_bytes[1] as i32) << 8) | 
                               (pressure_bytes[2] as i32);
        
        // Sign extend if negative (MSB is 1)
        if raw_pressure & 0x800000 != 0 {
            raw_pressure |= -0x1000000;
        }
        
        // Mark pressure as ready in raw data
        self.raw_data.p_ready = true;
        self.raw_data.raw_pressure = raw_pressure;
        
        Ok(raw_pressure)
    }
    
    /// Calculate scaled temperature from raw value
    fn calculate_scaled_temperature(&mut self, raw_temp: i32) -> f32 {
        // Convert raw temperature to scaled temperature using calibration coefficients
        let scaled_temp = self.cal_data.c0 as f32 * 0.5 + 
                          self.cal_data.c1 as f32 * (raw_temp as f32 / self.cal_data.temperature_scale);
        
        // Store the scaled temperature for pressure compensation
        self.raw_data.scaled_temp = scaled_temp;
        
        scaled_temp
    }
    
    /// Calculate compensated pressure from raw pressure and temperature
    fn calculate_compensated_pressure(&self, raw_pressure: i32, scaled_temp: f32) -> f32 {
        // Convert raw pressure to scaled pressure using calibration coefficients
        let scaled_pressure = raw_pressure as f32 / self.cal_data.pressure_scale;
        
        // Apply compensation formula with temperature dependency
        let pressure = self.cal_data.c00 as f32 + 
                       scaled_pressure * (self.cal_data.c10 as f32 + 
                                         scaled_pressure * (self.cal_data.c20 as f32 + 
                                                           scaled_pressure * self.cal_data.c30 as f32)) + 
                       scaled_temp * (self.cal_data.c01 as f32 + 
                                     scaled_pressure * (self.cal_data.c11 as f32 + 
                                                       scaled_pressure * self.cal_data.c21 as f32));
        
        pressure
    }
}

impl<I: I2cDevice> BarometerChip for Spl06<I> {
    async fn init(&mut self) -> Result<(), ()> {
        // Make sure we have an I2C device
        if self.i2c.is_none() {
            return Err(());
        }
        
        // Reset the device
        self.soft_reset().await?;
        
        // Check chip ID
        let chip_id = self.read_chip_id().await?;
        if chip_id != SPL06_CHIP_ID {
            return Err(());
        }
        
        // Wait for sensor to be ready
        Timer::after(Duration::from_millis(40)).await;
        
        // Read calibration coefficients
        self.cal_data = self.read_calibration_coefficients().await?;
        
        // Configure the pressure sensor
        // Pressure config register: bits 0-2 for oversampling, bits 4-6 for rate
        let prs_cfg = self.config.pressure_oversample | self.config.pressure_rate;
        self.write_reg(SPL06_REG_PRS_CFG, prs_cfg).await?;
        
        // Configure the temperature sensor
        // Temperature config register: bits 0-2 for oversampling, bits 4-6 for rate, bit 7 for source
        let tmp_cfg = self.config.temperature_oversample | self.config.temperature_rate | self.config.temperature_source;
        self.write_reg(SPL06_REG_TMP_CFG, tmp_cfg).await?;
        
        // Configure interrupts and FIFO if enabled
        let cfg_reg = if self.config.interrupt_settings != SPL06_INTERRUPT_DISABLE {
            self.config.interrupt_settings | self.config.fifo_mode
        } else {
            self.config.fifo_mode
        };
        self.write_reg(SPL06_REG_CFG_REG, cfg_reg).await?;
        
        // Set the measurement mode
        self.write_reg(SPL06_REG_MEAS_CFG, self.config.measurement_mode).await?;
        
        // Take initial readings to ensure system is working
        if self.config.measurement_mode == SPL06_MEAS_CONTINUOUS_BOTH || 
           self.config.measurement_mode == SPL06_MEAS_CONTINUOUS_TEMP {
            // For continuous mode, wait a bit for first measurements
            Timer::after(Duration::from_millis(50)).await;
        } else {
            // For single measurements, explicitly request them
            self.read_raw_temperature().await?;
            self.read_raw_pressure().await?;
        }
        
        self.initialized = true;
        Ok(())
    }
    
    async fn read_pressure(&mut self) -> Result<f32, ()> {
        if !self.initialized {
            return Err(());
        }
        
        // Ensure we have a temperature reading for compensation
        if !self.raw_data.t_ready {
            self.read_raw_temperature().await?;
            self.calculate_scaled_temperature(self.raw_data.raw_temp);
        }
        
        // Read raw pressure
        let raw_pressure = self.read_raw_pressure().await?;
        
        // Calculate compensated pressure in Pascal
        let pressure_pa = self.calculate_compensated_pressure(raw_pressure, self.raw_data.scaled_temp);
        
        Ok(pressure_pa)
    }
    
    async fn read_temperature(&mut self) -> Result<f32, ()> {
        if !self.initialized {
            return Err(());
        }
        
        // Read raw temperature
        let raw_temp = self.read_raw_temperature().await?;
        
        // Calculate calibrated temperature in Celsius
        let temperature_c = self.calculate_scaled_temperature(raw_temp);
        
        Ok(temperature_c)
    }
    
    async fn self_test(&mut self) -> Result<bool, ()> {
        if !self.initialized {
            // Initialize if not already done
            self.init().await?;
        }
        
        // Test by reading temperature and pressure
        let temp_result = self.read_temperature().await;
        let pressure_result = self.read_pressure().await;
        
        // Check if both readings were successful and in reasonable ranges
        match (temp_result, pressure_result) {
            (Ok(temp), Ok(pressure)) => {
                // Check if values are within reasonable ranges
                // Temperature: -40°C to 85°C
                // Pressure: 30,000 Pa (300 mbar) to 110,000 Pa (1100 mbar)
                if temp >= -40.0 && temp <= 85.0 && pressure >= 30000.0 && pressure <= 110000.0 {
                    Ok(true)
                } else {
                    Ok(false)
                }
            },
            _ => Ok(false),
        }
    }
    
    async fn calibrate(&mut self) -> Result<(), ()> {
        // For SPL06, calibration is primarily reading the factory coefficients
        // which is already done during initialization
        
        // We can do a soft reset and re-initialize to ensure fresh state
        self.soft_reset().await?;
        self.initialized = false;
        self.init().await
    }
    
    async fn reset(&mut self) -> Result<(), ()> {
        self.soft_reset().await?;
        self.initialized = false;
        Ok(())
    }
    
    async fn set_update_rate(&mut self, rate_hz: u8) -> Result<u8, ()> {
        if !self.initialized {
            return Err(());
        }
        
        // Convert Hz to SPL06 rate setting
        let (pressure_rate, temp_rate) = match rate_hz {
            1 => (SPL06_PM_RATE_1, SPL06_TMP_RATE_1),
            2 => (SPL06_PM_RATE_2, SPL06_TMP_RATE_2),
            r if r <= 4 => (SPL06_PM_RATE_4, SPL06_TMP_RATE_4),
            r if r <= 8 => (SPL06_PM_RATE_8, SPL06_TMP_RATE_8),
            r if r <= 16 => (SPL06_PM_RATE_16, SPL06_TMP_RATE_16),
            r if r <= 32 => (SPL06_PM_RATE_32, SPL06_TMP_RATE_32),
            r if r <= 64 => (SPL06_PM_RATE_64, SPL06_TMP_RATE_64),
            _ => (SPL06_PM_RATE_128, SPL06_TMP_RATE_128),
        };
        
        // Update configuration
        self.config.pressure_rate = pressure_rate;
        self.config.temperature_rate = temp_rate;
        
        // Apply new settings
        let prs_cfg = self.config.pressure_oversample | pressure_rate;
        self.write_reg(SPL06_REG_PRS_CFG, prs_cfg).await?;
        
        let tmp_cfg = self.config.temperature_oversample | temp_rate | self.config.temperature_source;
        self.write_reg(SPL06_REG_TMP_CFG, tmp_cfg).await?;
        
        // Return actual rate in Hz
        let actual_rate = match pressure_rate {
            SPL06_PM_RATE_1 => 1,
            SPL06_PM_RATE_2 => 2,
            SPL06_PM_RATE_4 => 4,
            SPL06_PM_RATE_8 => 8,
            SPL06_PM_RATE_16 => 16,
            SPL06_PM_RATE_32 => 32,
            SPL06_PM_RATE_64 => 64,
            SPL06_PM_RATE_128 => 128,
            _ => 1,
        };
        
        Ok(actual_rate)
    }
    
    fn chip_name(&self) -> &'static str {
        "SPL06"
    }
} 