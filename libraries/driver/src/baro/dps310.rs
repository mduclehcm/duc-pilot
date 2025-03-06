use embassy_time::{Timer, Duration};
use crate::baro::{BarometerChip, I2cDevice};

// DPS310 I2C addresses
pub const DPS310_I2C_ADDR_PRIMARY: u8 = 0x77;
pub const DPS310_I2C_ADDR_SECONDARY: u8 = 0x76;

// DPS310 register addresses
const DPS310_REG_PSR_B2: u8 = 0x00;     // Pressure data byte 2
const DPS310_REG_PSR_B1: u8 = 0x01;     // Pressure data byte 1
const DPS310_REG_PSR_B0: u8 = 0x02;     // Pressure data byte 0
const DPS310_REG_TMP_B2: u8 = 0x03;     // Temperature data byte 2
const DPS310_REG_TMP_B1: u8 = 0x04;     // Temperature data byte 1
const DPS310_REG_TMP_B0: u8 = 0x05;     // Temperature data byte 0
const DPS310_REG_PRS_CFG: u8 = 0x06;    // Pressure configuration
const DPS310_REG_TMP_CFG: u8 = 0x07;    // Temperature configuration
const DPS310_REG_MEAS_CFG: u8 = 0x08;   // Measurement configuration
const DPS310_REG_CFG_REG: u8 = 0x09;    // Interrupt/FIFO configuration
const DPS310_REG_INT_STS: u8 = 0x0A;    // Interrupt status
const DPS310_REG_FIFO_STS: u8 = 0x0B;   // FIFO status
const DPS310_REG_RESET: u8 = 0x0C;      // Soft reset
const DPS310_REG_PROD_ID: u8 = 0x0D;    // Product ID (= 0x10)
const DPS310_REG_COEF_SRC: u8 = 0x28;   // Temperature coefficient source

// Coefficient registers (from 0x10 to 0x21)
const DPS310_REG_COEF: u8 = 0x10;
const DPS310_REG_COEF_SRCE: u8 = 0x28;

// DPS310 sensor constants
const DPS310_PROD_ID: u8 = 0x10;
const DPS310_RESET_CODE: u8 = 0x09;
const DPS310_READY_BIT: u8 = 0x80;      // Sensor initialization done bit
const DPS310_COEF_RDY: u8 = 0x80;       // Coefficients available in memory

// DPS310 measurement modes
pub const DPS310_MODE_IDLE: u8 = 0x00;
pub const DPS310_MODE_PRESSURE_ONCE: u8 = 0x01;
pub const DPS310_MODE_TEMPERATURE_ONCE: u8 = 0x02;
pub const DPS310_MODE_CONTINUOUS_PRESSURE: u8 = 0x05;
pub const DPS310_MODE_CONTINUOUS_TEMP: u8 = 0x06;
pub const DPS310_MODE_CONTINUOUS_BOTH: u8 = 0x07;

// DPS310 pressure measurement rate (samples per second)
pub const DPS310_PM_RATE_1: u8 = 0x00;   // 1 measurement per second
pub const DPS310_PM_RATE_2: u8 = 0x10;   // 2 measurements per second
pub const DPS310_PM_RATE_4: u8 = 0x20;   // 4 measurements per second
pub const DPS310_PM_RATE_8: u8 = 0x30;   // 8 measurements per second
pub const DPS310_PM_RATE_16: u8 = 0x40;  // 16 measurements per second
pub const DPS310_PM_RATE_32: u8 = 0x50;  // 32 measurements per second
pub const DPS310_PM_RATE_64: u8 = 0x60;  // 64 measurements per second
pub const DPS310_PM_RATE_128: u8 = 0x70; // 128 measurements per second

// DPS310 temperature measurement rate (samples per second)
pub const DPS310_TMP_RATE_1: u8 = 0x00;  // 1 measurement per second
pub const DPS310_TMP_RATE_2: u8 = 0x10;  // 2 measurements per second
pub const DPS310_TMP_RATE_4: u8 = 0x20;  // 4 measurements per second
pub const DPS310_TMP_RATE_8: u8 = 0x30;  // 8 measurements per second
pub const DPS310_TMP_RATE_16: u8 = 0x40; // 16 measurements per second
pub const DPS310_TMP_RATE_32: u8 = 0x50; // 32 measurements per second
pub const DPS310_TMP_RATE_64: u8 = 0x60; // 64 measurements per second
pub const DPS310_TMP_RATE_128: u8 = 0x70;// 128 measurements per second

// DPS310 pressure oversampling rates (OSR)
pub const DPS310_PM_PRC_1: u8 = 0x00;    // Single (2^0), default
pub const DPS310_PM_PRC_2: u8 = 0x01;    // 2 times (2^1)
pub const DPS310_PM_PRC_4: u8 = 0x02;    // 4 times (2^2)
pub const DPS310_PM_PRC_8: u8 = 0x03;    // 8 times (2^3)
pub const DPS310_PM_PRC_16: u8 = 0x04;   // 16 times (2^4)
pub const DPS310_PM_PRC_32: u8 = 0x05;   // 32 times (2^5)
pub const DPS310_PM_PRC_64: u8 = 0x06;   // 64 times (2^6)
pub const DPS310_PM_PRC_128: u8 = 0x07;  // 128 times (2^7)

// DPS310 temperature oversampling rates (OSR)
pub const DPS310_TMP_PRC_1: u8 = 0x00;   // Single (2^0), default
pub const DPS310_TMP_PRC_2: u8 = 0x01;   // 2 times (2^1)
pub const DPS310_TMP_PRC_4: u8 = 0x02;   // 4 times (2^2)
pub const DPS310_TMP_PRC_8: u8 = 0x03;   // 8 times (2^3)
pub const DPS310_TMP_PRC_16: u8 = 0x04;  // 16 times (2^4)
pub const DPS310_TMP_PRC_32: u8 = 0x05;  // 32 times (2^5)
pub const DPS310_TMP_PRC_64: u8 = 0x06;  // 64 times (2^6)
pub const DPS310_TMP_PRC_128: u8 = 0x07; // 128 times (2^7)

// DPS310 temperature sensor source
pub const DPS310_TMP_SRC_INTERNAL: u8 = 0x00;  // Internal sensor (MEMS)
pub const DPS310_TMP_SRC_EXTERNAL: u8 = 0x80;  // External sensor (ASIC)

// DPS310 interrupt settings
pub const DPS310_INT_DISABLE: u8 = 0x00;       // Disable interrupts
pub const DPS310_INT_PRESSURE: u8 = 0x01;      // Pressure measurement interrupt
pub const DPS310_INT_TEMPERATURE: u8 = 0x02;   // Temperature measurement interrupt
pub const DPS310_INT_BOTH: u8 = 0x03;          // Both measurements interrupt
pub const DPS310_INT_FIFO: u8 = 0x04;          // FIFO interrupt

// DPS310 config register shift
const DPS310_CFG_REG_SHIFT_7_TO_4: u8 = 0x04;
const DPS310_CFG_REG_SHIFT_BITS_3_2_1_0: u8 = 0x0F;

/// DPS310 configuration structure
#[derive(Debug, Clone, Copy)]
pub struct Dps310Config {
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
    
    /// Measurement mode (idle, continuous, etc.)
    pub measurement_mode: u8,
    
    /// Interrupt settings
    pub interrupt_settings: u8,
    
    /// Internal use for proper temperature compensation
    pub temp_shift: bool,
}

impl Default for Dps310Config {
    fn default() -> Self {
        Self {
            i2c_addr: DPS310_I2C_ADDR_PRIMARY,
            pressure_oversample: DPS310_PM_PRC_64,
            pressure_rate: DPS310_PM_RATE_16,
            temperature_oversample: DPS310_TMP_PRC_16,
            temperature_rate: DPS310_TMP_RATE_16,
            temperature_source: DPS310_TMP_SRC_INTERNAL,
            measurement_mode: DPS310_MODE_CONTINUOUS_BOTH,
            interrupt_settings: DPS310_INT_DISABLE,
            temp_shift: false,
        }
    }
}

/// DPS310 calibration coefficients
#[derive(Debug, Default)]
struct Dps310CalibData {
    c0: i32,    // 12-bit
    c1: i32,    // 12-bit
    c00: i32,   // 20-bit
    c10: i32,   // 20-bit
    c01: i32,   // 16-bit
    c11: i32,   // 16-bit
    c20: i32,   // 16-bit
    c21: i32,   // 16-bit
    c30: i32,   // 16-bit
    
    // Scaling factors
    k_p: f32,   // Pressure scaling factor
    k_t: f32,   // Temperature scaling factor
}

/// Raw temperature and pressure data
#[derive(Debug, Clone, Copy)]
struct Dps310RawData {
    raw_temp: i32,
    raw_pressure: i32,
    scaled_temp: f32,
    t_ready: bool,
    p_ready: bool,
}

impl Default for Dps310RawData {
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

/// DPS310 barometer driver
pub struct Dps310<I: I2cDevice> {
    /// Configuration for the sensor
    config: Dps310Config,
    /// I2C device for communication
    i2c: Option<I>,
    /// Calibration data for the sensor
    cal_data: Dps310CalibData,
    /// Most recent measurement data
    raw_data: Dps310RawData,
    /// Whether the sensor is initialized
    initialized: bool,
}

// Implement Send if the I2C device is Send
unsafe impl<I: I2cDevice + Send> Send for Dps310<I> {}

impl<I: I2cDevice> Dps310<I> {
    /// Create a new DPS310 instance
    pub fn new() -> Self {
        Self {
            config: Dps310Config::default(),
            i2c: None,
            cal_data: Dps310CalibData::default(),
            raw_data: Dps310RawData::default(),
            initialized: false,
        }
    }
    
    /// Create a new DPS310 instance with custom configuration
    pub fn new_with_config(config: Dps310Config) -> Self {
        Self {
            config,
            i2c: None,
            cal_data: Dps310CalibData::default(),
            raw_data: Dps310RawData::default(),
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
    
    /// Read the product ID
    async fn read_product_id(&mut self) -> Result<u8, ()> {
        self.read_reg(DPS310_REG_PROD_ID).await
    }
    
    /// Perform a soft reset of the sensor
    async fn soft_reset(&mut self) -> Result<(), ()> {
        self.write_reg(DPS310_REG_RESET, DPS310_RESET_CODE).await?;
        Timer::after(Duration::from_millis(40)).await;
        Ok(())
    }
    
    /// Read calibration coefficients from the sensor
    async fn read_calibration_coefficients(&mut self) -> Result<Dps310CalibData, ()> {
        // Read the raw coefficient data
        let mut coef_buf = [0u8; 18];
        self.read_regs(DPS310_REG_COEF, &mut coef_buf).await?;
        
        // Read the temperature coefficient source
        let _coef_src = self.read_reg(DPS310_REG_COEF_SRC).await?;
        
        // Calculate c0, c1 (12-bit values)
        let mut c0 = ((coef_buf[0] as u16) << 4) | ((coef_buf[1] as u16) >> 4);
        if c0 & 0x800 != 0 {
            c0 |= 0xF000; // Sign extension for negative values
        }
        
        let mut c1 = (((coef_buf[1] as u16) & 0x0F) << 8) | (coef_buf[2] as u16);
        if c1 & 0x800 != 0 {
            c1 |= 0xF000; // Sign extension
        }
        
        // Calculate c00, c10 (20-bit values)
        let mut c00 = ((coef_buf[3] as u32) << 12) | ((coef_buf[4] as u32) << 4) | ((coef_buf[5] as u32) >> 4);
        if c00 & 0x80000 != 0 {
            c00 |= 0xFFF00000; // Sign extension
        }
        
        let mut c10 = (((coef_buf[5] as u32) & 0x0F) << 16) | ((coef_buf[6] as u32) << 8) | (coef_buf[7] as u32);
        if c10 & 0x80000 != 0 {
            c10 |= 0xFFF00000; // Sign extension
        }
        
        // Calculate c01, c11, c20, c21, c30 (16-bit values)
        let c01 = ((coef_buf[8] as i16) << 8) | (coef_buf[9] as i16);
        let c11 = ((coef_buf[10] as i16) << 8) | (coef_buf[11] as i16);
        let c20 = ((coef_buf[12] as i16) << 8) | (coef_buf[13] as i16);
        let c21 = ((coef_buf[14] as i16) << 8) | (coef_buf[15] as i16);
        let c30 = ((coef_buf[16] as i16) << 8) | (coef_buf[17] as i16);
        
        // Create the calibration data structure
        let cal_data = Dps310CalibData {
            c0: c0 as i32,
            c1: c1 as i32,
            c00: c00 as i32,
            c10: c10 as i32,
            c01: c01 as i32,
            c11: c11 as i32,
            c20: c20 as i32,
            c21: c21 as i32,
            c30: c30 as i32,
            k_p: 1.0,
            k_t: 1.0,
        };
        
        Ok(cal_data)
    }
    
    /// Read raw temperature value
    async fn read_raw_temperature(&mut self) -> Result<i32, ()> {
        // If we're in one-shot mode, trigger a measurement
        if self.config.measurement_mode == DPS310_MODE_IDLE {
            // Configure for a single temperature measurement
            self.write_reg(DPS310_REG_MEAS_CFG, DPS310_MODE_TEMPERATURE_ONCE).await?;
            
            // Wait for measurement to complete
            let mut attempts = 0;
            while attempts < 20 {
                let status = self.read_reg(DPS310_REG_MEAS_CFG).await?;
                if status & 0x20 != 0 {
                    break; // Temperature measurement ready
                }
                Timer::after(Duration::from_millis(10)).await;
                attempts += 1;
            }
            
            if attempts >= 20 {
                return Err(()); // Timed out waiting for measurement
            }
        }
        
        // Read the temperature registers
        let mut temp_bytes = [0u8; 3];
        self.read_regs(DPS310_REG_TMP_B2, &mut temp_bytes).await?;
        
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
        // If we're in one-shot mode, trigger a measurement
        if self.config.measurement_mode == DPS310_MODE_IDLE {
            // Configure for a single pressure measurement
            self.write_reg(DPS310_REG_MEAS_CFG, DPS310_MODE_PRESSURE_ONCE).await?;
            
            // Wait for measurement to complete
            let mut attempts = 0;
            while attempts < 20 {
                let status = self.read_reg(DPS310_REG_MEAS_CFG).await?;
                if status & 0x10 != 0 {
                    break; // Pressure measurement ready
                }
                Timer::after(Duration::from_millis(10)).await;
                attempts += 1;
            }
            
            if attempts >= 20 {
                return Err(()); // Timed out waiting for measurement
            }
        }
        
        // Read the pressure registers
        let mut pressure_bytes = [0u8; 3];
        self.read_regs(DPS310_REG_PSR_B2, &mut pressure_bytes).await?;
        
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
        // Get the scaling factor based on oversampling
        let scaling_factor = match self.config.temperature_oversample {
            DPS310_TMP_PRC_1 => 524288.0,
            DPS310_TMP_PRC_2 => 1048576.0,
            DPS310_TMP_PRC_4 => 2097152.0,
            DPS310_TMP_PRC_8 => 4194304.0,
            DPS310_TMP_PRC_16 => 8388608.0,
            DPS310_TMP_PRC_32 => 16777216.0,
            DPS310_TMP_PRC_64 => 33554432.0,
            DPS310_TMP_PRC_128 => 67108864.0,
            _ => 524288.0, // Default to single measurement
        };
        
        // Apply calibration coefficients
        let scaled_temp = self.cal_data.c0 as f32 * 0.5 + 
                         self.cal_data.c1 as f32 * (raw_temp as f32 / scaling_factor);
        
        // Store the scaled temperature for pressure compensation
        self.raw_data.scaled_temp = scaled_temp;
        
        scaled_temp
    }
    
    /// Calculate compensated pressure from raw pressure and temperature
    fn calculate_compensated_pressure(&self, raw_pressure: i32, scaled_temp: f32) -> f32 {
        // Get the scaling factor based on oversampling
        let scaling_factor = match self.config.pressure_oversample {
            DPS310_PM_PRC_1 => 524288.0,
            DPS310_PM_PRC_2 => 1048576.0,
            DPS310_PM_PRC_4 => 2097152.0,
            DPS310_PM_PRC_8 => 4194304.0,
            DPS310_PM_PRC_16 => 8388608.0,
            DPS310_PM_PRC_32 => 16777216.0,
            DPS310_PM_PRC_64 => 33554432.0,
            DPS310_PM_PRC_128 => 67108864.0,
            _ => 524288.0, // Default to single measurement
        };
        
        // Apply calibration coefficients and temperature compensation
        let raw_scaled = raw_pressure as f32 / scaling_factor;
        
        let pressure = self.cal_data.c00 as f32 + 
                      raw_scaled * (self.cal_data.c10 as f32 + 
                                   raw_scaled * (self.cal_data.c20 as f32 + 
                                               raw_scaled * self.cal_data.c30 as f32)) +
                      scaled_temp * (self.cal_data.c01 as f32 + 
                                    raw_scaled * (self.cal_data.c11 as f32 + 
                                                raw_scaled * self.cal_data.c21 as f32));
        
        pressure
    }
}

impl<I: I2cDevice> BarometerChip for Dps310<I> {
    async fn init(&mut self) -> Result<(), ()> {
        // Make sure we have an I2C device
        if self.i2c.is_none() {
            return Err(());
        }
        
        // Reset the device
        self.soft_reset().await?;
        
        // Check product ID
        let prod_id = self.read_product_id().await?;
        if prod_id != DPS310_PROD_ID {
            return Err(());
        }
        
        // Wait for sensor to be ready
        let mut attempts = 0;
        while attempts < 10 {
            let status = self.read_reg(DPS310_REG_MEAS_CFG).await?;
            if status & DPS310_COEF_RDY != 0 {
                break;
            }
            Timer::after(Duration::from_millis(10)).await;
            attempts += 1;
        }
        
        if attempts >= 10 {
            return Err(());
        }
        
        // Read calibration coefficients
        self.cal_data = self.read_calibration_coefficients().await?;
        
        // Configure the temperature sensor
        let tmp_cfg = self.config.temperature_oversample | self.config.temperature_rate | self.config.temperature_source;
        self.write_reg(DPS310_REG_TMP_CFG, tmp_cfg).await?;
        
        // Configure the pressure sensor
        let prs_cfg = self.config.pressure_oversample | self.config.pressure_rate;
        self.write_reg(DPS310_REG_PRS_CFG, prs_cfg).await?;
        
        // Configure interrupts if enabled
        if self.config.interrupt_settings != DPS310_INT_DISABLE {
            self.write_reg(DPS310_REG_CFG_REG, self.config.interrupt_settings).await?;
        }
        
        // Set the measurement mode
        self.write_reg(DPS310_REG_MEAS_CFG, self.config.measurement_mode).await?;
        
        // Take initial readings to ensure system is working
        self.read_raw_temperature().await?;
        self.read_raw_pressure().await?;
        
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
        // For DPS310, calibration is primarily reading the factory coefficients
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
        
        // Convert Hz to DPS310 rate setting
        let (pressure_rate, temp_rate) = match rate_hz {
            1 => (DPS310_PM_RATE_1, DPS310_TMP_RATE_1),
            2 => (DPS310_PM_RATE_2, DPS310_TMP_RATE_2),
            r if r <= 4 => (DPS310_PM_RATE_4, DPS310_TMP_RATE_4),
            r if r <= 8 => (DPS310_PM_RATE_8, DPS310_TMP_RATE_8),
            r if r <= 16 => (DPS310_PM_RATE_16, DPS310_TMP_RATE_16),
            r if r <= 32 => (DPS310_PM_RATE_32, DPS310_TMP_RATE_32),
            r if r <= 64 => (DPS310_PM_RATE_64, DPS310_TMP_RATE_64),
            _ => (DPS310_PM_RATE_128, DPS310_TMP_RATE_128),
        };
        
        // Update configuration
        self.config.pressure_rate = pressure_rate;
        self.config.temperature_rate = temp_rate;
        
        // Apply new settings
        let tmp_cfg = self.config.temperature_oversample | temp_rate | self.config.temperature_source;
        self.write_reg(DPS310_REG_TMP_CFG, tmp_cfg).await?;
        
        let prs_cfg = self.config.pressure_oversample | pressure_rate;
        self.write_reg(DPS310_REG_PRS_CFG, prs_cfg).await?;
        
        // Return actual rate in Hz
        let actual_rate = match pressure_rate {
            DPS310_PM_RATE_1 => 1,
            DPS310_PM_RATE_2 => 2,
            DPS310_PM_RATE_4 => 4,
            DPS310_PM_RATE_8 => 8,
            DPS310_PM_RATE_16 => 16,
            DPS310_PM_RATE_32 => 32,
            DPS310_PM_RATE_64 => 64,
            DPS310_PM_RATE_128 => 128,
            _ => 1,
        };
        
        Ok(actual_rate)
    }
    
    fn chip_name(&self) -> &'static str {
        "DPS310"
    }
} 