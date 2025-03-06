
use crate::baro::{BarometerChip, I2cDevice};

// BMP280 I2C addresses (depends on SDO pin state)
pub const BMP280_I2C_ADDR_PRIMARY: u8 = 0x76;
pub const BMP280_I2C_ADDR_SECONDARY: u8 = 0x77;

// Register addresses
pub const BMP280_REG_ID: u8 = 0xD0;
pub const BMP280_REG_RESET: u8 = 0xE0;
pub const BMP280_REG_STATUS: u8 = 0xF3;
pub const BMP280_REG_CTRL_MEAS: u8 = 0xF4;
pub const BMP280_REG_CONFIG: u8 = 0xF5;
pub const BMP280_REG_PRESSURE_MSB: u8 = 0xF7;
pub const BMP280_REG_PRESSURE_LSB: u8 = 0xF8;
pub const BMP280_REG_PRESSURE_XLSB: u8 = 0xF9;
pub const BMP280_REG_TEMP_MSB: u8 = 0xFA;
pub const BMP280_REG_TEMP_LSB: u8 = 0xFB;
pub const BMP280_REG_TEMP_XLSB: u8 = 0xFC;
pub const BMP280_REG_CALIB_START: u8 = 0x88;
pub const BMP280_REG_CALIB_LENGTH: usize = 24;

// Chip ID for verification
pub const BMP280_CHIP_ID: u8 = 0x58;

// Reset command
pub const BMP280_RESET_CMD: u8 = 0xB6;

// Filter settings (IIR filter coefficient)
pub const BMP280_FILTER_OFF: u8 = 0x00;
pub const BMP280_FILTER_2: u8 = 0x01;
pub const BMP280_FILTER_4: u8 = 0x02;
pub const BMP280_FILTER_8: u8 = 0x03;
pub const BMP280_FILTER_16: u8 = 0x04;

// Oversampling settings for temperature and pressure
pub const BMP280_OVERSAMPLING_SKIP: u8 = 0x00;
pub const BMP280_OVERSAMPLING_1X: u8 = 0x01;
pub const BMP280_OVERSAMPLING_2X: u8 = 0x02;
pub const BMP280_OVERSAMPLING_4X: u8 = 0x03;
pub const BMP280_OVERSAMPLING_8X: u8 = 0x04;
pub const BMP280_OVERSAMPLING_16X: u8 = 0x05;

// Power mode settings
pub const BMP280_POWER_SLEEP: u8 = 0x00;
pub const BMP280_POWER_FORCED: u8 = 0x01;
pub const BMP280_POWER_NORMAL: u8 = 0x03;

// Standby time settings (for normal power mode)
pub const BMP280_STANDBY_0_5_MS: u8 = 0x00;
pub const BMP280_STANDBY_62_5_MS: u8 = 0x01;
pub const BMP280_STANDBY_125_MS: u8 = 0x02;
pub const BMP280_STANDBY_250_MS: u8 = 0x03;
pub const BMP280_STANDBY_500_MS: u8 = 0x04;
pub const BMP280_STANDBY_1000_MS: u8 = 0x05;
pub const BMP280_STANDBY_2000_MS: u8 = 0x06;
pub const BMP280_STANDBY_4000_MS: u8 = 0x07;

/// Configuration for BMP280 sensor
/// Allows customizing various aspects of the sensor's behavior
#[derive(Clone, Copy)]
pub struct Bmp280Config {
    /// I2C address of the BMP280 (0x76 or 0x77)
    pub i2c_addr: u8,
    
    /// Temperature oversampling setting
    pub temp_oversampling: u8,
    
    /// Pressure oversampling setting
    pub pressure_oversampling: u8,
    
    /// IIR filter coefficient
    pub filter_coefficient: u8,
    
    /// Standby time between measurements in normal mode
    pub standby_time: u8,
    
    /// Power mode (sleep, forced, normal)
    pub power_mode: u8,
}

impl Default for Bmp280Config {
    fn default() -> Self {
        Self {
            i2c_addr: BMP280_I2C_ADDR_PRIMARY,
            temp_oversampling: BMP280_OVERSAMPLING_2X,
            pressure_oversampling: BMP280_OVERSAMPLING_16X,
            filter_coefficient: BMP280_FILTER_16,
            standby_time: BMP280_STANDBY_250_MS,
            power_mode: BMP280_POWER_NORMAL,
        }
    }
}

/// BMP280 calibration data
#[derive(Debug, Clone, Copy)]
struct Bmp280CalibData {
    // Temperature compensation
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    
    // Pressure compensation
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    
    // Intermediate value used in calculations
    t_fine: i32,
}

/// Structure to hold raw sensor measurements
#[derive(Debug, Clone, Copy)]
struct RawMeasurements {
    /// Raw pressure value from sensor
    pressure: u32,
    /// Raw temperature value from sensor
    temperature: u32,
}

/// BMP280 barometer implementation
pub struct Bmp280<I: I2cDevice> {
    /// BMP280 configuration
    config: Bmp280Config,
    /// I2C device for communication
    i2c: Option<I>,
    /// I2C address of the device
    addr: u8,
    /// Calibration data
    cal: Option<Bmp280CalibData>,
    /// Current measurements
    measurements: Option<RawMeasurements>,
}

// Implement Send for the driver if the I2C device is Send
unsafe impl<I: I2cDevice + Send> Send for Bmp280<I> {}

impl<I: I2cDevice> Bmp280<I> {
    /// Create a new BMP280 driver with the specified I2C address
    pub fn new(addr: u8) -> Self {
        Self {
            config: Bmp280Config::default(),
            i2c: None,
            addr,
            cal: None,
            measurements: None,
        }
    }
    
    /// Create a new BMP280 driver with the primary I2C address (0x76)
    pub fn new_primary() -> Self {
        Self::new(BMP280_I2C_ADDR_PRIMARY)
    }
    
    /// Create a new BMP280 driver with custom configuration
    pub fn new_with_config(config: Bmp280Config) -> Self {
        Self {
            config,
            i2c: None,
            addr: BMP280_I2C_ADDR_PRIMARY,
            cal: None,
            measurements: None,
        }
    }
    
    /// Set the I2C device for communication
    pub fn set_i2c(&mut self, i2c: I) {
        self.i2c = Some(i2c);
    }
    
    /// Read a register
    async fn read_register(&mut self, reg: u8) -> Result<u8, ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.read_reg(self.addr, reg).await
        } else {
            Err(())
        }
    }
    
    /// Write to a register
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.write_reg(self.addr, reg, value).await
        } else {
            Err(())
        }
    }
    
    /// Read multiple registers
    async fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> Result<(), ()> {
        if let Some(i2c) = &mut self.i2c {
            i2c.read_regs(self.addr, reg, data).await
        } else {
            Err(())
        }
    }
    
    /// Read the chip ID
    async fn read_chip_id(&mut self) -> Result<u8, ()> {
        self.read_register(BMP280_REG_ID).await
    }
    
    /// Read calibration data
    async fn read_calibration_data(&mut self) -> Result<Bmp280CalibData, ()> {
        let mut buffer = [0u8; BMP280_REG_CALIB_LENGTH];
        self.read_registers(BMP280_REG_CALIB_START, &mut buffer).await?;
        
        let cal = Bmp280CalibData {
            dig_t1: u16::from_le_bytes([buffer[0], buffer[1]]),
            dig_t2: i16::from_le_bytes([buffer[2], buffer[3]]),
            dig_t3: i16::from_le_bytes([buffer[4], buffer[5]]),
            dig_p1: u16::from_le_bytes([buffer[6], buffer[7]]),
            dig_p2: i16::from_le_bytes([buffer[8], buffer[9]]),
            dig_p3: i16::from_le_bytes([buffer[10], buffer[11]]),
            dig_p4: i16::from_le_bytes([buffer[12], buffer[13]]),
            dig_p5: i16::from_le_bytes([buffer[14], buffer[15]]),
            dig_p6: i16::from_le_bytes([buffer[16], buffer[17]]),
            dig_p7: i16::from_le_bytes([buffer[18], buffer[19]]),
            dig_p8: i16::from_le_bytes([buffer[20], buffer[21]]),
            dig_p9: i16::from_le_bytes([buffer[22], buffer[23]]),
            t_fine: 0,
        };
        
        Ok(cal)
    }
    
    /// Read raw measurements
    async fn read_raw_measurements(&mut self) -> Result<RawMeasurements, ()> {
        let mut buffer = [0u8; 6];
        self.read_registers(BMP280_REG_PRESSURE_MSB, &mut buffer).await?;
        
        let raw_pressure = ((buffer[0] as u32) << 12) | ((buffer[1] as u32) << 4) | ((buffer[2] as u32) >> 4);
        let raw_temperature = ((buffer[3] as u32) << 12) | ((buffer[4] as u32) << 4) | ((buffer[5] as u32) >> 4);
        
        Ok(RawMeasurements {
            pressure: raw_pressure,
            temperature: raw_temperature,
        })
    }
    
    /// Calculate temperature from raw measurement
    fn calculate_temperature(&self, raw_temp: u32, cal: &mut Bmp280CalibData) -> f32 {
        // Temperature calculation based on BMP280 datasheet
        let var1 = (((raw_temp as f64) / 16384.0) - ((cal.dig_t1 as f64) / 1024.0)) * (cal.dig_t2 as f64);
        let var2 = ((((raw_temp as f64) / 131072.0) - ((cal.dig_t1 as f64) / 8192.0)) * 
                   (((raw_temp as f64) / 131072.0) - ((cal.dig_t1 as f64) / 8192.0))) * (cal.dig_t3 as f64);
        
        let t_fine = (var1 + var2) as i32;
        cal.t_fine = t_fine;
        
        let temperature = (var1 + var2) / 5120.0;
        
        temperature as f32
    }
    
    /// Calculate pressure from raw measurement
    fn calculate_pressure(&self, raw_pressure: u32, cal: &Bmp280CalibData) -> f32 {
        // Pressure calculation based on BMP280 datasheet
        let var1 = (cal.t_fine as f64 / 2.0) - 64000.0;
        let var2 = var1 * var1 * (cal.dig_p6 as f64) / 32768.0;
        let var2 = var2 + var1 * (cal.dig_p5 as f64) * 2.0;
        let var2 = (var2 / 4.0) + ((cal.dig_p4 as f64) * 65536.0);
        let var1 = ((cal.dig_p3 as f64) * var1 * var1 / 524288.0 + (cal.dig_p2 as f64) * var1) / 524288.0;
        let var1 = (1.0 + var1 / 32768.0) * (cal.dig_p1 as f64);
        
        if var1.abs() < 0.0000001 {
            return 0.0; // Avoid division by zero
        }
        
        let pressure = 1048576.0 - (raw_pressure as f64);
        let pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1;
        let var1 = (cal.dig_p9 as f64) * pressure * pressure / 2147483648.0;
        let var2 = pressure * (cal.dig_p8 as f64) / 32768.0;
        let pressure = pressure + (var1 + var2 + (cal.dig_p7 as f64)) / 16.0;
        
        // Convert from hPa to Pa
        (pressure as f32) * 100.0
    }
}

impl<I: I2cDevice> BarometerChip for Bmp280<I> {
    async fn init(&mut self) -> Result<(), ()> {
        // Verify chip ID
        let chip_id = self.read_chip_id().await?;
        if chip_id != BMP280_CHIP_ID {
            return Err(());
        }
        
        // Read calibration data
        let cal = self.read_calibration_data().await?;
        self.cal = Some(cal);
        
        // Configure the sensor according to our settings
        let ctrl_meas = ((self.config.temp_oversampling as u8) << 5) |
                        ((self.config.pressure_oversampling as u8) << 2) |
                        (self.config.power_mode as u8);
                        
        let config = ((self.config.standby_time as u8) << 5) |
                     ((self.config.filter_coefficient as u8) << 2);
                     
        // Reset the device first
        self.write_register(BMP280_REG_RESET, BMP280_RESET_CMD).await?;
        
        // Wait a bit for the reset to complete
        // In a real implementation, we'd use embassy_time::Timer here
        // For now we'll use a spin loop
        for _ in 0..1000 { core::hint::spin_loop(); }
        
        // Configure the device
        self.write_register(BMP280_REG_CONFIG, config).await?;
        self.write_register(BMP280_REG_CTRL_MEAS, ctrl_meas).await?;
        
        Ok(())
    }
    
    async fn read_pressure(&mut self) -> Result<f32, ()> {
        if self.cal.is_none() {
            return Err(());
        }
        
        let measurements = self.read_raw_measurements().await?;
        let mut cal = self.cal.unwrap();
        
        // We need to calculate temperature first to get t_fine
        self.calculate_temperature(measurements.temperature, &mut cal);
        
        // Now we can calculate pressure
        let pressure = self.calculate_pressure(measurements.pressure, &cal);
        
        // Update the calibration data with the new t_fine value
        self.cal = Some(cal);
        
        Ok(pressure)
    }
    
    async fn read_temperature(&mut self) -> Result<f32, ()> {
        if self.cal.is_none() {
            return Err(());
        }
        
        let measurements = self.read_raw_measurements().await?;
        let mut cal = self.cal.unwrap();
        
        let temperature = self.calculate_temperature(measurements.temperature, &mut cal);
        
        // Update the calibration data with the new t_fine value
        self.cal = Some(cal);
        
        Ok(temperature)
    }
    
    async fn self_test(&mut self) -> Result<bool, ()> {
        // Verify chip ID as a simple self-test
        let chip_id = self.read_chip_id().await?;
        Ok(chip_id == BMP280_CHIP_ID)
    }
    
    async fn calibrate(&mut self) -> Result<(), ()> {
        // The BMP280 doesn't require runtime calibration
        Ok(())
    }
    
    async fn reset(&mut self) -> Result<(), ()> {
        self.write_register(BMP280_REG_RESET, BMP280_RESET_CMD).await?;
        
        // Wait a bit for the reset to complete
        // In a real implementation, we'd use embassy_time::Timer here
        for _ in 0..1000 { core::hint::spin_loop(); }
        
        Ok(())
    }
    
    async fn set_update_rate(&mut self, rate_hz: u8) -> Result<u8, ()> {
        // BMP280 doesn't directly support setting a specific update rate
        // We'd need to map the desired rate to appropriate standby time
        // For now, we're just accepting the requested rate
        Ok(rate_hz)
    }
    
    fn chip_name(&self) -> &'static str {
        "BMP280"
    }
}
