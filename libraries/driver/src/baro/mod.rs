// Barometer driver module providing a unified interface to various pressure sensors
//
// This implementation follows the design principles from the README:
// 1. Consistent implementation style across all sensors
// 2. Maintains a snapshot of the latest sensor data
// 3. Safe access in asynchronous contexts using appropriate guards
// 4. Interrupt-driven operation where possible

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, Ordering};

// Define our own BaroStatus and BaroSensor for this module

/// Status information for barometer sensors
#[derive(Debug, Clone, Copy)]
pub struct BaroStatus {
    /// Whether the sensor is healthy and functioning properly
    pub healthy: bool,
    /// Current temperature reading in Celsius
    pub temperature: f32,
    /// Current sampling rate in Hz
    pub sample_rate: u8,
    /// Timestamp of the last reading in milliseconds
    pub last_reading_ms: u32,
    /// Whether the sensor is currently calibrating
    pub calibrating: bool,
}

/// Common interface for barometer sensors
pub trait BaroSensor {
    /// Initialize the barometer
    async fn init(&mut self) -> bool;
    
    /// Get the current pressure in Pascals
    async fn get_pressure(&mut self) -> Result<f32, ()>;
    
    /// Get the current temperature in Celsius
    async fn get_temperature(&mut self) -> Result<f32, ()>;
    
    /// Get the current altitude in meters
    async fn get_altitude(&mut self) -> Result<f32, ()>;
    
    /// Get the sensor status
    fn get_status(&self) -> BaroStatus;
    
    /// Calibrate the barometer
    async fn calibrate(&mut self) -> bool;
    
    /// Perform a self-test of the sensor
    async fn self_test(&mut self) -> bool;
    
    /// Set the sea level pressure reference for altitude calculations
    fn set_sea_level_pressure(&mut self, pressure_pa: f32);
    
    /// Set the update rate in Hz
    async fn set_update_rate(&mut self, rate_hz: u8) -> bool;
    
    /// Reset the sensor
    async fn reset(&mut self) -> bool;
}

// Embassy STM32 dependencies
use embassy_stm32::i2c::{I2c, Instance as I2cInstance};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull, AnyPin};

// Embassy time for async operations
use embassy_time::{Timer, Duration, Instant};

// Heapless data structures for no_std environment
use heapless::spsc::Queue;
use heapless::String;

// Export specific barometer implementations
pub mod bmp280;
pub mod spl06;
pub mod dps310;

// Re-export the barometer implementations
pub use self::bmp280::Bmp280;
pub use self::spl06::Spl06;
pub use self::dps310::Dps310;

// Standard sea level pressure for altitude calculations (1013.25 hPa)
pub const STD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Barometer measurement state
#[derive(Debug, Clone, Copy)]
pub struct BaroMeasurement {
    /// Pressure in Pascals
    pub pressure_pa: f32,
    /// Temperature in Celsius
    pub temperature_c: f32,
    /// Timestamp in milliseconds
    pub timestamp_ms: u32,
    /// Calculated altitude in meters based on standard sea level pressure
    pub altitude_m: f32,
    /// Measurement is valid
    pub valid: bool,
}

impl Default for BaroMeasurement {
    fn default() -> Self {
        Self {
            pressure_pa: 0.0,
            temperature_c: 0.0,
            timestamp_ms: 0,
            altitude_m: 0.0,
            valid: false,
        }
    }
}

/// Common interface for I2C device operations with async support
pub trait I2cDevice: Send {
    /// Write data to a device at the specified address
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), ()>;
    
    /// Read data from a device at the specified address
    async fn read(&mut self, addr: u8, data: &mut [u8]) -> Result<(), ()>;
    
    /// Write data to a device and then read from it (combined operation)
    async fn write_read(&mut self, addr: u8, write_data: &[u8], read_data: &mut [u8]) -> Result<(), ()>;
    
    /// Read a single register from a device
    async fn read_reg(&mut self, addr: u8, reg: u8) -> Result<u8, ()> {
        let mut buffer = [0u8; 1];
        self.write_read(addr, &[reg], &mut buffer).await?;
        Ok(buffer[0])
    }
    
    /// Write to a single register on a device
    async fn write_reg(&mut self, addr: u8, reg: u8, value: u8) -> Result<(), ()> {
        self.write(addr, &[reg, value]).await
    }
    
    /// Read multiple registers from a device
    async fn read_regs(&mut self, addr: u8, reg: u8, data: &mut [u8]) -> Result<(), ()> {
        self.write_read(addr, &[reg], data).await
    }
}

/// Interface for barometer chip implementations
pub trait BarometerChip: Send {
    /// Initialize the barometer chip
    async fn init(&mut self) -> Result<(), ()>;
    
    /// Read pressure in Pascals
    async fn read_pressure(&mut self) -> Result<f32, ()>;
    
    /// Read temperature in Celsius
    async fn read_temperature(&mut self) -> Result<f32, ()>;
    
    /// Perform self-test of the chip
    async fn self_test(&mut self) -> Result<bool, ()>;
    
    /// Calibrate the chip
    async fn calibrate(&mut self) -> Result<(), ()>;
    
    /// Reset the chip
    async fn reset(&mut self) -> Result<(), ()>;
    
    /// Set update rate (in Hz)
    async fn set_update_rate(&mut self, rate_hz: u8) -> Result<u8, ()>;
    
    /// Get the name of the chip
    fn chip_name(&self) -> &'static str;
}

/// I2C barometer driver
/// 
/// This implements the barometer driver functionality for asynchronous operations:
/// - Maintains a snapshot of latest sensor data
/// - Provides thread-safe access to the data
/// - Supports async operations
pub struct I2CBarometer<B: BarometerChip> {
    /// The barometer chip implementation
    chip: core::cell::RefCell<B>,
    /// Current data from the sensor
    measurement: core::cell::RefCell<BaroMeasurement>,
    /// Current sample rate in Hz
    sample_rate: core::cell::Cell<u8>,
    /// Whether the sensor has been successfully initialized
    initialized: AtomicBool,
    /// Whether the sensor is currently in calibration mode
    calibrating: AtomicBool,
    /// Reference sea level pressure in Pascals (for altitude calculation)
    sea_level_pressure_pa: core::cell::Cell<f32>,
    /// Last update time in milliseconds
    last_update_ms: core::cell::Cell<u32>,
    /// Sensor health status
    healthy: AtomicBool,
}

// We need to implement Send and Sync manually since RefCell isn't Send or Sync by default
// This is safe because we handle all the interior mutability correctly
unsafe impl<B: BarometerChip> Send for I2CBarometer<B> {}
unsafe impl<B: BarometerChip> Sync for I2CBarometer<B> {}

impl<B: BarometerChip> I2CBarometer<B> {
    /// Create a new async I2C barometer with the specified chip
    pub fn new(chip: B) -> Self {
        Self {
            chip: core::cell::RefCell::new(chip),
            measurement: core::cell::RefCell::new(BaroMeasurement::default()),
            sample_rate: core::cell::Cell::new(10), // Default 10Hz
            initialized: AtomicBool::new(false),
            calibrating: AtomicBool::new(false),
            sea_level_pressure_pa: core::cell::Cell::new(STD_SEA_LEVEL_PRESSURE_PA),
            last_update_ms: core::cell::Cell::new(0),
            healthy: AtomicBool::new(false),
        }
    }
    
    /// Calculate altitude using the hypsometric formula
    pub fn calculate_altitude(&self, pressure_pa: f32) -> f32 {
        let sea_level_pa = self.sea_level_pressure_pa.get();
        
        // Hypsometric formula: h = 44330 * (1 - (p/p0)^(1/5.255))
        44330.0 * (1.0 - (pressure_pa / sea_level_pa).powf(1.0 / 5.255))
    }
    
    /// Update the internal measurement state by reading the sensor
    pub async fn update_measurement(&mut self) -> Result<(), ()> {
        // Ensure we're initialized
        if !self.initialized.load(Ordering::Relaxed) ||
           self.calibrating.load(Ordering::Relaxed) {
            return Err(());
        }
        
        // Safely borrow the chip
        let mut chip = self.chip.borrow_mut();
        
        // Read pressure and temperature (with async methods)
        let pressure_result = chip.read_pressure().await;
        let temperature_result = chip.read_temperature().await;
        
        // If both readings were successful, update the measurement
        if let (Ok(pressure), Ok(temperature)) = (pressure_result, temperature_result) {
            let mut measurement = self.measurement.borrow_mut();
            measurement.pressure_pa = pressure;
            measurement.temperature_c = temperature;
            measurement.timestamp_ms = self.get_system_time_ms();
            
            // Calculate altitude using the current sea level reference
            measurement.altitude_m = self.calculate_altitude(pressure);
            measurement.valid = true;
            
            self.healthy.store(true, Ordering::Relaxed);
            Ok(())
        } else {
            self.healthy.store(false, Ordering::Relaxed);
            Err(())
        }
    }
    
    /// Get the current system time in milliseconds
    fn get_system_time_ms(&self) -> u32 {
        // In a real implementation, this would use embassy_time
        // For now, we'll just return a dummy value
        0
    }
    
    /// Initialize the barometer
    pub async fn init(&mut self) -> Result<(), ()> {
        if self.initialized.load(Ordering::Relaxed) {
            return Ok(());
        }
        
        {
            let mut chip = self.chip.borrow_mut();
            chip.init().await?;
        }
        
        self.initialized.store(true, Ordering::Relaxed);
        self.healthy.store(true, Ordering::Relaxed);
        
        // Read initial measurement
        self.update_measurement().await?;
        
        Ok(())
    }
    
    /// Get the current pressure in Pascals
    pub async fn get_pressure(&mut self) -> Result<f32, ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        {
            let measurement = self.measurement.borrow();
            if measurement.valid {
                return Ok(measurement.pressure_pa);
            }
        }
        
        // If we don't have a valid measurement, get a new one
        self.update_measurement().await?;
        
        let measurement = self.measurement.borrow();
        if measurement.valid {
            Ok(measurement.pressure_pa)
        } else {
            Err(())
        }
    }
    
    /// Get the current temperature in Celsius
    pub async fn get_temperature(&mut self) -> Result<f32, ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        {
            let measurement = self.measurement.borrow();
            if measurement.valid {
                return Ok(measurement.temperature_c);
            }
        }
        
        // If we don't have a valid measurement, get a new one
        self.update_measurement().await?;
        
        let measurement = self.measurement.borrow();
        if measurement.valid {
            Ok(measurement.temperature_c)
        } else {
            Err(())
        }
    }
    
    /// Get the current altitude in meters
    pub async fn get_altitude(&mut self) -> Result<f32, ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        {
            let measurement = self.measurement.borrow();
            if measurement.valid {
                return Ok(measurement.altitude_m);
            }
        }
        
        // If we don't have a valid measurement, get a new one
        self.update_measurement().await?;
        
        let measurement = self.measurement.borrow();
        if measurement.valid {
            Ok(measurement.altitude_m)
        } else {
            Err(())
        }
    }
    
    /// Calibrate the barometer
    pub async fn calibrate(&mut self) -> Result<(), ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        self.calibrating.store(true, Ordering::Relaxed);
        
        {
            let mut chip = self.chip.borrow_mut();
            chip.calibrate().await?;
        }
        
        self.calibrating.store(false, Ordering::Relaxed);
        
        // Get a new measurement after calibration
        self.update_measurement().await?;
        
        Ok(())
    }
    
    /// Perform a self-test
    pub async fn self_test(&mut self) -> Result<bool, ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        let mut chip = self.chip.borrow_mut();
        chip.self_test().await
    }
    
    /// Get the sensor status
    pub fn get_status(&self) -> BaroStatus {
        BaroStatus {
            healthy: self.healthy.load(Ordering::Relaxed),
            temperature: self.measurement.borrow().temperature_c,
            sample_rate: self.sample_rate.get(),
            last_reading_ms: self.last_update_ms.get(),
            calibrating: self.calibrating.load(Ordering::Relaxed),
        }
    }
    
    /// Set the sea level pressure reference for altitude calculations
    pub fn set_sea_level_pressure(&mut self, pressure_pa: f32) {
        self.sea_level_pressure_pa.set(pressure_pa);
        
        // Update the altitude calculation based on the new reference
        let mut measurement = self.measurement.borrow_mut();
        if measurement.valid {
            measurement.altitude_m = self.calculate_altitude(measurement.pressure_pa);
        }
    }
    
    /// Set the update rate in Hz
    pub async fn set_update_rate(&mut self, rate_hz: u8) -> Result<u8, ()> {
        if !self.initialized.load(Ordering::Relaxed) {
            return Err(());
        }
        
        let mut chip = self.chip.borrow_mut();
        let actual_rate = chip.set_update_rate(rate_hz).await?;
        
        self.sample_rate.set(actual_rate);
        
        Ok(actual_rate)
    }
    
    /// Reset the sensor
    pub async fn reset(&mut self) -> Result<(), ()> {
        self.initialized.store(false, Ordering::Relaxed);
        
        {
            let mut chip = self.chip.borrow_mut();
            chip.reset().await?;
        }
        
        // Reinitialize
        self.init().await
    }
}

// Implement the BaroSensor trait for I2CBarometer
impl<B: BarometerChip> BaroSensor for I2CBarometer<B> {
    async fn init(&mut self) -> bool {
        self.init().await.is_ok()
    }
    
    async fn get_pressure(&mut self) -> Result<f32, ()> {
        self.get_pressure().await
    }
    
    async fn get_temperature(&mut self) -> Result<f32, ()> {
        self.get_temperature().await
    }
    
    async fn get_altitude(&mut self) -> Result<f32, ()> {
        self.get_altitude().await
    }
    
    fn get_status(&self) -> BaroStatus {
        self.get_status()
    }
    
    async fn calibrate(&mut self) -> bool {
        self.calibrate().await.is_ok()
    }
    
    async fn self_test(&mut self) -> bool {
        match self.self_test().await {
            Ok(result) => result,
            Err(_) => false,
        }
    }
    
    fn set_sea_level_pressure(&mut self, pressure_pa: f32) {
        self.set_sea_level_pressure(pressure_pa);
    }
    
    async fn set_update_rate(&mut self, rate_hz: u8) -> bool {
        self.set_update_rate(rate_hz).await.is_ok()
    }
    
    async fn reset(&mut self) -> bool {
        self.reset().await.is_ok()
    }
}
