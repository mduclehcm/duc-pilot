/// Barometer sensor interface

/// Barometer sensor interface
pub trait BaroSensor {
    /// Initialize the barometer sensor
    fn init(&mut self) -> bool;

    /// Get the current pressure in Pascals
    fn get_pressure(&self) -> f32;

    /// Get the current temperature in Celsius
    fn get_temperature(&self) -> f32;

    /// Get the estimated altitude in meters based on pressure
    fn get_altitude(&self) -> f32;
    
    /// Calibrate the barometer sensor
    /// 
    /// Returns true if calibration was successful
    fn calibrate(&mut self) -> bool;
    
    /// Perform sensor self-test
    /// 
    /// Returns true if the sensor passed the self-test
    fn self_test(&mut self) -> bool;
    
    /// Get the status of the sensor
    /// 
    /// Returns a BaroStatus struct containing sensor health information
    fn get_status(&self) -> BaroStatus;
    
    /// Set the sea level reference pressure in Pascals
    /// 
    /// This is used for altitude calculations
    fn set_sea_level_pressure(&mut self, pressure_pa: f32);
    
    /// Set the update rate in Hertz
    /// 
    /// Returns true if the rate was successfully set
    fn set_update_rate(&mut self, rate_hz: u8) -> bool;
    
    /// Reset the sensor to default state
    /// 
    /// Returns true if reset was successful
    fn reset(&mut self) -> bool;
}

/// Barometer sensor status information
#[derive(Debug, Clone, Copy)]
pub struct BaroStatus {
    /// Whether the sensor is currently functioning properly
    pub healthy: bool,
    
    /// Temperature of the sensor in Celsius
    pub temperature: f32,
    
    /// The current sample rate in Hz
    pub sample_rate: u8,
    
    /// Time since last successful reading in milliseconds
    pub last_reading_ms: u32,
    
    /// Whether the sensor is currently in calibration mode
    pub calibrating: bool,
}
