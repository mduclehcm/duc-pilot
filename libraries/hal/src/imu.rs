/// IMU (Inertial Measurement Unit) sensor interface
use crate::types::Vector3d;

/// IMU (Inertial Measurement Unit) interface
pub trait ImuSensor {
    /// Initialize the IMU sensor
    fn init(&mut self) -> bool;

    /// Get acceleration data (in m/s²)
    fn get_acceleration(&self) -> Vector3d;

    /// Get gyroscope data (in rad/s)
    fn get_gyro(&self) -> Vector3d;

    /// Get magnetometer data (in μT), if available
    fn get_mag(&self) -> Vector3d;
    
    /// Calibrate the IMU sensors
    /// 
    /// Returns true if calibration was successful
    fn calibrate(&mut self) -> bool;
    
    /// Calibrate the accelerometer specifically
    /// 
    /// The device should be held in 6 different orientations during this process
    /// Returns true if calibration was successful
    fn calibrate_accel(&mut self) -> bool;
    
    /// Calibrate the gyroscope specifically
    /// 
    /// The device should remain stationary during this process
    /// Returns true if calibration was successful
    fn calibrate_gyro(&mut self) -> bool;
    
    /// Calibrate the magnetometer specifically
    /// 
    /// The device should be rotated in a figure-8 pattern during this process
    /// Returns true if calibration was successful
    fn calibrate_mag(&mut self) -> bool;
    
    /// Get the current temperature of the IMU in Celsius
    fn get_temperature(&self) -> f32;
    
    /// Set the accelerometer bias correction
    /// 
    /// Returns true if the bias was successfully set
    fn set_accel_bias(&mut self, bias: Vector3d) -> bool;
    
    /// Set the gyroscope bias correction
    /// 
    /// Returns true if the bias was successfully set
    fn set_gyro_bias(&mut self, bias: Vector3d) -> bool;
    
    /// Set the magnetometer bias correction
    /// 
    /// Returns true if the bias was successfully set
    fn set_mag_bias(&mut self, bias: Vector3d) -> bool;
    
    /// Get the current accelerometer bias correction
    fn get_accel_bias(&self) -> Vector3d;
    
    /// Get the current gyroscope bias correction
    fn get_gyro_bias(&self) -> Vector3d;
    
    /// Get the current magnetometer bias correction
    fn get_mag_bias(&self) -> Vector3d;
    
    /// Set the filter bandwidth for the sensors
    /// 
    /// Returns true if the bandwidth was successfully set
    fn set_filter_bandwidth(&mut self, bandwidth_hz: u16) -> bool;
    
    /// Set the sample rate in Hz
    /// 
    /// Returns true if the rate was successfully set
    fn set_sample_rate(&mut self, rate_hz: u16) -> bool;
    
    /// Check if the IMU sensor is healthy and operating correctly
    fn is_healthy(&self) -> bool;
    
    /// Reset the IMU sensor to default state
    /// 
    /// Returns true if reset was successful
    fn reset(&mut self) -> bool;
    
    /// Get detailed information about the IMU status
    fn get_status(&self) -> ImuStatus;
}

/// IMU sensor status information
#[derive(Debug, Clone, Copy)]
pub struct ImuStatus {
    /// Whether the accelerometer is currently functioning properly
    pub accel_healthy: bool,
    
    /// Whether the gyroscope is currently functioning properly
    pub gyro_healthy: bool,
    
    /// Whether the magnetometer is currently functioning properly
    pub mag_healthy: bool,
    
    /// Temperature of the sensor in Celsius
    pub temperature: f32,
    
    /// The current sample rate in Hz
    pub sample_rate: u16,
    
    /// Time since last successful reading in milliseconds
    pub last_reading_ms: u32,
    
    /// Whether the sensor is currently in calibration mode
    pub calibrating: bool,
}
