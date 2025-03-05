use nalgebra as na;
use std::time::Instant;

/// IMU data containing accelerometer, gyroscope and optional magnetometer measurements
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Accelerometer measurements in body frame (x, y, z) in m/s^2
    pub accel: na::Vector3<f32>,
    
    /// Gyroscope measurements in body frame (x, y, z) in rad/s
    pub gyro: na::Vector3<f32>,
    
    /// Optional magnetometer measurements in body frame (x, y, z) in gauss or tesla
    pub mag: Option<na::Vector3<f32>>,
    
    /// Timestamp when the measurements were taken
    pub timestamp: Instant,
}

/// GPS data containing position and velocity measurements
#[derive(Debug, Clone)]
pub struct GpsData {
    /// Position in NED frame (North, East, Down) in meters
    pub position: na::Vector3<f32>,
    
    /// Velocity in NED frame (North, East, Down) in meters per second
    pub velocity: na::Vector3<f32>,
    
    /// Accuracy estimates (position_accuracy, velocity_accuracy)
    pub accuracy: na::Vector2<f32>,
    
    /// Timestamp when the measurements were taken
    pub timestamp: Instant,
}

/// Barometer data containing altitude measurements
#[derive(Debug, Clone)]
pub struct BaroData {
    /// Altitude measurement in meters (positive up)
    pub altitude: f32,
    
    /// Accuracy estimate in meters
    pub accuracy: f32,
    
    /// Timestamp when the measurement was taken
    pub timestamp: Instant,
}

/// Magnetometer data containing magnetic field measurements
#[derive(Debug, Clone)]
pub struct MagData {
    /// Magnetic field vector in body frame (x, y, z) in gauss or tesla
    pub field: na::Vector3<f32>,
    
    /// Accuracy estimate
    pub accuracy: f32,
    
    /// Timestamp when the measurement was taken
    pub timestamp: Instant,
}

/// Function to transform body frame to NED frame
pub fn body_to_ned(vec: &na::Vector3<f32>, attitude: &na::UnitQuaternion<f32>) -> na::Vector3<f32> {
    attitude * vec
}

/// Function to transform NED frame to body frame
pub fn ned_to_body(vec: &na::Vector3<f32>, attitude: &na::UnitQuaternion<f32>) -> na::Vector3<f32> {
    attitude.inverse() * vec
} 