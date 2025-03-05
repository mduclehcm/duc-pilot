use nalgebra as na;
#[cfg(feature = "std")]
use std::time::Instant;
#[cfg(feature = "embassy")]
use embassy_time::{Instant, Duration};

/// Timestamp type - using f32 in seconds for all timestamps
pub type TimeStamp = f32;

/// Time conversion utilities
#[cfg(feature = "std")]
pub mod time_convert {
    use std::time::Instant;
    use super::TimeStamp;
    
    /// Base time for std::time::Instant conversions
    static mut BASE_TIME: Option<Instant> = None;
    
    /// Convert from std::time::Instant to TimeStamp (f32 seconds)
    pub fn instant_to_timestamp(instant: &Instant) -> TimeStamp {
        unsafe {
            if BASE_TIME.is_none() {
                BASE_TIME = Some(*instant);
            }
            
            if let Some(base) = BASE_TIME {
                instant.duration_since(base).as_secs_f32()
            } else {
                // Fallback - should never happen due to check above
                instant.elapsed().as_secs_f32()
            }
        }
    }
    
    /// Get current time as timestamp
    pub fn now() -> TimeStamp {
        instant_to_timestamp(&Instant::now())
    }
}

/// Time conversion utilities for Embassy
#[cfg(feature = "embassy")]
pub mod time_convert {
    use embassy_time::{Instant, Duration};
    use super::TimeStamp;
    
    /// Base time for embassy_time::Instant conversions
    static mut BASE_TIME: Option<Instant> = None;
    
    /// Convert from embassy_time::Instant to TimeStamp (f32 seconds)
    pub fn instant_to_timestamp(instant: &Instant) -> TimeStamp {
        unsafe {
            if BASE_TIME.is_none() {
                BASE_TIME = Some(*instant);
            }
            
            if let Some(base) = BASE_TIME {
                let duration = instant.duration_since(base);
                duration.as_micros() as f32 / 1_000_000.0
            } else {
                // Fallback - should never happen due to check above
                instant.as_micros() as f32 / 1_000_000.0
            }
        }
    }
    
    /// Get current time as timestamp
    pub fn now() -> TimeStamp {
        instant_to_timestamp(&Instant::now())
    }
}

/// Bare minimum time utilities for no_std without embassy
#[cfg(not(any(feature = "std", feature = "embassy")))]
pub mod time_convert {
    use super::TimeStamp;
    
    /// Global counter for basic timing
    static mut COUNTER: TimeStamp = 0.0;
    
    /// Get current time as timestamp (increments counter)
    pub fn now() -> TimeStamp {
        unsafe {
            COUNTER += 0.001; // Increment by 1ms
            COUNTER
        }
    }
}

/// IMU data containing accelerometer, gyroscope and optional magnetometer measurements
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Accelerometer measurements in body frame (x, y, z) in m/s^2
    pub accel: na::Vector3<f32>,
    
    /// Gyroscope measurements in body frame (x, y, z) in rad/s
    pub gyro: na::Vector3<f32>,
    
    /// Optional magnetometer measurements in body frame (x, y, z) in gauss or tesla
    pub mag: Option<na::Vector3<f32>>,
    
    /// Timestamp when the measurements were taken (seconds)
    pub timestamp: TimeStamp,
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
    
    /// Timestamp when the measurements were taken (seconds)
    pub timestamp: TimeStamp,
}

/// Barometer data containing altitude measurements
#[derive(Debug, Clone)]
pub struct BaroData {
    /// Altitude measurement in meters (positive up)
    pub altitude: f32,
    
    /// Accuracy estimate in meters
    pub accuracy: f32,
    
    /// Timestamp when the measurement was taken (seconds)
    pub timestamp: TimeStamp,
}

/// Magnetometer data containing magnetic field measurements
#[derive(Debug, Clone)]
pub struct MagData {
    /// Magnetic field vector in body frame (x, y, z) in gauss or tesla
    pub field: na::Vector3<f32>,
    
    /// Accuracy estimate
    pub accuracy: f32,
    
    /// Timestamp when the measurement was taken (seconds)
    pub timestamp: TimeStamp,
}

/// Function to transform body frame to NED frame
pub fn body_to_ned(vec: &na::Vector3<f32>, attitude: &na::UnitQuaternion<f32>) -> na::Vector3<f32> {
    attitude * vec
}

/// Function to transform NED frame to body frame
pub fn ned_to_body(vec: &na::Vector3<f32>, attitude: &na::UnitQuaternion<f32>) -> na::Vector3<f32> {
    attitude.inverse() * vec
} 