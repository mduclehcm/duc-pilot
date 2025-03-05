#[cfg(feature = "embassy")]
use embassy_time::{Duration, Instant};
use nalgebra as na;

/// Timestamp type - using f32 in seconds for all timestamps
pub type TimeStamp = f32;

/// Standard library time conversion utilities
#[cfg(feature = "desktop")]
pub mod time_convert {
    use super::TimeStamp;
    use std::sync::OnceLock;
    use std::time::Instant;

    /// Base time for std::time::Instant conversions
    static BASE_TIME: OnceLock<Instant> = OnceLock::new();

    /// Convert from std::time::Instant to TimeStamp (f32 seconds)
    pub fn instant_to_timestamp(instant: &Instant) -> TimeStamp {
        // Initialize once, thread-safely
        let base = BASE_TIME.get_or_init(|| *instant);
        instant.duration_since(*base).as_secs_f32()
    }

    /// Get current time as timestamp
    pub fn now() -> TimeStamp {
        instant_to_timestamp(&Instant::now())
    }
}

/// Time conversion for Embassy RTOS
#[cfg(feature = "embassy")]
pub mod time_convert {
    use super::TimeStamp;
    use embassy_time::{Duration, Instant};

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

// The std feature is enabled by default in Cargo.toml.
// This error only occurs if someone explicitly disables default features
// without enabling either std or embassy.
#[cfg(not(any(feature = "desktop", feature = "embassy")))]
compile_error!("Either the \"std\" or \"embassy\" feature must be enabled for this crate");

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
