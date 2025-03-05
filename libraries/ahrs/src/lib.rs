// ### AHRS Module with EKF and IMM-Based Estimation

// The **Attitude and Heading Reference System (AHRS) module** for UAVs is a high-precision state estimation unit that integrates data from **Inertial Measurement Units (IMU), GPS, and a barometric altimeter** to determine the drone's **true position, attitude, and velocity** in real-time. This module is designed to provide robust state estimation under various flight conditions, including GPS-denied environments, by leveraging advanced sensor fusion techniques.

// #### **Key Features:**
// - **Multi-Sensor Data Fusion:**
//   - **IMU** (Accelerometer, Gyroscope, Magnetometer) for high-frequency attitude and angular velocity estimation.
//   - **GPS** for absolute position, velocity, and aiding navigation.
//   - **Barometric Altimeter** for altitude corrections and redundancy.

// - **Extended Kalman Filter (EKF):**
//   - Third-order EKF implementation for improved nonlinear state estimation.
//   - Corrects IMU drift using GPS and barometer data.
//   - Incorporates vehicle dynamics to enhance accuracy.

// - **Interactive Multiple Model (IMM) Pattern:**
//   - Uses multiple motion models (e.g., constant velocity, constant acceleration) to adapt to different flight conditions.
//   - Dynamically switches between models based on sensor data confidence.
//   - Improves robustness against sensor noise and anomalies.

// - **High-Precision State Estimation:**
//   - **Attitude (Roll, Pitch, Yaw)**
//   - **Velocity (North, East, Down)**
//   - **Position (Latitude, Longitude, Altitude)**
//   - **Acceleration and Angular Rates**

// - **Resilient in GNSS-Denied Environments:**
//   - Integrates IMU and baro data for dead-reckoning navigation when GPS is unavailable.
//   - Uses model-based corrections to minimize drift.

// - **Low-Latency Output for Control Systems:**
//   - Provides real-time updates to the UAV's flight controller for smooth, stable flight.
//   - Supports MAVLink or custom communication protocols for seamless integration.

// This AHRS module is ideal for UAV applications requiring **high accuracy, fault tolerance, and adaptability**, making it suitable for autonomous drones, VTOL aircraft, and UAV swarms operating in complex environments.

//! # AHRS - Attitude and Heading Reference System
//!
//! The AHRS module provides a robust state estimation solution for UAVs, integrating
//! data from multiple sensors including IMU, GPS, and barometric altimeter.
//!
//! ## Features
//!
//! - Advanced sensor fusion with EKF (Extended Kalman Filter)
//! - Interactive Multiple Model (IMM) pattern for adaptive estimation
//! - High-precision state estimation for attitude, velocity, and position
//! - Operation in GPS-denied environments
//! - Low-latency output for real-time control systems
//!
//! ## Platform Support
//!
//! This crate supports multiple platforms with different feature flags:
//!
//! - **desktop**: For desktop applications and SITL (Software In The Loop) simulation
//! - **stm32**: For STM32 microcontrollers with Embassy framework

use nalgebra as na;
use thiserror::Error;
use crate::imm::NUM_MODELS;

pub mod ekf;
pub mod imm;
pub mod models;
pub mod sensors;
pub mod utils;

/// Errors that can occur during AHRS operation
#[derive(Error, Debug)]
pub enum AhrsError {
    #[error("Initialization error: {0}")]
    InitializationError(String),

    #[error("Sensor error: {0}")]
    SensorError(String),

    #[error("Filter divergence detected")]
    FilterDivergence,

    #[error("Invalid state detected")]
    InvalidState,

    #[error("Timing error: {0}")]
    TimingError(String),
}

/// Result type for AHRS operations
pub type AhrsResult<T> = Result<T, AhrsError>;

/// Struct to hold information about sensor update rates
#[derive(Debug, Clone, Copy)]
pub struct SensorUpdateRates {
    /// IMU update rate in Hz (if available)
    pub imu: Option<f32>,

    /// GPS update rate in Hz (if available)
    pub gps: Option<f32>,

    /// Barometer update rate in Hz (if available)
    pub baro: Option<f32>,

    /// Timestamp of last IMU update
    pub imu_last_update: Option<f32>,

    /// Timestamp of last GPS update
    pub gps_last_update: Option<f32>,

    /// Timestamp of last barometer update
    pub baro_last_update: Option<f32>,

    /// Time since last IMU update in seconds
    pub time_since_imu: Option<f32>,

    /// Time since last GPS update in seconds
    pub time_since_gps: Option<f32>,

    /// Time since last barometer update in seconds
    pub time_since_baro: Option<f32>,
}

impl Default for SensorUpdateRates {
    fn default() -> Self {
        Self {
            imu: None,
            gps: None,
            baro: None,
            imu_last_update: None,
            gps_last_update: None,
            baro_last_update: None,
            time_since_imu: None,
            time_since_gps: None,
            time_since_baro: None,
        }
    }
}

/// State vector containing the complete vehicle state
#[derive(Debug, Clone)]
pub struct StateVector {
    /// Attitude quaternion (w, x, y, z)
    pub attitude: na::UnitQuaternion<f32>,

    /// Position in NED frame (North, East, Down) in meters
    pub position: na::Vector3<f32>,

    /// Velocity in NED frame (North, East, Down) in meters per second
    pub velocity: na::Vector3<f32>,

    /// Acceleration bias in body frame (x, y, z) in meters per second^2
    pub accel_bias: na::Vector3<f32>,

    /// Gyroscope bias in body frame (x, y, z) in radians per second
    pub gyro_bias: na::Vector3<f32>,

    /// Position uncertainty covariance matrix
    pub position_covariance: na::Matrix3<f32>,

    /// Velocity uncertainty covariance matrix
    pub velocity_covariance: na::Matrix3<f32>,

    /// Attitude uncertainty covariance matrix
    pub attitude_covariance: na::Matrix3<f32>,
}

impl Default for StateVector {
    fn default() -> Self {
        Self {
            attitude: na::UnitQuaternion::identity(),
            position: na::Vector3::zeros(),
            velocity: na::Vector3::zeros(),
            accel_bias: na::Vector3::zeros(),
            gyro_bias: na::Vector3::zeros(),
            position_covariance: na::Matrix3::identity() * 10.0,
            velocity_covariance: na::Matrix3::identity() * 1.0,
            attitude_covariance: na::Matrix3::identity() * 0.1,
        }
    }
}

impl StateVector {
    /// Create a new state vector with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    pub fn euler_angles(&self) -> na::Vector3<f32> {
        let euler = self.attitude.euler_angles();
        na::Vector3::new(euler.0, euler.1, euler.2)
    }

    /// Check if the state vector contains valid (non-NaN, non-infinite) values
    pub fn is_valid(&self) -> bool {
        !self.position.iter().any(|v| v.is_nan() || v.is_infinite()) &&
        !self.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) &&
        !self.accel_bias.iter().any(|v| v.is_nan() || v.is_infinite()) &&
        !self.gyro_bias.iter().any(|v| v.is_nan() || v.is_infinite()) &&
        // Check that quaternion components are valid
        !self.attitude.into_inner().coords.iter().any(|v| v.is_nan() || v.is_infinite())
    }

    /// Get the magnitude of the position uncertainty
    pub fn position_uncertainty(&self) -> f32 {
        self.position_covariance.trace().sqrt()
    }

    /// Get the magnitude of the velocity uncertainty
    pub fn velocity_uncertainty(&self) -> f32 {
        self.velocity_covariance.trace().sqrt()
    }

    /// Get the magnitude of the attitude uncertainty in radians
    pub fn attitude_uncertainty(&self) -> f32 {
        self.attitude_covariance.trace().sqrt()
    }

    /// Transform a vector from body frame to NED frame
    pub fn body_to_ned(&self, vector_body: &na::Vector3<f32>) -> na::Vector3<f32> {
        self.attitude * vector_body
    }

    /// Transform a vector from NED frame to body frame
    pub fn ned_to_body(&self, vector_ned: &na::Vector3<f32>) -> na::Vector3<f32> {
        self.attitude.inverse() * vector_ned
    }
}

/// Configuration for the AHRS system
#[derive(Debug, Clone)]
pub struct AhrsConfig {
    /// Update rate for the filter in Hz
    pub update_rate: f32,

    /// Process noise parameters
    pub process_noise: ProcessNoise,

    /// Sensor noise parameters
    pub sensor_noise: SensorNoise,

    /// IMM model weights
    /// Vector of weights for each model in the IMM algorithm:
    /// [0] = Constant Velocity Model
    /// [1] = Constant Acceleration Model
    /// [2] = Coordinated Turn Model
    /// [3] = Energy-Based Model (for fixed-wing aircraft pitch-altitude-speed modeling)
    /// If fewer than NUM_MODELS weights are provided, defaults to equal weights (1/NUM_MODELS each)
    pub model_weights: Vec<f32>,
}

/// Process noise configuration
#[derive(Debug, Clone)]
pub struct ProcessNoise {
    pub accel_noise: f32,
    pub gyro_noise: f32,
    pub accel_bias_noise: f32,
    pub gyro_bias_noise: f32,
}

/// Sensor noise configuration
#[derive(Debug, Clone)]
pub struct SensorNoise {
    pub accel_noise: f32,
    pub gyro_noise: f32,
    pub mag_noise: f32,
    pub baro_noise: f32,
    pub gps_position_noise: f32,
    pub gps_velocity_noise: f32,
}

/// Main AHRS implementation with EKF and IMM pattern
pub struct Ahrs {
    /// Current state estimate
    state: StateVector,

    /// System configuration
    config: AhrsConfig,

    /// Last update timestamp in seconds
    last_update: Option<f32>,

    /// IMM implementation from imm module
    imm: imm::Imm,

    /// Flag indicating if GPS is available
    gps_available: bool,

    /// Last IMU update timestamp in seconds
    last_imu_update: Option<f32>,

    /// Last GPS update timestamp in seconds
    last_gps_update: Option<f32>,

    /// Last barometer update timestamp in seconds
    last_baro_update: Option<f32>,

    /// Recent IMU update intervals for rate calculation (seconds)
    imu_update_intervals: Vec<f32>,

    /// Recent GPS update intervals for rate calculation (seconds)
    gps_update_intervals: Vec<f32>,

    /// Recent barometer update intervals for rate calculation (seconds)
    baro_update_intervals: Vec<f32>,

    /// Number of intervals to keep for rate calculation
    rate_window_size: usize,
}

impl Ahrs {
    /// Create a new AHRS instance with the given configuration
    pub fn new(config: AhrsConfig) -> AhrsResult<Self> {
        let state = StateVector::new();
        
        // Initialize the IMM - it will internally create and manage the individual EKF instances
        let imm = imm::Imm::new(&config)?;

        // Default window size for rate calculation
        const DEFAULT_RATE_WINDOW_SIZE: usize = 10;

        Ok(Self {
            state,
            config,
            last_update: None,
            imm,
            gps_available: false,
            last_imu_update: None,
            last_gps_update: None,
            last_baro_update: None,
            imu_update_intervals: Vec::with_capacity(DEFAULT_RATE_WINDOW_SIZE),
            gps_update_intervals: Vec::with_capacity(DEFAULT_RATE_WINDOW_SIZE),
            baro_update_intervals: Vec::with_capacity(DEFAULT_RATE_WINDOW_SIZE),
            rate_window_size: DEFAULT_RATE_WINDOW_SIZE,
        })
    }

    /// Update the filter with IMU measurements
    pub fn update_imu(&mut self, imu_data: &sensors::ImuData) -> AhrsResult<()> {
        // Get timestamp
        let timestamp = imu_data.timestamp;

        // Calculate time delta for prediction
        let dt = match self.last_update {
            Some(last_time) => timestamp - last_time,
            None => 1.0 / self.config.update_rate,
        };

        // Validate time step
        if dt <= 0.0 || dt.is_nan() || dt.is_infinite() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }

        // Run IMM predict step - this handles prediction for all internal models
        self.imm.predict(imu_data, dt)?;
        
        // Get the combined state from IMM - this is now our system state
        self.state = self.imm.combine_states();

        // Track IMU update rate
        if let Some(last_imu_time) = self.last_imu_update {
            let interval = timestamp - last_imu_time;
            if interval > 0.0 && !interval.is_nan() && !interval.is_infinite() {
                // Add interval to the tracking array
                self.imu_update_intervals.push(interval);
                // Keep only the window size number of intervals
                if self.imu_update_intervals.len() > self.rate_window_size {
                    self.imu_update_intervals.remove(0);
                }
            }
        }

        self.last_update = Some(timestamp);
        self.last_imu_update = Some(timestamp);

        Ok(())
    }

    /// Update the filter with GPS measurements
    pub fn update_gps(&mut self, gps_data: &sensors::GpsData) -> AhrsResult<()> {
        // Validate input data
        if gps_data
            .position
            .iter()
            .any(|v| v.is_nan() || v.is_infinite())
            || gps_data
                .velocity
                .iter()
                .any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::SensorError(
                "GPS data contains NaN or infinite values".into(),
            ));
        }

        // Update using IMM - this handles updates for all internal models
        // and updates the model probabilities
        self.imm.update_gps(gps_data)?;
        
        // Get the combined state from IMM after the update
        self.state = self.imm.combine_states();

        // Mark GPS as available
        self.gps_available = true;

        // Track GPS update rate
        let timestamp = gps_data.timestamp;
        if let Some(last_gps_time) = self.last_gps_update {
            let interval = timestamp - last_gps_time;
            if interval > 0.0 && !interval.is_nan() && !interval.is_infinite() {
                // Add interval to the tracking array
                self.gps_update_intervals.push(interval);
                // Keep only the window size number of intervals
                if self.gps_update_intervals.len() > self.rate_window_size {
                    self.gps_update_intervals.remove(0);
                }
            }
        }

        self.last_gps_update = Some(timestamp);

        Ok(())
    }

    /// Update the filter with barometer measurements
    pub fn update_baro(&mut self, baro_data: &sensors::BaroData) -> AhrsResult<()> {
        // Validate input data
        if baro_data.altitude.is_nan() || baro_data.altitude.is_infinite() {
            return Err(AhrsError::SensorError(
                "Barometer data contains NaN or infinite values".into(),
            ));
        }

        // Update using IMM - this handles updates for all internal models
        // and updates the model probabilities
        self.imm.update_baro(baro_data)?;
        
        // Get the combined state from IMM after the update
        self.state = self.imm.combine_states();

        // Track barometer update rate
        let timestamp = baro_data.timestamp;
        if let Some(last_baro_time) = self.last_baro_update {
            let interval = timestamp - last_baro_time;
            if interval > 0.0 && !interval.is_nan() && !interval.is_infinite() {
                // Add interval to the tracking array
                self.baro_update_intervals.push(interval);
                // Keep only the window size number of intervals
                if self.baro_update_intervals.len() > self.rate_window_size {
                    self.baro_update_intervals.remove(0);
                }
            }
        }

        self.last_baro_update = Some(timestamp);

        Ok(())
    }

    /// Get the current IMU update rate in Hz
    ///
    /// This method calculates the average update rate over the last several updates
    /// (defined by `rate_window_size`). Returns `None` if no updates have been received
    /// or if only a single update has been received.
    pub fn imu_update_rate(&self) -> Option<f32> {
        self.calculate_update_rate(&self.imu_update_intervals)
    }

    /// Get the current GPS update rate in Hz
    ///
    /// This method calculates the average update rate over the last several updates
    /// (defined by `rate_window_size`). Returns `None` if no updates have been received
    /// or if only a single update has been received.
    pub fn gps_update_rate(&self) -> Option<f32> {
        self.calculate_update_rate(&self.gps_update_intervals)
    }

    /// Get the current barometer update rate in Hz
    ///
    /// This method calculates the average update rate over the last several updates
    /// (defined by `rate_window_size`). Returns `None` if no updates have been received
    /// or if only a single update has been received.
    pub fn baro_update_rate(&self) -> Option<f32> {
        self.calculate_update_rate(&self.baro_update_intervals)
    }

    /// Get a summary of all sensor update rates and timing information
    ///
    /// This returns a `SensorUpdateRates` struct containing:
    /// - Update rates for each sensor in Hz
    /// - Timestamps of last updates for each sensor
    /// - Time elapsed since the last update for each sensor
    ///
    /// This is useful for monitoring sensor health and diagnosing issues
    /// with sensor data feeds.
    pub fn sensor_update_rates(&self) -> SensorUpdateRates {
        SensorUpdateRates {
            imu: self.imu_update_rate(),
            gps: self.gps_update_rate(),
            baro: self.baro_update_rate(),
            imu_last_update: self.last_imu_update,
            gps_last_update: self.last_gps_update,
            baro_last_update: self.last_baro_update,
            time_since_imu: self.time_since_last_imu_update(),
            time_since_gps: self.time_since_last_gps_update(),
            time_since_baro: self.time_since_last_baro_update(),
        }
    }

    /// Calculate the update rate from a list of intervals
    fn calculate_update_rate(&self, intervals: &[f32]) -> Option<f32> {
        if intervals.is_empty() {
            return None;
        }

        // Calculate average interval
        let sum: f32 = intervals.iter().sum();
        let avg_interval = sum / intervals.len() as f32;

        // Convert to rate (Hz)
        if avg_interval > 0.0 {
            Some(1.0 / avg_interval)
        } else {
            None
        }
    }

    /// Get time since the last IMU update in seconds
    ///
    /// This method calculates the time elapsed since the last IMU update by comparing
    /// the current time with the timestamp of the last update. Returns `None` if no
    /// IMU updates have been received.
    pub fn time_since_last_imu_update(&self) -> Option<f32> {
        match self.last_imu_update {
            Some(last_time) => {
                // Get current time using the sensors time utility
                let current_time = sensors::time_convert::now();
                Some(current_time - last_time)
            }
            None => None,
        }
    }

    /// Get time since the last GPS update in seconds
    ///
    /// This method calculates the time elapsed since the last GPS update by comparing
    /// the current time with the timestamp of the last update. Returns `None` if no
    /// GPS updates have been received.
    pub fn time_since_last_gps_update(&self) -> Option<f32> {
        match self.last_gps_update {
            Some(last_time) => {
                // Get current time using the sensors time utility
                let current_time = sensors::time_convert::now();
                Some(current_time - last_time)
            }
            None => None,
        }
    }

    /// Get time since the last barometer update in seconds
    ///
    /// This method calculates the time elapsed since the last barometer update by comparing
    /// the current time with the timestamp of the last update. Returns `None` if no
    /// barometer updates have been received.
    pub fn time_since_last_baro_update(&self) -> Option<f32> {
        match self.last_baro_update {
            Some(last_time) => {
                // Get current time using the sensors time utility
                let current_time = sensors::time_convert::now();
                Some(current_time - last_time)
            }
            None => None,
        }
    }
}
