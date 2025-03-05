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

use nalgebra as na;
use thiserror::Error;

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
}

/// Result type for AHRS operations
pub type AhrsResult<T> = Result<T, AhrsError>;

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

    /// EKF implementation from ekf module
    ekf: ekf::Ekf,

    /// IMM implementation from imm module
    imm: imm::Imm,

    /// Flag indicating if GPS is available
    gps_available: bool,
}

impl Ahrs {
    /// Create a new AHRS instance with the given configuration
    pub fn new(config: AhrsConfig) -> AhrsResult<Self> {
        let state = StateVector::new();
        let ekf = ekf::Ekf::new(&config)?;
        let imm = imm::Imm::new(&config)?;

        Ok(Self {
            state,
            config,
            last_update: None,
            ekf,
            imm,
            gps_available: false,
        })
    }

    /// Update the filter with IMU measurements
    pub fn update_imu(&mut self, imu_data: &sensors::ImuData, timestamp: f32) -> AhrsResult<()> {
        // Calculate time delta for prediction
        let dt = match self.last_update {
            Some(last_time) => timestamp - last_time,
            None => 1.0 / self.config.update_rate,
        };

        // Prediction step using EKF
        let updated_state = self.ekf.predict(&self.state, imu_data, dt)?;
        self.state = updated_state;

        // Predict using IMM
        if dt > 0.0 {
            let _predicted_states = self.imm.predict(&self.state, dt);

            // Combine the states from all models
            self.state = self.imm.combine_states();
        }

        self.last_update = Some(timestamp);
        Ok(())
    }

    /// Update the filter with GPS measurements
    pub fn update_gps(&mut self, gps_data: &sensors::GpsData) -> AhrsResult<()> {
        // Update EKF with GPS data
        let updated_state = self.ekf.update_gps(&self.state, gps_data)?;
        self.state = updated_state;

        // Mark GPS as available
        self.gps_available = true;

        Ok(())
    }

    /// Update the filter with barometer measurements
    pub fn update_baro(&mut self, baro_data: &sensors::BaroData) -> AhrsResult<()> {
        // Update EKF with barometer data
        let updated_state = self.ekf.update_baro(&self.state, baro_data)?;
        self.state = updated_state;

        Ok(())
    }

    /// Get the current state estimate
    pub fn state(&self) -> &StateVector {
        &self.state
    }

    /// Get the most likely model name from the IMM
    pub fn most_likely_model_name(&self) -> &'static str {
        self.imm.most_likely_model_name()
    }

    /// Check if the filter is healthy
    ///
    /// Performs a comprehensive health assessment of the AHRS system using multiple criteria:
    ///
    /// 1. **Covariance Analysis**: Checks position, velocity, and attitude covariance traces
    ///    against adaptive thresholds that become more strict when GPS is available.
    ///
    /// 2. **Eigenvalue Analysis**: Calculates maximum eigenvalues of each covariance matrix
    ///    using power iteration for a more precise measure of uncertainty in the worst direction.
    ///
    /// 3. **Time Health**: Verifies that the filter has received updates, which could be
    ///    extended to check the time since the last update against a maximum allowable delay.
    ///
    /// 4. **State Validity**: Ensures no state values contain NaN or infinite values,
    ///    which would indicate numerical instability or filter divergence.
    ///
    /// 5. **Model Probability**: Checks that the most likely IMM model has a reasonable
    ///    probability (>20%), indicating confidence in the motion model selection.
    ///
    /// Returns `true` if all health criteria are met, indicating the filter is functioning
    /// correctly and estimates can be trusted. Returns `false` if any health check fails.
    pub fn is_healthy(&self) -> bool {
        // Check covariance values
        let pos_trace = self.state.position_covariance.trace();
        let vel_trace = self.state.velocity_covariance.trace();
        let att_trace = self.state.attitude_covariance.trace();

        // Get maximum eigenvalues for better uncertainty assessment
        let pos_max_eigenval = utils::max_eigenvalue(&self.state.position_covariance);
        let vel_max_eigenval = utils::max_eigenvalue(&self.state.velocity_covariance);
        let att_max_eigenval = utils::max_eigenvalue(&self.state.attitude_covariance);

        // Check time since last update (if available)
        let time_health = match self.last_update {
            Some(_last_time) => {
                // If we have a current time source, we could check against it
                // For now, we'll assume it's healthy if we have any update
                // In a real system, you would compare against current time:
                // let current_time = get_current_time();
                // let time_diff = current_time - last_time;
                // time_diff < MAX_TIME_WITHOUT_UPDATE
                true
            }
            None => false, // No updates received yet
        };

        // Check if any state values are NaN or infinite
        let state_valid = !self
            .state
            .position
            .iter()
            .any(|v| v.is_nan() || v.is_infinite())
            && !self
                .state
                .velocity
                .iter()
                .any(|v| v.is_nan() || v.is_infinite())
            && !self
                .state
                .attitude
                .into_inner()
                .coords
                .iter()
                .any(|v| v.is_nan() || v.is_infinite());

        // Check if the most likely model has a reasonable probability
        // (helps detect if the IMM filter is uncertain about which model to use)
        let most_likely_idx = self.imm.most_likely_model();
        let model_prob_health = self.imm.model_probability(most_likely_idx) > 0.2;

        // Adaptive thresholds based on GPS availability
        let (pos_threshold, vel_threshold, att_threshold) = if self.gps_available {
            (100.0, 10.0, 1.0) // Tighter thresholds with GPS
        } else {
            (200.0, 20.0, 2.0) // Looser thresholds without GPS
        };

        // Combined health check
        let covariance_health =
            pos_trace < pos_threshold && vel_trace < vel_threshold && att_trace < att_threshold;

        // Eigenvalue check (more strict than trace)
        let eigenvalue_health = pos_max_eigenval < pos_threshold / 2.0
            && vel_max_eigenval < vel_threshold / 2.0
            && att_max_eigenval < att_threshold / 2.0;

        // Final health determination
        state_valid && time_health && covariance_health && eigenvalue_health && model_prob_health
    }
}
