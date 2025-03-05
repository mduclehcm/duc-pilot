use thiserror::Error;
use std::fmt;

/// Primary error type for the AHRS crate
#[derive(Error, Debug)]
pub enum AhrsError {
    /// Error during initialization of AHRS components
    #[error("Initialization error: {message}")]
    InitializationError {
        /// Detailed error message
        message: String,
        /// Source component where the error occurred
        component: Option<String>,
    },

    /// Error from sensor measurements or processing
    #[error("Sensor error: {message}")]
    SensorError {
        /// Detailed error message
        message: String,
        /// The sensor type that caused the error
        sensor_type: SensorType,
        /// Additional details about the error
        details: Option<String>,
    },

    /// Filter divergence detected (estimates becoming unstable)
    #[error("Filter divergence detected: {message}")]
    FilterDivergence {
        /// Detailed error message
        message: String,
        /// Filter that diverged
        filter_type: FilterType,
        /// Divergence magnitude if available
        magnitude: Option<f32>,
    },

    /// State became invalid (containing NaN or infinite values)
    #[error("Invalid state detected in {component}: {message}")]
    InvalidState {
        /// Detailed error message
        message: String,
        /// Component where invalid state was detected
        component: String,
    },

    /// Timing-related errors (negative or invalid time deltas)
    #[error("Timing error: {message}")]
    TimingError {
        /// Detailed error message
        message: String,
        /// Time value that caused the error if available
        time_value: Option<f32>,
    },

    /// Numerical computation errors
    #[error("Numerical error: {message}")]
    NumericalError {
        /// Detailed error message
        message: String,
        /// Additional details about the error
        details: Option<String>,
    },

    /// Matrix operation errors
    #[error("Matrix operation error: {message}")]
    MatrixError {
        /// Detailed error message
        message: String,
        /// Operation that failed
        operation: String,
    },

    /// Configuration errors
    #[error("Configuration error: {message}")]
    ConfigurationError {
        /// Detailed error message
        message: String,
        /// Configuration parameter that caused the error
        parameter: Option<String>,
    },
}

/// Types of sensors that could generate errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorType {
    /// Inertial Measurement Unit
    IMU,
    /// Global Positioning System
    GPS,
    /// Barometric altimeter
    Barometer,
    /// Magnetometer
    Magnetometer,
    /// Other or unspecified sensor
    Other,
}

/// Types of filters that could diverge
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterType {
    /// Extended Kalman Filter
    EKF,
    /// Interacting Multiple Model
    IMM,
    /// Model filter
    Model,
    /// Other or unspecified filter
    Other,
}

impl fmt::Display for SensorType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SensorType::IMU => write!(f, "IMU"),
            SensorType::GPS => write!(f, "GPS"),
            SensorType::Barometer => write!(f, "Barometer"),
            SensorType::Magnetometer => write!(f, "Magnetometer"),
            SensorType::Other => write!(f, "Other sensor"),
        }
    }
}

impl fmt::Display for FilterType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FilterType::EKF => write!(f, "EKF"),
            FilterType::IMM => write!(f, "IMM"),
            FilterType::Model => write!(f, "Model filter"),
            FilterType::Other => write!(f, "Other filter"),
        }
    }
}

/// Helper functions for creating common errors
pub mod helpers {
    use super::*;

    /// Create an initialization error
    pub fn init_error(message: impl Into<String>, component: Option<impl Into<String>>) -> AhrsError {
        AhrsError::InitializationError {
            message: message.into(),
            component: component.map(|c| c.into()),
        }
    }

    /// Create a sensor error
    pub fn sensor_error(
        message: impl Into<String>,
        sensor_type: SensorType,
        details: Option<impl Into<String>>,
    ) -> AhrsError {
        AhrsError::SensorError {
            message: message.into(),
            sensor_type,
            details: details.map(|d| d.into()),
        }
    }

    /// Create a filter divergence error
    pub fn filter_divergence(
        message: impl Into<String>,
        filter_type: FilterType,
        magnitude: Option<f32>,
    ) -> AhrsError {
        AhrsError::FilterDivergence {
            message: message.into(),
            filter_type,
            magnitude,
        }
    }

    /// Create an invalid state error
    pub fn invalid_state(message: impl Into<String>, component: impl Into<String>) -> AhrsError {
        AhrsError::InvalidState {
            message: message.into(),
            component: component.into(),
        }
    }

    /// Create a timing error
    pub fn timing_error(message: impl Into<String>, time_value: Option<f32>) -> AhrsError {
        AhrsError::TimingError {
            message: message.into(),
            time_value,
        }
    }

    /// Create a numerical error
    pub fn numerical_error(
        message: impl Into<String>,
        details: Option<impl Into<String>>,
    ) -> AhrsError {
        AhrsError::NumericalError {
            message: message.into(),
            details: details.map(|d| d.into()),
        }
    }

    /// Create a matrix error
    pub fn matrix_error(message: impl Into<String>, operation: impl Into<String>) -> AhrsError {
        AhrsError::MatrixError {
            message: message.into(),
            operation: operation.into(),
        }
    }

    /// Create a configuration error
    pub fn config_error(
        message: impl Into<String>,
        parameter: Option<impl Into<String>>,
    ) -> AhrsError {
        AhrsError::ConfigurationError {
            message: message.into(),
            parameter: parameter.map(|p| p.into()),
        }
    }

    /// Check if a vector contains invalid values (NaN or infinite)
    pub fn check_vector_valid(vec: &nalgebra::Vector3<f32>, name: &str) -> Result<(), AhrsError> {
        if vec.iter().any(|v| v.is_nan() || v.is_infinite()) {
            Err(invalid_state(
                format!("Vector {} contains NaN or infinite values", name),
                "Vector validation",
            ))
        } else {
            Ok(())
        }
    }

    /// Check if a time delta is valid
    pub fn check_time_delta(dt: f32) -> Result<(), AhrsError> {
        if dt <= 0.0 || dt.is_nan() || dt.is_infinite() {
            Err(timing_error(format!("Invalid time delta: {}", dt), Some(dt)))
        } else {
            Ok(())
        }
    }
}

/// Type alias for Result with AhrsError
pub type AhrsResult<T> = Result<T, AhrsError>;
