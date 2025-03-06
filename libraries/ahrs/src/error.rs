// No longer need alloc dependency for the refactored approach
// Removed:
// extern crate alloc;
// use alloc::string::String;
// use alloc::format;
use core::fmt;

// Using &'static str instead of String for static error messages
#[derive(Debug)]
pub enum AhrsError {
    /// Error during initialization of AHRS components
    InitializationError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Source component where the error occurred
        component: Option<Component>,
        /// Optional numeric value associated with the error
        value: Option<f32>,
    },

    /// Error from sensor measurements or processing
    SensorError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// The sensor type that caused the error
        sensor_type: SensorType,
        /// Additional numeric details if available
        value: Option<f32>,
    },

    /// Filter divergence detected (estimates becoming unstable)
    FilterDivergence {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Filter that diverged
        filter_type: FilterType,
        /// Divergence magnitude if available
        magnitude: Option<f32>,
    },

    /// State became invalid (containing NaN or infinite values)
    InvalidState {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Component where invalid state was detected
        component: Component,
    },

    /// Timing-related errors (negative or invalid time deltas)
    TimingError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Time value that caused the error if available
        time_value: Option<f32>,
    },

    /// Numerical computation errors
    NumericalError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Additional numeric details if available
        value: Option<f32>,
    },

    /// Matrix operation errors
    MatrixError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Operation that failed
        operation: MatrixOperation,
    },

    /// Configuration errors
    ConfigurationError {
        /// Predefined error message code
        error_code: ErrorCode,
        /// Configuration parameter that caused the error
        parameter: Option<ConfigParameter>,
        /// Parameter value if available
        value: Option<f32>,
    },
}

/// Static error codes used for common error messages
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    // Initialization errors
    InvalidModelConfiguration,
    InvalidTransitionMatrix,
    InvalidProbability,
    InvalidStateInitialization,
    InvalidCovariance,
    
    // Sensor errors
    InvalidSensorData,
    SensorCalibrationError,
    SensorOverflow,
    SensorTimeout,
    
    // Timing errors
    NegativeTimeStep,
    ZeroTimeStep,
    ExcessiveTimeStep,
    InvalidTimestamp,
    
    // State errors
    NanInState,
    InfiniteValue,
    DiagonalityViolation,
    SymmetryViolation,
    
    // Numerical errors
    SingularMatrix,
    DivergentState,
    CovarExceedsLimit,
    
    // Configuration errors
    InvalidParameter,
    ParameterExceedsLimit,
    ParameterBelowMinimum,
    
    // Generic errors
    UnknownError,
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

/// System components where errors can occur
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Component {
    Estimator,
    EKF,
    IMM,
    Model,
    StateVector,
    Covariance,
    SensorFusion,
    Calibration,
    Navigation,
    Other,
}

/// Matrix operations that can fail
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatrixOperation {
    Inversion,
    Multiplication,
    SVD,
    Eigendecomposition,
    Cholesky,
    QR,
    Other,
}

/// Configuration parameters that could cause errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigParameter {
    UpdateRate,
    ProcessNoise,
    SensorNoise,
    ModelWeight,
    Covariance,
    Other,
}

impl fmt::Display for SensorType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SensorType::IMU => "IMU",
            SensorType::GPS => "GPS",
            SensorType::Barometer => "Barometer",
            SensorType::Magnetometer => "Magnetometer",
            SensorType::Other => "Other sensor",
        })
    }
}

impl fmt::Display for FilterType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            FilterType::EKF => "EKF",
            FilterType::IMM => "IMM",
            FilterType::Model => "Model filter",
            FilterType::Other => "Other filter",
        })
    }
}

impl fmt::Display for Component {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Component::Estimator => "Estimator",
            Component::EKF => "EKF",
            Component::IMM => "IMM",
            Component::Model => "Model",
            Component::StateVector => "State Vector",
            Component::Covariance => "Covariance Matrix",
            Component::SensorFusion => "Sensor Fusion",
            Component::Calibration => "Calibration",
            Component::Navigation => "Navigation",
            Component::Other => "Other component",
        })
    }
}

impl fmt::Display for MatrixOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            MatrixOperation::Inversion => "Matrix Inversion",
            MatrixOperation::Multiplication => "Matrix Multiplication",
            MatrixOperation::SVD => "Singular Value Decomposition",
            MatrixOperation::Eigendecomposition => "Eigendecomposition",
            MatrixOperation::Cholesky => "Cholesky Decomposition",
            MatrixOperation::QR => "QR Decomposition",
            MatrixOperation::Other => "Other matrix operation",
        })
    }
}

impl fmt::Display for ConfigParameter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ConfigParameter::UpdateRate => "Update Rate",
            ConfigParameter::ProcessNoise => "Process Noise",
            ConfigParameter::SensorNoise => "Sensor Noise",
            ConfigParameter::ModelWeight => "Model Weight",
            ConfigParameter::Covariance => "Covariance",
            ConfigParameter::Other => "Other parameter",
        })
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            // Initialization errors
            ErrorCode::InvalidModelConfiguration => "Invalid model configuration",
            ErrorCode::InvalidTransitionMatrix => "Invalid transition matrix",
            ErrorCode::InvalidProbability => "Invalid probability value",
            ErrorCode::InvalidStateInitialization => "Invalid state initialization",
            ErrorCode::InvalidCovariance => "Invalid covariance matrix",
            
            // Sensor errors
            ErrorCode::InvalidSensorData => "Invalid sensor data",
            ErrorCode::SensorCalibrationError => "Sensor calibration error",
            ErrorCode::SensorOverflow => "Sensor value overflow",
            ErrorCode::SensorTimeout => "Sensor timeout",
            
            // Timing errors
            ErrorCode::NegativeTimeStep => "Negative time step",
            ErrorCode::ZeroTimeStep => "Zero time step",
            ErrorCode::ExcessiveTimeStep => "Excessive time step",
            ErrorCode::InvalidTimestamp => "Invalid timestamp",
            
            // State errors
            ErrorCode::NanInState => "NaN in state",
            ErrorCode::InfiniteValue => "Infinite value in state",
            ErrorCode::DiagonalityViolation => "Diagonality violation",
            ErrorCode::SymmetryViolation => "Symmetry violation",
            
            // Numerical errors
            ErrorCode::SingularMatrix => "Singular matrix",
            ErrorCode::DivergentState => "Divergent state",
            ErrorCode::CovarExceedsLimit => "Covariance exceeds limit",
            
            // Configuration errors
            ErrorCode::InvalidParameter => "Invalid parameter",
            ErrorCode::ParameterExceedsLimit => "Parameter exceeds limit",
            ErrorCode::ParameterBelowMinimum => "Parameter below minimum",
            
            // Generic errors
            ErrorCode::UnknownError => "Unknown error",
        })
    }
}

/// Helper functions for creating common errors
pub mod helpers {
    use super::*;
    

    /// Fixed-size buffer for formatting error messages with no heap allocation
    pub struct StaticBuf<const N: usize> {
        buf: [u8; N],
        len: usize,
    }

    impl<const N: usize> StaticBuf<N> {
        pub fn new() -> Self {
            Self {
                buf: [0; N],
                len: 0,
            }
        }

        pub fn as_str(&self) -> &str {
            // Safety: we only write valid UTF-8 to the buffer
            unsafe { core::str::from_utf8_unchecked(&self.buf[..self.len]) }
        }
    }

    impl<const N: usize> fmt::Write for StaticBuf<N> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let remaining = N - self.len;
            let to_copy = core::cmp::min(s.len(), remaining);
            
            self.buf[self.len..self.len + to_copy].copy_from_slice(&s.as_bytes()[..to_copy]);
            self.len += to_copy;
            
            if to_copy < s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    /// Create an initialization error
    pub fn init_error(
        error_code: ErrorCode,
        component: Option<Component>,
        value: Option<f32>,
    ) -> AhrsError {
        AhrsError::InitializationError {
            error_code,
            component,
            value,
        }
    }

    /// Create a sensor error
    pub fn sensor_error(
        error_code: ErrorCode,
        sensor_type: SensorType,
        value: Option<f32>,
    ) -> AhrsError {
        AhrsError::SensorError {
            error_code,
            sensor_type,
            value,
        }
    }

    /// Create a filter divergence error
    pub fn filter_divergence(
        error_code: ErrorCode,
        filter_type: FilterType,
        magnitude: Option<f32>,
    ) -> AhrsError {
        AhrsError::FilterDivergence {
            error_code,
            filter_type,
            magnitude,
        }
    }

    /// Create an invalid state error
    pub fn invalid_state(
        error_code: ErrorCode,
        component: Component,
    ) -> AhrsError {
        AhrsError::InvalidState {
            error_code,
            component,
        }
    }

    /// Create a timing error
    pub fn timing_error(
        error_code: ErrorCode,
        time_value: Option<f32>,
    ) -> AhrsError {
        AhrsError::TimingError {
            error_code,
            time_value,
        }
    }

    /// Create a numerical error
    pub fn numerical_error(
        error_code: ErrorCode,
        value: Option<f32>,
    ) -> AhrsError {
        AhrsError::NumericalError {
            error_code,
            value,
        }
    }

    /// Create a matrix error
    pub fn matrix_error(
        error_code: ErrorCode,
        operation: MatrixOperation,
    ) -> AhrsError {
        AhrsError::MatrixError {
            error_code,
            operation,
        }
    }

    /// Create a configuration error
    pub fn config_error(
        error_code: ErrorCode,
        parameter: Option<ConfigParameter>,
        value: Option<f32>,
    ) -> AhrsError {
        AhrsError::ConfigurationError {
            error_code,
            parameter,
            value,
        }
    }

    /// Check if a vector contains invalid values (NaN or infinite)
    pub fn check_vector_valid(vec: &nalgebra::Vector3<f32>, name: &str) -> Result<(), AhrsError> {
        if vec.iter().any(|v| v.is_nan() || v.is_infinite()) {
            Err(invalid_state(
                if name == "position" {
                    ErrorCode::NanInState
                } else {
                    ErrorCode::InfiniteValue
                },
                Component::StateVector,
            ))
        } else {
            Ok(())
        }
    }

    /// Check if a time delta is valid
    pub fn check_time_delta(dt: f32) -> Result<(), AhrsError> {
        if dt < 0.0 {
            Err(timing_error(ErrorCode::NegativeTimeStep, Some(dt)))
        } else if dt == 0.0 {
            Err(timing_error(ErrorCode::ZeroTimeStep, Some(dt)))
        } else if dt.is_nan() || dt.is_infinite() {
            Err(timing_error(ErrorCode::InvalidTimestamp, Some(dt)))
        } else {
            Ok(())
        }
    }
}

/// Type alias for Result with AhrsError
pub type AhrsResult<T> = Result<T, AhrsError>;

// Implementation of Display for AhrsError
impl fmt::Display for AhrsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AhrsError::InitializationError { error_code, component, value } => {
                write!(f, "Initialization error: {}", error_code)?;
                if let Some(component) = component {
                    write!(f, " in {}", component)?;
                }
                if let Some(value) = value {
                    write!(f, " (value: {})", value)?;
                }
                Ok(())
            }
            
            AhrsError::SensorError { error_code, sensor_type, value } => {
                write!(f, "{} error: {}", sensor_type, error_code)?;
                if let Some(value) = value {
                    write!(f, " (value: {})", value)?;
                }
                Ok(())
            }
            
            AhrsError::FilterDivergence { error_code, filter_type, magnitude } => {
                write!(f, "{} divergence: {}", filter_type, error_code)?;
                if let Some(magnitude) = magnitude {
                    write!(f, " (magnitude: {})", magnitude)?;
                }
                Ok(())
            }
            
            AhrsError::InvalidState { error_code, component } => {
                write!(f, "Invalid state in {}: {}", component, error_code)
            }
            
            AhrsError::TimingError { error_code, time_value } => {
                write!(f, "Timing error: {}", error_code)?;
                if let Some(time_value) = time_value {
                    write!(f, " (value: {})", time_value)?;
                }
                Ok(())
            }
            
            AhrsError::NumericalError { error_code, value } => {
                write!(f, "Numerical error: {}", error_code)?;
                if let Some(value) = value {
                    write!(f, " (value: {})", value)?;
                }
                Ok(())
            }
            
            AhrsError::MatrixError { error_code, operation } => {
                write!(f, "Matrix error during {}: {}", operation, error_code)
            }
            
            AhrsError::ConfigurationError { error_code, parameter, value } => {
                write!(f, "Configuration error: {}", error_code)?;
                if let Some(parameter) = parameter {
                    write!(f, " for {}", parameter)?;
                }
                if let Some(value) = value {
                    write!(f, " (value: {})", value)?;
                }
                Ok(())
            }
        }
    }
}

// For no_std environments, we can't implement Error trait
// as it requires std. If std support is needed, it should be
// added conditionally with proper imports.

#[cfg(test)]
mod tests {
    use super::*;
    // Remove unused import: core::fmt::Write
    // ... existing code ...
}
