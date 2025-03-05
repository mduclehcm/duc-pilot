# AHRS - Attitude and Heading Reference System

The AHRS module provides a robust state estimation solution for UAVs, integrating data from multiple sensors including IMU, GPS, and barometric altimeter.

## Features

- Advanced sensor fusion with EKF (Extended Kalman Filter)
- Interactive Multiple Model (IMM) pattern for adaptive estimation
- High-precision state estimation for attitude, velocity, and position
- Operation in GPS-denied environments
- Low-latency output for real-time control systems
- Comprehensive error handling system

## Error Handling System

The AHRS crate implements a comprehensive error handling system designed to provide detailed, contextual error information. This helps with debugging and allows robust error handling in applications using the crate.

### Error Types

The primary error type is `AhrsError`, which includes variants for different error categories:

- `InitializationError`: Errors during initialization of AHRS components
- `SensorError`: Errors from sensor measurements or processing
- `FilterDivergence`: Filter divergence detection (estimates becoming unstable)
- `InvalidState`: State containing NaN or infinite values
- `TimingError`: Timing-related errors (negative or invalid time deltas)
- `NumericalError`: Numerical computation errors
- `MatrixError`: Matrix operation errors
- `ConfigurationError`: Configuration errors

### Error Helpers

The crate provides convenient helper functions for creating errors with proper context:

```rust
// Create a sensor error
let error = helpers::sensor_error(
    "GPS data contains invalid values",
    SensorType::GPS,
    Some("NaN values detected in position")
);

// Create a timing error
let error = helpers::timing_error(
    "Invalid time step",
    Some(dt)
);

// Check if a vector contains invalid values
helpers::check_vector_valid(&position, "Position")?;

// Check if a time delta is valid
helpers::check_time_delta(dt)?;
```

### Using the Error System

When implementing functions that can fail, return `AhrsResult<T>`:

```rust
use crate::error::{AhrsResult, helpers, SensorType};

pub fn process_gps_data(data: &GpsData) -> AhrsResult<StateVector> {
    // Validate input data
    if data.position.iter().any(|v| v.is_nan() || v.is_infinite()) {
        return Err(helpers::sensor_error(
            "GPS data contains invalid values".to_string(),
            SensorType::GPS,
            Some("NaN or infinite values detected".to_string())
        ));
    }
    
    // Process valid data
    // ...
    
    Ok(state)
}
```

## Platform Support

This crate supports multiple platforms with different feature flags:

- **desktop**: For desktop applications and SITL (Software In The Loop) simulation
- **stm32**: For STM32 microcontrollers with Embassy framework

## License

This crate is provided under the MIT License.

## Overview

This crate provides a robust implementation of state estimation algorithms for attitude and navigation systems. It can be used in various applications including:

- Drone and UAV navigation systems
- Robotics orientation estimation
- Virtual/Augmented reality tracking
- Aerospace attitude determination
- Inertial navigation systems

## Usage

```rust
use ahrs::{Ahrs, AhrsConfig, ProcessNoise, SensorNoise, sensors::{ImuData, time_convert}};
use nalgebra as na;

fn main() {
    // Configure the AHRS system
    let config = AhrsConfig {
        update_rate: 100.0, // 100 Hz
        process_noise: ProcessNoise {
            accel_noise: 0.05,
            gyro_noise: 0.01,
            accel_bias_noise: 0.001,
            gyro_bias_noise: 0.0001,
        },
        sensor_noise: SensorNoise {
            accel_noise: 0.1,
            gyro_noise: 0.01,
            mag_noise: 0.1,
            baro_noise: 0.5,
            gps_position_noise: 1.0,
            gps_velocity_noise: 0.1,
        },
        model_weights: vec![0.5, 0.5], // Equal weights for different motion models
    };

    // Initialize the AHRS
    let mut ahrs = Ahrs::new(config).expect("Failed to initialize AHRS");

    // Simulate IMU data
    let imu_data = ImuData {
        accel: na::Vector3::new(0.0, 0.0, 9.81), // Gravity along z-axis
        gyro: na::Vector3::new(0.0, 0.0, 0.0),   // No rotation
        mag: Some(na::Vector3::new(0.3, 0.0, 0.5)), // Magnetic field
        timestamp: time_convert::now(), // Current time in seconds
    };

    // Update the filter with sensor data
    ahrs.update_imu(&imu_data).expect("Failed to update with IMU data");

    // Get the current state estimate
    let state = ahrs.state();
    
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    let euler_angles = state.euler_angles();
    println!("Roll: {}, Pitch: {}, Yaw: {}", 
             euler_angles.x, euler_angles.y, euler_angles.z);
             
    // Check if the filter is healthy
    if ahrs.is_healthy() {
        println!("AHRS estimates are reliable");
    } else {
        println!("AHRS may have diverged, reset recommended");
    }
}
```

## Timing System

The library uses a simple floating-point (`f32`) representation for timestamps, where:

- All timestamps are in seconds
- Helper modules provide platform-specific time conversion
- `time_convert::now()` returns the current time in seconds
- Timestamps are relative to system startup

This approach simplifies usage across both desktop and embedded platforms while maintaining consistent time units.

## State Representation

The filter maintains a state vector that includes:

- Attitude (orientation quaternion)
- Position (NED frame)
- Velocity (NED frame)
- Accelerometer bias
- Gyroscope bias

Each component has associated uncertainty covariance matrices.

## Health Monitoring

The system provides comprehensive health checks through the `is_healthy()` method:

1. Covariance analysis to detect filter divergence
2. Eigenvalue analysis for uncertainty assessment
3. Update timing verification
4. State validity checks
5. Model probability monitoring
