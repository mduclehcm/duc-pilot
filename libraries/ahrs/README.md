# AHRS: Attitude and Heading Reference System

A Rust library implementing advanced algorithms for attitude estimation and navigation with sensor fusion.

## Overview

This crate provides a robust implementation of state estimation algorithms for attitude and navigation systems. It can be used in various applications including:

- Drone and UAV navigation systems
- Robotics orientation estimation
- Virtual/Augmented reality tracking
- Aerospace attitude determination
- Inertial navigation systems

## Features

- **Extended Kalman Filter (EKF)** implementation for state estimation
- **Interacting Multiple Model (IMM)** filter for adaptive motion model selection
- **Sensor fusion** from various inputs:
  - IMU (accelerometer, gyroscope, magnetometer)
  - GPS
  - Barometer
- **Automatic bias estimation** for gyroscope and accelerometer
- **Health monitoring** with multiple diagnostic checks
- **Coordinate transformations** between body and NED (North-East-Down) frames
- **Quaternion-based attitude representation** for numerical stability

## Usage

```rust
use ahrs::{Ahrs, AhrsConfig, ProcessNoise, SensorNoise, sensors::ImuData};
use nalgebra as na;
use std::time::Instant;

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
        timestamp: Instant::now(),
    };

    // Update the filter with sensor data
    let timestamp = 0.01; // time in seconds
    ahrs.update_imu(&imu_data, timestamp).expect("Failed to update with IMU data");

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
