use crate::sensors::{BaroData, GpsData, ImuData};
use crate::utils;
use crate::{AhrsConfig, AhrsError, AhrsResult, StateVector};
use nalgebra as na;

/// Extended Kalman Filter (EKF) implementation
pub struct Ekf {
    /// Process noise covariance matrix
    process_noise: na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>,

    /// State covariance matrix
    covariance: na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>,

    /// Configuration
    config: AhrsConfig,

    /// Earth's gravitational acceleration vector in NED frame (m/s^2)
    gravity: na::Vector3<f32>,

    /// Earth's magnetic field vector in NED frame (normalized)
    mag_field_ned: na::Vector3<f32>,

    /// Previous gyroscope measurement
    prev_gyro: Option<na::Vector3<f32>>,

    /// Previous accelerometer measurement
    prev_accel: Option<na::Vector3<f32>>,
}

// Add module-level constant for state dimension
const STATE_DIM: usize = 12; // position (3), velocity (3), attitude (3), gyro bias (3)

impl Ekf {
    /// Create a new EKF instance
    pub fn new(config: &AhrsConfig) -> AhrsResult<Self> {
        // Initialize process noise
        let mut process_noise =
            na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();

        // Position process noise
        for i in 0..3 {
            process_noise[(i, i)] = 0.01;
        }

        // Velocity process noise
        for i in 3..6 {
            process_noise[(i, i)] =
                config.process_noise.accel_noise * config.process_noise.accel_noise;
        }

        // Attitude process noise
        for i in 6..9 {
            process_noise[(i, i)] =
                config.process_noise.gyro_noise * config.process_noise.gyro_noise;
        }

        // Gyro bias process noise
        for i in 9..12 {
            process_noise[(i, i)] =
                config.process_noise.gyro_bias_noise * config.process_noise.gyro_bias_noise;
        }

        // Initialize covariance matrix
        let mut covariance =
            na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();

        // Initial position uncertainty (10m)
        for i in 0..3 {
            covariance[(i, i)] = 10.0 * 10.0;
        }

        // Initial velocity uncertainty (1m/s)
        for i in 3..6 {
            covariance[(i, i)] = 1.0 * 1.0;
        }

        // Initial attitude uncertainty (0.1rad)
        for i in 6..9 {
            covariance[(i, i)] = 0.1 * 0.1;
        }

        // Initial gyro bias uncertainty
        for i in 9..12 {
            covariance[(i, i)] = 0.01 * 0.01;
        }

        // Earth's gravitational acceleration in NED frame (9.81 m/s^2 down)
        let gravity = na::Vector3::new(0.0, 0.0, 9.81);

        // Earth's magnetic field in NED frame (normalized)
        // This is a simplification and should be set based on the location
        let mag_field_ned = na::Vector3::new(0.0, 1.0, 0.0).normalize();

        Ok(Self {
            process_noise,
            covariance,
            config: config.clone(),
            gravity,
            mag_field_ned,
            prev_gyro: None,
            prev_accel: None,
        })
    }

    /// Set the Earth's magnetic field vector in NED frame
    pub fn set_magnetic_field(&mut self, mag_field: na::Vector3<f32>) {
        self.mag_field_ned = mag_field.normalize();
    }

    /// Predict step using IMU measurements
    pub fn predict(
        &mut self,
        state: &StateVector,
        imu_data: &ImuData,
        dt: f32,
    ) -> AhrsResult<StateVector> {
        // Input validation
        if dt <= 0.0 || dt.is_nan() {
            return Err(AhrsError::TimingError(format!(
                "Invalid time step in EKF prediction: {}", dt
            )));
        }
        
        if imu_data.accel.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           imu_data.gyro.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::SensorError(
                "IMU data contains NaN or infinite values".into()
            ));
        }
        
        let mut new_state = state.clone();

        // Extract gyro and accelerometer measurements
        let gyro_meas = imu_data.gyro;
        let accel_meas = imu_data.accel;

        // Apply gyro bias correction
        let gyro_corrected = gyro_meas - state.gyro_bias;

        // Update attitude using gyro measurements
        let delta_angle = gyro_corrected * dt;
        let delta_q = na::UnitQuaternion::from_scaled_axis(delta_angle);
        new_state.attitude = state.attitude * delta_q;

        // Transform accelerometer measurements to NED frame
        let accel_body = accel_meas - state.accel_bias;
        let accel_ned = state.attitude * accel_body;

        // Remove gravity to get linear acceleration
        let linear_accel = accel_ned - self.gravity;

        // Update velocity using linear acceleration
        new_state.velocity += linear_accel * dt;

        // Update position using velocity
        new_state.position += state.velocity * dt + 0.5 * linear_accel * dt * dt;

        // Store current measurements for next iteration
        self.prev_gyro = Some(gyro_meas);
        self.prev_accel = Some(accel_meas);

        // Update state covariance using linearized system model
        self.predict_covariance(&gyro_corrected, dt)?;

        // Update state uncertainty estimates
        new_state.position_covariance = na::Matrix3::new(
            self.covariance[(0, 0)],
            self.covariance[(0, 1)],
            self.covariance[(0, 2)],
            self.covariance[(1, 0)],
            self.covariance[(1, 1)],
            self.covariance[(1, 2)],
            self.covariance[(2, 0)],
            self.covariance[(2, 1)],
            self.covariance[(2, 2)],
        );

        new_state.velocity_covariance = na::Matrix3::new(
            self.covariance[(3, 3)],
            self.covariance[(3, 4)],
            self.covariance[(3, 5)],
            self.covariance[(4, 3)],
            self.covariance[(4, 4)],
            self.covariance[(4, 5)],
            self.covariance[(5, 3)],
            self.covariance[(5, 4)],
            self.covariance[(5, 5)],
        );

        new_state.attitude_covariance = na::Matrix3::new(
            self.covariance[(6, 6)],
            self.covariance[(6, 7)],
            self.covariance[(6, 8)],
            self.covariance[(7, 6)],
            self.covariance[(7, 7)],
            self.covariance[(7, 8)],
            self.covariance[(8, 6)],
            self.covariance[(8, 7)],
            self.covariance[(8, 8)],
        );

        Ok(new_state)
    }

    /// Predict the covariance matrix using the linearized system model
    fn predict_covariance(
        &mut self,
        gyro_corrected: &na::Vector3<f32>,
        dt: f32,
    ) -> AhrsResult<()> {
        // Create state transition matrix (F)
        let mut f = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::identity();

        // Position update from velocity
        for i in 0..3 {
            f[(i, i + 3)] = dt;
        }

        // Velocity update from attitude
        let accel_skew = utils::skew_symmetric(&self.prev_accel.unwrap_or(na::Vector3::zeros()));
        for i in 0..3 {
            for j in 0..3 {
                f[(i + 3, j + 6)] = -dt * accel_skew[(i, j)];
            }
        }

        // Attitude update from gyro
        let gyro_skew = utils::skew_symmetric(gyro_corrected);
        for i in 0..3 {
            for j in 0..3 {
                f[(i + 6, j + 6)] += -dt * gyro_skew[(i, j)];
            }
        }

        // Attitude update from gyro bias
        for i in 0..3 {
            for j in 0..3 {
                f[(i + 6, j + 9)] = -dt * (if i == j { 1.0 } else { 0.0 });
            }
        }

        // Update process noise using process noise factors from config

        // Scale noise based on state dimensions
        let scale_factor = (STATE_DIM as f32).sqrt();
        let q = self.process_noise * dt * scale_factor;

        // Update covariance matrix: P = F*P*F' + Q
        self.covariance = f * self.covariance * f.transpose() + q;

        Ok(())
    }

    /// Update step using GPS measurements
    pub fn update_gps(
        &mut self,
        state: &StateVector,
        gps_data: &GpsData,
    ) -> AhrsResult<StateVector> {
        // Input validation
        if gps_data.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           gps_data.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           gps_data.accuracy.iter().any(|v| v.is_nan() || v.is_infinite() || *v <= 0.0) {
            return Err(AhrsError::SensorError(
                "GPS data contains invalid values".into()
            ));
        }
        
        let mut new_state = state.clone();

        // Create measurement vector (GPS position and velocity)
        let z = na::Vector6::new(
            gps_data.position.x,
            gps_data.position.y,
            gps_data.position.z,
            gps_data.velocity.x,
            gps_data.velocity.y,
            gps_data.velocity.z,
        );

        // Create predicted measurement vector
        let z_pred = na::Vector6::new(
            state.position.x,
            state.position.y,
            state.position.z,
            state.velocity.x,
            state.velocity.y,
            state.velocity.z,
        );

        // Create measurement Jacobian matrix (H)
        let mut h =
            na::Matrix::<f32, na::Const<6>, na::Const<12>, na::ArrayStorage<f32, 6, 12>>::zeros();

        // Position measurement
        for i in 0..3 {
            h[(i, i)] = 1.0;
        }

        // Velocity measurement
        for i in 0..3 {
            h[(i + 3, i + 3)] = 1.0;
        }

        // Create measurement noise covariance matrix (R)
        let mut r = na::Matrix6::zeros();

        // Position measurement noise
        for i in 0..3 {
            r[(i, i)] = gps_data.accuracy.x * gps_data.accuracy.x;
        }

        // Velocity measurement noise
        for i in 0..3 {
            r[(i + 3, i + 3)] = gps_data.accuracy.y * gps_data.accuracy.y;
        }

        // Kalman gain: K = P*H'*(H*P*H' + R)^-1
        let pht = self.covariance * h.transpose();
        let s = h * pht + r;

        // Check if S is invertible
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return Err(AhrsError::FilterDivergence),
        };

        let k = pht * s_inv;

        // Update state: x = x + K*(z - z_pred)
        let innovation = z - z_pred;

        // Apply position correction
        for i in 0..3 {
            new_state.position[i] += k[(i, 0)] * innovation[0]
                + k[(i, 1)] * innovation[1]
                + k[(i, 2)] * innovation[2]
                + k[(i, 3)] * innovation[3]
                + k[(i, 4)] * innovation[4]
                + k[(i, 5)] * innovation[5];
        }

        // Apply velocity correction
        for i in 0..3 {
            new_state.velocity[i] += k[(i + 3, 0)] * innovation[0]
                + k[(i + 3, 1)] * innovation[1]
                + k[(i + 3, 2)] * innovation[2]
                + k[(i + 3, 3)] * innovation[3]
                + k[(i + 3, 4)] * innovation[4]
                + k[(i + 3, 5)] * innovation[5];
        }

        // Apply attitude correction
        let attitude_correction = na::Vector3::new(
            k[(6, 0)] * innovation[0]
                + k[(6, 1)] * innovation[1]
                + k[(6, 2)] * innovation[2]
                + k[(6, 3)] * innovation[3]
                + k[(6, 4)] * innovation[4]
                + k[(6, 5)] * innovation[5],
            k[(7, 0)] * innovation[0]
                + k[(7, 1)] * innovation[1]
                + k[(7, 2)] * innovation[2]
                + k[(7, 3)] * innovation[3]
                + k[(7, 4)] * innovation[4]
                + k[(7, 5)] * innovation[5],
            k[(8, 0)] * innovation[0]
                + k[(8, 1)] * innovation[1]
                + k[(8, 2)] * innovation[2]
                + k[(8, 3)] * innovation[3]
                + k[(8, 4)] * innovation[4]
                + k[(8, 5)] * innovation[5],
        );

        if attitude_correction.norm() > 1e-6 {
            let delta_q = na::UnitQuaternion::from_scaled_axis(attitude_correction);
            new_state.attitude = delta_q * state.attitude;
        }

        // Apply gyro bias correction
        for i in 0..3 {
            new_state.gyro_bias[i] += k[(i + 9, 0)] * innovation[0]
                + k[(i + 9, 1)] * innovation[1]
                + k[(i + 9, 2)] * innovation[2]
                + k[(i + 9, 3)] * innovation[3]
                + k[(i + 9, 4)] * innovation[4]
                + k[(i + 9, 5)] * innovation[5];
        }

        // Update covariance: P = (I - K*H)*P
        let identity = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::identity();
        self.covariance = (identity - k * h) * self.covariance;

        // Update state uncertainty estimates
        new_state.position_covariance = na::Matrix3::new(
            self.covariance[(0, 0)],
            self.covariance[(0, 1)],
            self.covariance[(0, 2)],
            self.covariance[(1, 0)],
            self.covariance[(1, 1)],
            self.covariance[(1, 2)],
            self.covariance[(2, 0)],
            self.covariance[(2, 1)],
            self.covariance[(2, 2)],
        );

        new_state.velocity_covariance = na::Matrix3::new(
            self.covariance[(3, 3)],
            self.covariance[(3, 4)],
            self.covariance[(3, 5)],
            self.covariance[(4, 3)],
            self.covariance[(4, 4)],
            self.covariance[(4, 5)],
            self.covariance[(5, 3)],
            self.covariance[(5, 4)],
            self.covariance[(5, 5)],
        );

        new_state.attitude_covariance = na::Matrix3::new(
            self.covariance[(6, 6)],
            self.covariance[(6, 7)],
            self.covariance[(6, 8)],
            self.covariance[(7, 6)],
            self.covariance[(7, 7)],
            self.covariance[(7, 8)],
            self.covariance[(8, 6)],
            self.covariance[(8, 7)],
            self.covariance[(8, 8)],
        );

        Ok(new_state)
    }

    /// Update step using barometer measurements
    pub fn update_baro(
        &mut self,
        state: &StateVector,
        baro_data: &BaroData,
    ) -> AhrsResult<StateVector> {
        // Input validation
        if baro_data.altitude.is_nan() || baro_data.altitude.is_infinite() ||
           baro_data.accuracy.is_nan() || baro_data.accuracy.is_infinite() || baro_data.accuracy <= 0.0 {
            return Err(AhrsError::SensorError(
                "Barometer data contains invalid values".into()
            ));
        }
        
        let mut new_state = state.clone();

        // Create measurement (altitude is negative of down position in NED)
        let z = -baro_data.altitude;

        // Create predicted measurement
        let z_pred = state.position.z;

        // Create measurement Jacobian (H)
        let mut h = na::RowVector::<f32, na::Const<12>, na::ArrayStorage<f32, 1, 12>>::zeros();
        h[2] = 1.0; // Measurement is the z (down) position

        // Measurement noise
        let r = baro_data.accuracy * baro_data.accuracy;

        // Kalman gain: K = P*H'*(H*P*H' + R)^-1
        let pht = self.covariance * h.transpose();
        let s = h * pht + na::Matrix1::new(r); // Convert scalar r to a 1x1 matrix

        // Check if S is invertible
        let s_inv = 1.0 / s[(0, 0)]; // Access element using (row, col) notation

        let k = pht * s_inv;

        // Update state: x = x + K*(z - z_pred)
        let innovation = z - z_pred;

        // Apply position correction
        for i in 0..3 {
            new_state.position[i] += k[i] * innovation;
        }

        // Apply velocity correction
        for i in 0..3 {
            new_state.velocity[i] += k[i + 3] * innovation;
        }

        // Apply attitude correction
        let attitude_correction =
            na::Vector3::new(k[6] * innovation, k[7] * innovation, k[8] * innovation);

        if attitude_correction.norm() > 1e-6 {
            let delta_q = na::UnitQuaternion::from_scaled_axis(attitude_correction);
            new_state.attitude = delta_q * state.attitude;
        }

        // Apply gyro bias correction
        for i in 0..3 {
            new_state.gyro_bias[i] += k[i + 9] * innovation;
        }

        // Update covariance: P = (I - K*H)*P
        let kh = k * h;
        let mut identity_minus_kh = na::Matrix::<
            f32,
            na::Const<12>,
            na::Const<12>,
            na::ArrayStorage<f32, 12, 12>,
        >::identity();
        for i in 0..12 {
            for j in 0..12 {
                identity_minus_kh[(i, j)] -= kh[(i, j)];
            }
        }

        self.covariance = identity_minus_kh * self.covariance;

        // Update state uncertainty estimates
        new_state.position_covariance = na::Matrix3::new(
            self.covariance[(0, 0)],
            self.covariance[(0, 1)],
            self.covariance[(0, 2)],
            self.covariance[(1, 0)],
            self.covariance[(1, 1)],
            self.covariance[(1, 2)],
            self.covariance[(2, 0)],
            self.covariance[(2, 1)],
            self.covariance[(2, 2)],
        );

        new_state.velocity_covariance = na::Matrix3::new(
            self.covariance[(3, 3)],
            self.covariance[(3, 4)],
            self.covariance[(3, 5)],
            self.covariance[(4, 3)],
            self.covariance[(4, 4)],
            self.covariance[(4, 5)],
            self.covariance[(5, 3)],
            self.covariance[(5, 4)],
            self.covariance[(5, 5)],
        );

        new_state.attitude_covariance = na::Matrix3::new(
            self.covariance[(6, 6)],
            self.covariance[(6, 7)],
            self.covariance[(6, 8)],
            self.covariance[(7, 6)],
            self.covariance[(7, 7)],
            self.covariance[(7, 8)],
            self.covariance[(8, 6)],
            self.covariance[(8, 7)],
            self.covariance[(8, 8)],
        );

        Ok(new_state)
    }

    /// Update step using magnetometer measurements
    pub fn update_magnetometer(
        &mut self,
        state: &StateVector,
        imu_data: &ImuData,
    ) -> AhrsResult<StateVector> {
        if imu_data.mag.is_none() {
            return Ok(state.clone());
        }

        let mut new_state = state.clone();
        let mag_meas = imu_data.mag.unwrap();

        // Normalize magnetic field measurement
        let mag_meas_normalized = mag_meas.normalize();

        // Expected magnetic field in body frame (transforming from NED to body)
        let expected_mag_body = state.attitude.inverse() * self.mag_field_ned;

        // Innovation (difference between measured and expected)
        let innovation = mag_meas_normalized - expected_mag_body;

        // Measurement Jacobian (H)
        // The measurement model is complex because it involves the attitude quaternion
        // For simplicity, we'll use a small angle approximation and update only the attitude

        // Create measurement noise
        let mag_noise = self.config.sensor_noise.mag_noise;
        let r = na::Matrix3::identity() * mag_noise;

        // Measurement dimension check to ensure model consistency
        assert_eq!(
            r.nrows(),
            3,
            "Magnetometer measurement dimension mismatch: expected {}, got {}",
            3,
            r.nrows()
        );

        // Create innovation covariance
        let mag_cross = utils::skew_symmetric(&expected_mag_body);
        let h_attitude = mag_cross;

        let attitude_cov = na::Matrix3::new(
            self.covariance[(6, 6)],
            self.covariance[(6, 7)],
            self.covariance[(6, 8)],
            self.covariance[(7, 6)],
            self.covariance[(7, 7)],
            self.covariance[(7, 8)],
            self.covariance[(8, 6)],
            self.covariance[(8, 7)],
            self.covariance[(8, 8)],
        );

        let s = h_attitude * attitude_cov * h_attitude.transpose() + r;

        // Check if S is invertible
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return Err(AhrsError::FilterDivergence),
        };

        // Kalman gain for attitude correction
        let k_attitude = attitude_cov * h_attitude.transpose() * s_inv;

        // Apply attitude correction
        let attitude_correction = k_attitude * innovation;

        if attitude_correction.norm() > 1e-6 {
            let delta_q = na::UnitQuaternion::from_scaled_axis(attitude_correction);
            new_state.attitude = delta_q * state.attitude;
        }

        // Update attitude covariance
        let identity_minus_kh = na::Matrix3::identity() - k_attitude * h_attitude;
        let new_attitude_cov = identity_minus_kh * attitude_cov;

        // Update the attitude portion of the full covariance matrix
        for i in 0..3 {
            for j in 0..3 {
                self.covariance[(i + 6, j + 6)] = new_attitude_cov[(i, j)];
            }
        }

        // Update state uncertainty estimates
        new_state.attitude_covariance = new_attitude_cov;

        Ok(new_state)
    }
}
