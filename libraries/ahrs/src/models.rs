use crate::ekf::Ekf;
use crate::sensors::{BaroData, GpsData, ImuData};
use crate::{AhrsConfig, AhrsResult, StateVector};
use nalgebra as na;
use std::f32::consts::PI;
use crate::imm::NUM_MODELS;

/// A trait for filters that implement different motion models
/// 
/// Note: The actual motion model algorithms are implemented directly in the EKF's predict
/// method rather than in separate model implementations. This makes the filter implementations
/// simpler but less flexible for swapping different motion models.
pub trait ModelFilter {
    /// Initialize the filter
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()>;
    
    /// Predict step using IMU data
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector>;
    
    /// Update with GPS measurements
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector>;
    
    /// Update with barometer measurements
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector>;
    
    /// Get the current state estimate
    fn get_state(&self) -> &StateVector;
    
    /// Set the current state estimate
    fn set_state(&mut self, state: &StateVector);
    
    /// Get the model name
    fn model_name(&self) -> &'static str;
    
    /// Get the likelihood of the current measurements given the state
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32;
    
    /// Compute sensor innovation for GPS measurements
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>);
}

/// Base implementation for common functionality across all model filters
#[derive(Debug)]
struct BaseModelFilter {
    /// Process noise settings for position
    pos_noise: f32,
    
    /// Process noise settings for velocity
    vel_noise: f32,
    
    /// Process noise settings for attitude
    att_noise: f32,
    
    /// Current state estimate
    state: StateVector,
}

impl BaseModelFilter {
    /// Create a new base filter with specified noise parameters
    fn new(pos_noise: f32, vel_noise: f32, att_noise: f32) -> Self {
        Self {
            pos_noise,
            vel_noise,
            att_noise,
            state: StateVector::new(),
        }
    }
    
    /// Calculate likelihood common to all models
    fn calculate_likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                        innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        // Compute the Mahalanobis distance squared
        let inv_cov = match innovation_covariance.try_inverse() {
            Some(inv) => inv,
            None => {
                // Return a very low likelihood if the covariance can't be inverted
                return 1e-10;
            }
        };
        
        let mahalanobis_sq = innovation.transpose() * inv_cov * innovation;
        let m_dist = mahalanobis_sq[(0, 0)];
        
        // Multivariate Gaussian distribution likelihood
        let n = innovation.len() as f32;
        let det = match innovation_covariance.determinant() {
            det if det <= 0.0 => 1e-10, // Avoid numerical issues
            det => det
        };
        
        let scale = 1.0 / ((2.0 * PI).powf(n / 2.0) * det.sqrt());
        let likelihood = scale * (-0.5 * m_dist).exp();
        
        if likelihood.is_nan() || likelihood <= 0.0 {
            1e-10
        } else {
            likelihood
        }
    }
    
    /// Calculate innovation common to all models
    fn calculate_gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        // Compute the innovation (measurement residual) for GPS
        let pos_vel_innovation = na::Vector6::new(
            gps_data.position.x - self.state.position.x,
            gps_data.position.y - self.state.position.y,
            gps_data.position.z - self.state.position.z,
            gps_data.velocity.x - self.state.velocity.x,
            gps_data.velocity.y - self.state.velocity.y,
            gps_data.velocity.z - self.state.velocity.z,
        );
        
        // Create an approximation of the innovation covariance
        let mut innovation_covariance = na::Matrix6::zeros();
        
        // Position part of the covariance - scale by position noise
        for i in 0..3 {
            for j in 0..3 {
                innovation_covariance[(i, j)] = self.state.position_covariance[(i, j)] * 
                                              (1.0 + self.pos_noise);
            }
        }
        
        // Velocity part of the covariance - scale by velocity noise
        for i in 0..3 {
            for j in 0..3 {
                innovation_covariance[(i+3, j+3)] = self.state.velocity_covariance[(i, j)] *
                                                  (1.0 + self.vel_noise);
            }
        }
        
        (pos_vel_innovation, innovation_covariance)
    }
}

/// Constant velocity filter implementation
pub struct ConstantVelocityFilter {
    /// The extended Kalman filter
    ekf: Ekf,
    
    /// Base filter implementation for common functionality
    base: BaseModelFilter,
}

impl ConstantVelocityFilter {
    /// Create a new constant velocity filter
    pub fn new(config: &AhrsConfig, 
               pos_noise: f32, 
               vel_noise: f32, 
               att_noise: f32) -> AhrsResult<Self> {
        // Create EKF with appropriate process noise for constant velocity model
        let mut ekf = Ekf::new(config)?;
        
        // Configure EKF with constant velocity process noise
        // Position process noise is proportional to velocity noise
        let pos_process_noise = vel_noise * 0.25;
        ekf.set_process_noise_factor(0, 2, pos_process_noise);
        
        // Velocity should have low process noise (it's constant in this model)
        ekf.set_process_noise_factor(3, 5, vel_noise * 0.1);
        
        // Attitude process noise
        ekf.set_process_noise_factor(6, 8, att_noise);
        
        // Create base filter
        let base = BaseModelFilter::new(pos_noise, vel_noise, att_noise);
        
        Ok(Self {
            ekf,
            base,
        })
    }
    
    /// Apply constant velocity model constraints
    fn apply_cv_constraints(&mut self, _state: &mut StateVector) {
        // Constant velocity model has no constraints to apply
        // State is already in the correct format
    }
}

impl ModelFilter for ConstantVelocityFilter {
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()> {
        self.base.state = state.clone();
        Ok(())
    }
    
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        // Use EKF with constant velocity process model
        let mut predicted_state = self.ekf.predict(&self.base.state, imu_data, dt)?;
        
        // Apply constant velocity model constraints
        self.apply_cv_constraints(&mut predicted_state);
        
        // Update internal state
        self.base.state = predicted_state.clone();
        
        Ok(predicted_state)
    }
    
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        // Update with GPS data - store in temporary variable
        let mut updated_state = self.ekf.update_gps(&self.base.state, gps_data)?;
        
        // Apply model constraints to temporary state
        self.apply_cv_constraints(&mut updated_state);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        // Update with barometer data - store in temporary variable
        let mut updated_state = self.ekf.update_baro(&self.base.state, baro_data)?;
        
        // Apply model constraints to temporary state
        self.apply_cv_constraints(&mut updated_state);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn get_state(&self) -> &StateVector {
        &self.base.state
    }
    
    fn set_state(&mut self, state: &StateVector) {
        self.base.state = state.clone();
    }
    
    fn model_name(&self) -> &'static str {
        "Constant Velocity"
    }
    
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        self.base.calculate_likelihood(innovation, innovation_covariance)
    }
    
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        self.base.calculate_gps_innovation(gps_data)
    }
}

/// Constant acceleration filter implementation
pub struct ConstantAccelerationFilter {
    /// The extended Kalman filter
    ekf: Ekf,
    
    /// Base filter implementation for common functionality
    base: BaseModelFilter,
    
    /// Process noise for acceleration
    acc_noise: f32,
    
    /// Estimated acceleration in NED frame
    acceleration: na::Vector3<f32>,
}

impl ConstantAccelerationFilter {
    /// Create a new constant acceleration filter
    pub fn new(config: &AhrsConfig, 
               pos_noise: f32, 
               vel_noise: f32,
               acc_noise: f32,
               att_noise: f32) -> AhrsResult<Self> {
        // Create EKF with appropriate process noise for constant acceleration model
        let mut ekf = Ekf::new(config)?;
        
        // Configure EKF with constant acceleration process noise
        // Position process noise related to velocity and acceleration
        ekf.set_process_noise_factor(0, 2, pos_noise * 0.5);
        
        // Velocity process noise related to acceleration
        ekf.set_process_noise_factor(3, 5, vel_noise * 0.75);
        
        // Attitude process noise
        ekf.set_process_noise_factor(6, 8, att_noise);
        
        // Create base filter
        let base = BaseModelFilter::new(pos_noise, vel_noise, att_noise);
        
        Ok(Self {
            ekf,
            base,
            acc_noise,
            acceleration: na::Vector3::zeros(),
        })
    }
    
    /// Apply constant acceleration model constraints
    fn apply_ca_constraints(&mut self, state: &mut StateVector, imu_data: Option<&ImuData>) {
        // Update acceleration estimate if IMU data is provided
        if let Some(_imu) = imu_data {
            // Transform accelerometer reading to NED frame and remove gravity
            let gravity = na::Vector3::new(0.0, 0.0, 9.81);
            let accel_body = _imu.accel - state.accel_bias;
            let accel_ned = state.attitude * accel_body;
            
            // Low-pass filter the acceleration estimate
            let alpha = 0.1; // Filter coefficient
            self.acceleration = self.acceleration * (1.0 - alpha) + (accel_ned - gravity) * alpha;
        }
        
        // For a constant acceleration model, we don't enforce constraints on the state itself
    }
}

impl ModelFilter for ConstantAccelerationFilter {
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()> {
        self.base.state = state.clone();
        self.acceleration = na::Vector3::zeros();
        Ok(())
    }
    
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        // Use EKF with constant acceleration process model
        let mut predicted_state = self.ekf.predict(&self.base.state, imu_data, dt)?;
        
        // Apply constant acceleration model constraints
        self.apply_ca_constraints(&mut predicted_state, Some(imu_data));
        
        // Update internal state
        self.base.state = predicted_state.clone();
        
        Ok(predicted_state)
    }
    
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        // Update with GPS data - store in temporary variable
        let mut updated_state = self.ekf.update_gps(&self.base.state, gps_data)?;
        
        // Apply model constraints to temporary state
        self.apply_ca_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        // Update with barometer data - store in temporary variable
        let mut updated_state = self.ekf.update_baro(&self.base.state, baro_data)?;
        
        // Apply model constraints to temporary state
        self.apply_ca_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn get_state(&self) -> &StateVector {
        &self.base.state
    }
    
    fn set_state(&mut self, state: &StateVector) {
        self.base.state = state.clone();
    }
    
    fn model_name(&self) -> &'static str {
        "Constant Acceleration"
    }
    
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        // Model-specific likelihood adjustments
        // For constant acceleration model, we adjust the likelihood based on
        // how consistent the measurements are with a constant acceleration model
        
        // First get the base likelihood
        let base_likelihood = self.base.calculate_likelihood(innovation, innovation_covariance);
        
        // Scale likelihood based on acceleration consistency
        // This is a simplified approach - in a full implementation you would
        // check how constant the acceleration has been
        let acc_magnitude = self.acceleration.norm();
        let acc_factor = if acc_magnitude < 0.5 {
            // Low acceleration - less likely for CA model
            0.6
        } else if acc_magnitude < 2.0 {
            // Moderate acceleration - ideal for CA model
            1.2
        } else {
            // High acceleration - still good for CA model
            1.0
        };
        
        base_likelihood * acc_factor
    }
    
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        // Get base innovation calculation
        let (innovation, mut covariance) = self.base.calculate_gps_innovation(gps_data);
        
        // For constant acceleration model, adjust innovation covariance
        // to account for expected acceleration effects
        for i in 3..6 {
            covariance[(i, i)] *= 1.0 + self.acc_noise;
        }
        
        (innovation, covariance)
    }
}

/// Coordinated turn filter implementation
pub struct CoordinatedTurnFilter {
    /// The extended Kalman filter
    ekf: Ekf,
    
    /// Base filter implementation for common functionality
    base: BaseModelFilter,
    
    /// Process noise for turn rate
    turn_rate_noise: f32,
    
    /// Estimated turn rate around z-axis (yaw rate)
    turn_rate: f32,
}

impl CoordinatedTurnFilter {
    /// Create a new coordinated turn filter
    pub fn new(config: &AhrsConfig, 
               pos_noise: f32, 
               vel_noise: f32, 
               turn_rate_noise: f32,
               att_noise: f32) -> AhrsResult<Self> {
        // Create EKF with appropriate process noise for coordinated turn model
        let mut ekf = Ekf::new(config)?;
        
        // Configure EKF with coordinated turn process noise
        // Position process noise
        ekf.set_process_noise_factor(0, 2, pos_noise);
        
        // Velocity process noise - increased for turns
        ekf.set_process_noise_factor(3, 5, vel_noise * 1.5);
        
        // Attitude process noise - increased for yaw (z-axis)
        ekf.set_process_noise_factor(6, 8, att_noise);
        ekf.set_process_noise_factor(8, 8, att_noise * 1.5); // Higher noise for yaw
        
        // Create base filter
        let base = BaseModelFilter::new(pos_noise, vel_noise, att_noise);
        
        Ok(Self {
            ekf,
            base,
            turn_rate_noise,
            turn_rate: 0.0,
        })
    }
    
    /// Apply coordinated turn model constraints
    fn apply_ct_constraints(&mut self, state: &mut StateVector, imu_data: Option<&ImuData>) {
        // Update turn rate estimate if IMU data is provided
        if let Some(_imu) = imu_data {
            // Get the yaw rate (rotation around z-axis in body frame)
            let gyro_corrected = _imu.gyro - state.gyro_bias;
            
            // Transform to NED frame to get the turn rate
            let gyro_ned = state.attitude * gyro_corrected;
            
            // Extract the vertical component (yaw rate)
            let yaw_rate = gyro_ned.z;
            
            // Low-pass filter the turn rate estimate
            let alpha = 0.2; // Filter coefficient
            self.turn_rate = self.turn_rate * (1.0 - alpha) + yaw_rate * alpha;
        }
        
        // Coordinated turn model: horizontal velocity components change direction
        // according to turn rate, but maintain consistent magnitude
        if self.turn_rate.abs() > 0.05 {  // Only apply if we're actually turning
            // Get current horizontal velocity
            let vel_horiz = na::Vector2::new(state.velocity.x, state.velocity.y);
            let speed = vel_horiz.norm();
            
            // Ensure consistent speed in turns
            if speed > 1.0 {  // Only normalize if we have meaningful velocity
                // Keep the direction but normalize to consistent speed
                let normalized_vel = vel_horiz.normalize() * speed;
                
                // Apply a small adjustment to velocity based on turn rate
                // (This is a simplified version of the CT model)
                state.velocity.x = normalized_vel.x;
                state.velocity.y = normalized_vel.y;
            }
        }
    }
}

impl ModelFilter for CoordinatedTurnFilter {
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()> {
        self.base.state = state.clone();
        self.turn_rate = 0.0;
        Ok(())
    }
    
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        // Use EKF with coordinated turn process model
        let mut predicted_state = self.ekf.predict(&self.base.state, imu_data, dt)?;
        
        // Apply coordinated turn model constraints
        self.apply_ct_constraints(&mut predicted_state, Some(imu_data));
        
        // Update internal state
        self.base.state = predicted_state.clone();
        
        Ok(predicted_state)
    }
    
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        // Update with GPS data - store in temporary variable
        let mut updated_state = self.ekf.update_gps(&self.base.state, gps_data)?;
        
        // Apply model constraints to temporary state
        self.apply_ct_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        // Update with barometer data - store in temporary variable
        let mut updated_state = self.ekf.update_baro(&self.base.state, baro_data)?;
        
        // Apply model constraints to temporary state
        self.apply_ct_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn get_state(&self) -> &StateVector {
        &self.base.state
    }
    
    fn set_state(&mut self, state: &StateVector) {
        self.base.state = state.clone();
    }
    
    fn model_name(&self) -> &'static str {
        "Coordinated Turn"
    }
    
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        // Model-specific likelihood adjustments
        // For coordinated turn model, we adjust the likelihood based on
        // the current turn rate
        
        // First get the base likelihood
        let base_likelihood = self.base.calculate_likelihood(innovation, innovation_covariance);
        
        // Scale likelihood based on turn rate
        let turn_rate_abs = self.turn_rate.abs();
        let turn_factor = if turn_rate_abs < 0.05 {
            // Very low turn rate - less likely for CT model
            0.6
        } else if turn_rate_abs < 0.2 {
            // Moderate turn rate - good for CT model
            1.1
        } else {
            // High turn rate - ideal for CT model
            1.5
        };
        
        base_likelihood * turn_factor
    }
    
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        // Get base innovation calculation
        let (innovation, mut covariance) = self.base.calculate_gps_innovation(gps_data);
        
        // For coordinated turn model, adjust innovation covariance
        // to account for turn dynamics
        
        // Scale horizontal velocity uncertainty by turn rate
        let turn_scale = 1.0 + (self.turn_rate.abs() * self.turn_rate_noise);
        
        // Apply to horizontal velocity components (x and y)
        covariance[(3, 3)] *= turn_scale;
        covariance[(4, 4)] *= turn_scale;
        
        (innovation, covariance)
    }
}

/// Energy-based filter for fixed-wing aircraft
pub struct EnergyBasedFilter {
    /// The extended Kalman filter
    ekf: Ekf,
    
    /// Base filter implementation for common functionality
    base: BaseModelFilter,
    
    /// Total energy estimate (kinetic + potential)
    energy: f32,
    
    /// Last calculated specific energy rate
    energy_rate: f32,
}

impl EnergyBasedFilter {
    /// Create a new energy-based filter
    pub fn new(config: &AhrsConfig, 
               pos_noise: f32, 
               vel_noise: f32, 
               att_noise: f32) -> AhrsResult<Self> {
        // Create EKF with appropriate process noise for energy-based model
        let mut ekf = Ekf::new(config)?;
        
        // Configure EKF with energy-based process noise
        // Position process noise - vertical position related to energy
        ekf.set_process_noise_factor(0, 2, pos_noise);
        ekf.set_process_noise_factor(2, 2, pos_noise * 0.8); // Lower for altitude
        
        // Velocity process noise
        ekf.set_process_noise_factor(3, 5, vel_noise);
        
        // Attitude process noise - pitch coupled with energy
        ekf.set_process_noise_factor(6, 8, att_noise);
        ekf.set_process_noise_factor(7, 7, att_noise * 0.8); // Lower for pitch
        
        // Create base filter
        let base = BaseModelFilter::new(pos_noise, vel_noise, att_noise);
        
        Ok(Self {
            ekf,
            base,
            energy: 0.0,
            energy_rate: 0.0,
        })
    }
    
    /// Apply energy-based model constraints
    fn apply_eb_constraints(&mut self, state: &mut StateVector, imu_data: Option<&ImuData>) {
        // Calculate specific energy (energy per unit mass)
        // E = KE + PE = v²/2 + g*h
        let velocity_squared = state.velocity.norm_squared();
        let altitude = -state.position.z; // NED frame, so z is negative altitude
        let gravity = 9.81;
        
        // Specific energy = v²/2 + g*h
        let specific_energy = velocity_squared / 2.0 + gravity * altitude;
        
        // Calculate energy rate of change
        let energy_rate = if self.energy != 0.0 {
            (specific_energy - self.energy) / 0.1 // Assuming 0.1s between updates
        } else {
            0.0
        };
        
        // Update stored energy
        self.energy = specific_energy;
        
        // Apply a filter to energy rate
        let alpha = 0.2;
        self.energy_rate = self.energy_rate * (1.0 - alpha) + energy_rate * alpha;
        
        // For fixed-wing aircraft, pitch is coupled with energy rate
        // and vertical speed is related to pitch and airspeed
        if let Some(_imu) = imu_data {
            // Extract pitch angle
            let euler = state.attitude.euler_angles();
            let pitch = euler.1;
            
            // For a fixed-wing aircraft in energy-based model:
            // - With positive pitch, expect energy to increase or vertical speed to increase
            // - With negative pitch, expect energy to decrease or vertical speed to decrease
            
            // This is a complex relationship that depends on the specific aircraft
            // In a complete implementation, we would have an aircraft model
            // Here we just model some basic physics constraints
            
            // Airspeed (simplified as horizontal speed)
            let airspeed = na::Vector2::new(state.velocity.x, state.velocity.y).norm();
            
            if airspeed > 5.0 {  // Only apply energy model at reasonable airspeeds
                // Check if pitch and vertical speed are consistent
                let pitch_sign = pitch.signum();
                let vertical_speed = -state.velocity.z; // NED frame, negative z is up
                
                // Simple constraint: pitch and vertical speed should have same sign
                // in a stable flight condition (ignoring transients)
                if (pitch.abs() > 0.05) && (pitch_sign != vertical_speed.signum()) {
                    // If inconsistent, apply a small correction to vertical speed
                    // This models the aircraft's natural tendency to follow pitch
                    let adjustment = pitch.abs() * airspeed * 0.1;
                    state.velocity.z += -pitch_sign * adjustment; // Adjust towards pitch direction
                }
            }
        }
    }
}

impl ModelFilter for EnergyBasedFilter {
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()> {
        self.base.state = state.clone();
        
        // Initialize energy estimate
        let velocity_squared = state.velocity.norm_squared();
        let altitude = -state.position.z; // NED frame, so z is negative altitude
        let gravity = 9.81;
        self.energy = velocity_squared / 2.0 + gravity * altitude;
        self.energy_rate = 0.0;
        
        Ok(())
    }
    
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        // Use EKF with energy-based process model
        let mut predicted_state = self.ekf.predict(&self.base.state, imu_data, dt)?;
        
        // Apply energy-based model constraints
        self.apply_eb_constraints(&mut predicted_state, Some(imu_data));
        
        // Update internal state
        self.base.state = predicted_state.clone();
        
        Ok(predicted_state)
    }
    
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        // Update with GPS data - store in temporary variable
        let mut updated_state = self.ekf.update_gps(&self.base.state, gps_data)?;
        
        // Apply model constraints to temporary state
        self.apply_eb_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        // Update with barometer data - store in temporary variable
        let mut updated_state = self.ekf.update_baro(&self.base.state, baro_data)?;
        
        // Apply model constraints to temporary state
        self.apply_eb_constraints(&mut updated_state, None);
        
        // Now update internal state
        self.base.state = updated_state;
        
        Ok(self.base.state.clone())
    }
    
    fn get_state(&self) -> &StateVector {
        &self.base.state
    }
    
    fn set_state(&mut self, state: &StateVector) {
        self.base.state = state.clone();
    }
    
    fn model_name(&self) -> &'static str {
        "Energy-Based"
    }
    
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        // Model-specific likelihood adjustments
        // For energy-based model, we adjust likelihood based on energy rate consistency
        
        // First get the base likelihood
        let base_likelihood = self.base.calculate_likelihood(innovation, innovation_covariance);
        
        // Scale likelihood based on energy rate consistency
        // High energy rate consistency is good for fixed-wing aircraft
        let energy_factor = if self.energy_rate.abs() < 1.0 {
            // Very low energy rate - could be gliding or level flight
            1.1
        } else if self.energy_rate.abs() < 5.0 {
            // Moderate energy rate - typical for fixed-wing
            1.2
        } else {
            // High energy rate - aggressive maneuvers
            0.9
        };
        
        base_likelihood * energy_factor
    }
    
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        // Get base innovation calculation
        let (innovation, mut covariance) = self.base.calculate_gps_innovation(gps_data);
        
        // For energy-based model, adjust covariance to account for
        // energy conservation principles
        
        // Scale vertical position/velocity uncertainty based on energy
        let energy_scale = (1.0 + 0.1 * self.energy_rate.abs()).min(2.0);
        
        // Apply energy scaling to vertical components
        covariance[(2, 2)] *= energy_scale; // Vertical position
        covariance[(5, 5)] *= energy_scale; // Vertical velocity
        
        (innovation, covariance)
    }
}

/// Enum containing all possible model filter types
pub enum ModelFilterEnum {
    ConstantVelocity(ConstantVelocityFilter),
    ConstantAcceleration(ConstantAccelerationFilter),
    CoordinatedTurn(CoordinatedTurnFilter),
    EnergyBased(EnergyBasedFilter),
}

// Manually implement Debug for ModelFilterEnum
impl std::fmt::Debug for ModelFilterEnum {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModelFilterEnum::ConstantVelocity(_) => write!(f, "ModelFilterEnum::ConstantVelocity"),
            ModelFilterEnum::ConstantAcceleration(_) => write!(f, "ModelFilterEnum::ConstantAcceleration"),
            ModelFilterEnum::CoordinatedTurn(_) => write!(f, "ModelFilterEnum::CoordinatedTurn"),
            ModelFilterEnum::EnergyBased(_) => write!(f, "ModelFilterEnum::EnergyBased"),
        }
    }
}

impl ModelFilter for ModelFilterEnum {
    fn initialize(&mut self, state: &StateVector) -> AhrsResult<()> {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.initialize(state),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.initialize(state),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.initialize(state),
            ModelFilterEnum::EnergyBased(filter) => filter.initialize(state),
        }
    }
    
    fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.predict(imu_data, dt),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.predict(imu_data, dt),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.predict(imu_data, dt),
            ModelFilterEnum::EnergyBased(filter) => filter.predict(imu_data, dt),
        }
    }
    
    fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.update_gps(gps_data),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.update_gps(gps_data),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.update_gps(gps_data),
            ModelFilterEnum::EnergyBased(filter) => filter.update_gps(gps_data),
        }
    }
    
    fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.update_baro(baro_data),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.update_baro(baro_data),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.update_baro(baro_data),
            ModelFilterEnum::EnergyBased(filter) => filter.update_baro(baro_data),
        }
    }
    
    fn get_state(&self) -> &StateVector {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.get_state(),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.get_state(),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.get_state(),
            ModelFilterEnum::EnergyBased(filter) => filter.get_state(),
        }
    }
    
    fn set_state(&mut self, state: &StateVector) {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.set_state(state),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.set_state(state),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.set_state(state),
            ModelFilterEnum::EnergyBased(filter) => filter.set_state(state),
        }
    }
    
    fn model_name(&self) -> &'static str {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.model_name(),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.model_name(),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.model_name(),
            ModelFilterEnum::EnergyBased(filter) => filter.model_name(),
        }
    }
    
    fn likelihood(&self, innovation: &na::Vector<f32, na::Const<6>, na::ArrayStorage<f32, 6, 1>>,
                 innovation_covariance: &na::Matrix<f32, na::Const<6>, na::Const<6>, na::ArrayStorage<f32, 6, 6>>) -> f32 {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.likelihood(innovation, innovation_covariance),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.likelihood(innovation, innovation_covariance),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.likelihood(innovation, innovation_covariance),
            ModelFilterEnum::EnergyBased(filter) => filter.likelihood(innovation, innovation_covariance),
        }
    }
    
    fn gps_innovation(&self, gps_data: &GpsData) -> (na::Vector6<f32>, na::Matrix6<f32>) {
        match self {
            ModelFilterEnum::ConstantVelocity(filter) => filter.gps_innovation(gps_data),
            ModelFilterEnum::ConstantAcceleration(filter) => filter.gps_innovation(gps_data),
            ModelFilterEnum::CoordinatedTurn(filter) => filter.gps_innovation(gps_data),
            ModelFilterEnum::EnergyBased(filter) => filter.gps_innovation(gps_data),
        }
    }
}

/// Create model filters for the IMM filter
pub fn create_model_filters(config: &AhrsConfig) -> AhrsResult<[ModelFilterEnum; NUM_MODELS]> {
    // Create the model filters
    let cv_model = ConstantVelocityFilter::new(
        config, 
        0.3, 
        1.0, 
        0.1
    )?;
    let ca_model = ConstantAccelerationFilter::new(
        config, 
        0.3, 
        1.0,
        2.0,
        0.1
    )?;
    let ct_model = CoordinatedTurnFilter::new(
        config, 
        0.3, 
        1.0,
        0.2,
        0.1
    )?;
    let eb_model = EnergyBasedFilter::new(
        config, 
        0.3, 
        1.0, 
        0.1
    )?;

    // Return the model filters directly
    Ok([
        ModelFilterEnum::ConstantVelocity(cv_model),
        ModelFilterEnum::ConstantAcceleration(ca_model),
        ModelFilterEnum::CoordinatedTurn(ct_model),
        ModelFilterEnum::EnergyBased(eb_model),
    ])
}
