use nalgebra as na;
use crate::{StateVector, AhrsError, AhrsResult};

/// Trait for motion models used in the AHRS system
pub trait MotionModel {
    /// Predict the next state based on the current state and time step
    fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector>;
    
    /// Get the process noise covariance for the given time step
    fn process_noise(&self, dt: f32) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>;
    
    /// Get the name of the model
    fn name(&self) -> &'static str;
}

/// Constant velocity motion model
pub struct ConstantVelocityModel {
    /// Process noise for position
    pub pos_noise: f32,
    
    /// Process noise for velocity
    pub vel_noise: f32,
    
    /// Process noise for attitude
    pub att_noise: f32,
    
    /// Process noise for gyro bias
    pub gyro_bias_noise: f32,
    
    /// Process noise for accelerometer bias
    pub accel_bias_noise: f32,
}

impl ConstantVelocityModel {
    /// Create a new constant velocity model with the given noise parameters
    pub fn new(
        pos_noise: f32,
        vel_noise: f32,
        att_noise: f32,
        gyro_bias_noise: f32,
        accel_bias_noise: f32,
    ) -> Self {
        Self {
            pos_noise,
            vel_noise,
            att_noise,
            gyro_bias_noise,
            accel_bias_noise,
        }
    }
}

impl MotionModel for ConstantVelocityModel {
    fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector> {
        // Validate input
        if dt <= 0.0 || dt.is_nan() || dt.is_infinite() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }
        
        if state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        let mut new_state = state.clone();
        
        // Position update: p_new = p + v * dt
        new_state.position += state.velocity * dt;
        
        // For constant velocity model, velocity remains the same
        // new_state.velocity = state.velocity;
        
        // Attitude, accel_bias, and gyro_bias remain the same in this simple model
        
        // Validate output
        if new_state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           new_state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        Ok(new_state)
    }
    
    fn process_noise(&self, dt: f32) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();
        
        // Position noise (grows with time squared)
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt;
        }
        
        // Velocity noise (grows with time)
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt;
        }
        
        // Attitude noise (grows with time)
        for i in 6..9 {
            q[(i, i)] = self.att_noise * dt;
        }
        
        // Gyro bias noise (random walk)
        for i in 9..12 {
            q[(i, i)] = self.gyro_bias_noise * dt;
        }
        
        q
    }
    
    fn name(&self) -> &'static str {
        "Constant Velocity Model"
    }
}

/// Constant acceleration motion model
pub struct ConstantAccelerationModel {
    /// Process noise for position
    pub pos_noise: f32,
    
    /// Process noise for velocity
    pub vel_noise: f32,
    
    /// Process noise for acceleration
    pub acc_noise: f32,
    
    /// Process noise for attitude
    pub att_noise: f32,
    
    /// Process noise for gyro bias
    pub gyro_bias_noise: f32,
    
    /// Process noise for accelerometer bias
    pub accel_bias_noise: f32,
}

impl ConstantAccelerationModel {
    /// Create a new constant acceleration model with the given noise parameters
    pub fn new(
        pos_noise: f32,
        vel_noise: f32,
        acc_noise: f32,
        att_noise: f32,
        gyro_bias_noise: f32,
        accel_bias_noise: f32,
    ) -> Self {
        Self {
            pos_noise,
            vel_noise,
            acc_noise,
            att_noise,
            gyro_bias_noise,
            accel_bias_noise,
        }
    }
}

impl MotionModel for ConstantAccelerationModel {
    fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector> {
        // Validate input
        if dt <= 0.0 || dt.is_nan() || dt.is_infinite() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }
        
        if state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        let mut new_state = state.clone();
        
        // Estimate current acceleration from accel_bias
        let acceleration = -state.accel_bias;
        
        // Position update with constant acceleration
        // p_new = p + v*dt + 0.5*a*dt^2
        new_state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;
        
        // Velocity update with constant acceleration
        // v_new = v + a*dt
        new_state.velocity += acceleration * dt;
        
        // Validate output
        if new_state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           new_state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        Ok(new_state)
    }
    
    fn process_noise(&self, dt: f32) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();
        
        // Position noise (grows with time cubed for constant acceleration model)
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt * dt;
        }
        
        // Velocity noise (grows with time squared)
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt * dt;
        }
        
        // Acceleration noise (grows with time)
        // Note: This affects the velocity part of the state
        for i in 3..6 {
            q[(i, i)] += self.acc_noise * dt;
        }
        
        // Attitude noise
        for i in 6..9 {
            q[(i, i)] = self.att_noise * dt;
        }
        
        // Gyro bias noise
        for i in 9..12 {
            q[(i, i)] = self.gyro_bias_noise * dt;
        }
        
        q
    }
    
    fn name(&self) -> &'static str {
        "Constant Acceleration Model"
    }
}

/// Coordinated turn motion model
pub struct CoordinatedTurnModel {
    /// Process noise for position
    pub pos_noise: f32,
    
    /// Process noise for velocity
    pub vel_noise: f32,
    
    /// Process noise for turn rate
    pub turn_rate_noise: f32,
    
    /// Process noise for attitude
    pub att_noise: f32,
}

impl CoordinatedTurnModel {
    /// Create a new coordinated turn model with the given noise parameters
    pub fn new(
        pos_noise: f32,
        vel_noise: f32,
        turn_rate_noise: f32,
        att_noise: f32,
    ) -> Self {
        Self {
            pos_noise,
            vel_noise,
            turn_rate_noise,
            att_noise,
        }
    }
}

impl MotionModel for CoordinatedTurnModel {
    fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector> {
        // Validate input
        if dt <= 0.0 || dt.is_nan() || dt.is_infinite() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }
        
        if state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        let mut new_state = state.clone();
        
        // Extract turn rate from gyro measurements (assume Z-axis turn)
        let turn_rate = state.gyro_bias.z;
        
        // Check for very small turn rate to avoid division by zero
        if turn_rate.abs() < 1e-6 {
            // If turn rate is very small, use constant velocity model
            new_state.position += state.velocity * dt;
        } else {
            // Coordinated turn equations
            let speed = state.velocity.norm();
            let heading = f32::atan2(state.velocity.y, state.velocity.x);
            let turn_radius = speed / turn_rate;
            
            // New heading after turn
            let new_heading = heading + turn_rate * dt;
            
            // New velocity components
            new_state.velocity.x = speed * f32::cos(new_heading);
            new_state.velocity.y = speed * f32::sin(new_heading);
            
            // New position
            // For small time steps, approximate with:
            new_state.position.x += state.velocity.x * dt;
            new_state.position.y += state.velocity.y * dt;
            
            // For larger time steps or more precision, use:
            // new_state.position.x = state.position.x + 2.0 * turn_radius * f32::sin(turn_rate * dt / 2.0) * f32::cos(heading + turn_rate * dt / 2.0);
            // new_state.position.y = state.position.y + 2.0 * turn_radius * f32::sin(turn_rate * dt / 2.0) * f32::sin(heading + turn_rate * dt / 2.0);
            
            // The z component remains the same in a coordinated turn
        }
        
        // Validate output
        if new_state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
           new_state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
            return Err(AhrsError::InvalidState);
        }
        
        Ok(new_state)
    }
    
    fn process_noise(&self, dt: f32) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();
        
        // Position noise
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt;
        }
        
        // Velocity noise
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt;
        }
        
        // Add extra noise for z-axis rotation (turn rate)
        q[(8, 8)] += self.turn_rate_noise * dt;
        
        // Attitude noise
        for i in 6..9 {
            q[(i, i)] = self.att_noise * dt;
        }
        
        q
    }
    
    fn name(&self) -> &'static str {
        "Coordinated Turn Model"
    }
}

// The create_models function is no longer needed as models are created directly in the IMM implementation
// pub fn create_models(config: &crate::AhrsConfig) -> Vec<Box<dyn MotionModel>> {
//     // Extract noise parameters from config
//     let process_noise = &config.process_noise;
//     
//     // Create models
//     let cv_model = Box::new(ConstantVelocityModel::new(
//         0.5,  // pos_noise
//         0.1,  // vel_noise
//         0.01, // att_noise
//         process_noise.gyro_bias_noise,
//         process_noise.accel_bias_noise,
//     ));
//     
//     let ca_model = Box::new(ConstantAccelerationModel::new(
//         1.0,  // pos_noise
//         0.2,  // vel_noise
//         0.1,  // acc_noise
//         0.01, // att_noise
//         process_noise.gyro_bias_noise,
//         process_noise.accel_bias_noise,
//     ));
//     
//     let ct_model = Box::new(CoordinatedTurnModel::new(
//         1.0,  // pos_noise
//         0.2,  // vel_noise
//         0.05, // turn_rate_noise
//         0.01, // att_noise
//     ));
//     
//     vec![cv_model, ca_model, ct_model]
// } 