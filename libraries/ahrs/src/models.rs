use nalgebra as na;
use crate::StateVector;

/// Trait for motion models used in the AHRS system
pub trait MotionModel {
    /// Predict the next state based on the current state and time step
    fn predict(&self, state: &StateVector, dt: f32) -> StateVector;
    
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
    fn predict(&self, state: &StateVector, dt: f32) -> StateVector {
        let mut new_state = state.clone();
        
        // Position update: p_new = p + v * dt
        new_state.position += state.velocity * dt;
        
        // For constant velocity model, velocity remains the same
        // new_state.velocity = state.velocity;
        
        // Attitude, accel_bias, and gyro_bias remain the same in this simple model
        
        new_state
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
    
    /// Current acceleration estimate
    pub acceleration: na::Vector3<f32>,
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
            acceleration: na::Vector3::zeros(),
        }
    }
    
    /// Set the current acceleration estimate
    pub fn set_acceleration(&mut self, acceleration: na::Vector3<f32>) {
        self.acceleration = acceleration;
    }
}

impl MotionModel for ConstantAccelerationModel {
    fn predict(&self, state: &StateVector, dt: f32) -> StateVector {
        let mut new_state = state.clone();
        
        // Velocity update: v_new = v + a * dt
        new_state.velocity += self.acceleration * dt;
        
        // Position update: p_new = p + v * dt + 0.5 * a * dt^2
        new_state.position += state.velocity * dt + 0.5 * self.acceleration * dt * dt;
        
        // Attitude, accel_bias, and gyro_bias remain the same in this simple model
        
        new_state
    }
    
    fn process_noise(&self, dt: f32) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q = na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();
        
        // Position noise (grows with time cubed)
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt * dt;
        }
        
        // Velocity noise (grows with time squared)
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt * dt;
        }
        
        // Acceleration noise (grows with time)
        let acc_idx = 6;
        for i in 0..3 {
            q[(acc_idx + i, acc_idx + i)] = self.acc_noise * dt;
        }
        
        // Attitude noise (grows with time)
        for i in 9..12 {
            q[(i, i)] = self.att_noise * dt;
        }
        
        q
    }
    
    fn name(&self) -> &'static str {
        "Constant Acceleration Model"
    }
}

/// Coordinated turn motion model for aircraft
pub struct CoordinatedTurnModel {
    /// Process noise for position
    pub pos_noise: f32,
    
    /// Process noise for velocity
    pub vel_noise: f32,
    
    /// Process noise for turn rate
    pub turn_rate_noise: f32,
    
    /// Process noise for attitude
    pub att_noise: f32,
    
    /// Current turn rate estimate (rad/s)
    pub turn_rate: f32,
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
            turn_rate: 0.0,
        }
    }
    
    /// Set the current turn rate estimate
    pub fn set_turn_rate(&mut self, turn_rate: f32) {
        self.turn_rate = turn_rate;
    }
}

impl MotionModel for CoordinatedTurnModel {
    fn predict(&self, state: &StateVector, dt: f32) -> StateVector {
        let mut new_state = state.clone();
        
        if self.turn_rate.abs() < 1e-6 {
            // If turn rate is very small, use constant velocity model
            new_state.position += state.velocity * dt;
        } else {
            let v = state.velocity.norm();
            let heading = state.velocity.y.atan2(state.velocity.x);
            let new_heading = heading + self.turn_rate * dt;
            
            // Calculate new velocity direction
            let new_vel_x = v * new_heading.cos();
            let new_vel_y = v * new_heading.sin();
            new_state.velocity.x = new_vel_x;
            new_state.velocity.y = new_vel_y;
            
            // Position update for coordinated turn
            let r = v / self.turn_rate; // Turn radius
            new_state.position.x = state.position.x + r * (new_heading.sin() - heading.sin());
            new_state.position.y = state.position.y + r * (heading.cos() - new_heading.cos());
            
            // Vertical component remains unchanged in a coordinated turn
            new_state.position.z = state.position.z + state.velocity.z * dt;
        }
        
        new_state
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
        
        // Turn rate noise
        q[(6, 6)] = self.turn_rate_noise * dt;
        
        // Attitude noise
        for i in 7..10 {
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