use nalgebra as na;
use crate::{AhrsConfig, AhrsError, AhrsResult, StateVector};
use crate::models::{MotionModel, ConstantVelocityModel, ConstantAccelerationModel, CoordinatedTurnModel};

/// Enum to represent the different motion models
/// This allows for stack allocation of the models
pub enum ModelType {
    /// Constant velocity model
    CV(ConstantVelocityModel),
    
    /// Constant acceleration model
    CA(ConstantAccelerationModel),
    
    /// Coordinated turn model
    CT(CoordinatedTurnModel),
}

impl ModelType {
    /// Predict the state using this model
    pub fn predict(&self, state: &StateVector, dt: f32) -> StateVector {
        match self {
            ModelType::CV(model) => model.predict(state, dt),
            ModelType::CA(model) => model.predict(state, dt),
            ModelType::CT(model) => model.predict(state, dt),
        }
    }
    
    /// Get the name of the model
    pub fn name(&self) -> &'static str {
        match self {
            ModelType::CV(model) => model.name(),
            ModelType::CA(model) => model.name(),
            ModelType::CT(model) => model.name(),
        }
    }
}

/// Interactive Multiple Model (IMM) implementation
/// Uses stack allocation for the three motion models
pub struct Imm {
    /// Motion models - fixed array of 3 models
    models: [ModelType; 3],
    
    /// Model probabilities
    model_probs: [f32; 3],
    
    /// Model transition matrix - 3x3 fixed size
    transition_matrix: [[f32; 3]; 3],
    
    /// Mixing probabilities - 3x3 fixed size
    mixing_probs: [[f32; 3]; 3],
    
    /// State vectors for each model
    model_states: [StateVector; 3],
}

impl Imm {
    /// Create a new IMM instance
    pub fn new(config: &AhrsConfig) -> AhrsResult<Self> {
        // Create the three models directly on the stack
        let cv_model = ConstantVelocityModel::new(
            0.5,  // pos_noise
            0.1,  // vel_noise
            0.01, // att_noise
            config.process_noise.gyro_bias_noise,
            config.process_noise.accel_bias_noise,
        );
        
        let ca_model = ConstantAccelerationModel::new(
            1.0,  // pos_noise
            0.2,  // vel_noise
            0.1,  // acc_noise
            0.01, // att_noise
            config.process_noise.gyro_bias_noise,
            config.process_noise.accel_bias_noise,
        );
        
        let ct_model = CoordinatedTurnModel::new(
            1.0,  // pos_noise
            0.2,  // vel_noise
            0.05, // turn_rate_noise
            0.01, // att_noise
        );
        
        // Create models array
        let models = [
            ModelType::CV(cv_model),
            ModelType::CA(ca_model),
            ModelType::CT(ct_model),
        ];
        
        // Initialize model probabilities
        let mut model_probs = [0.0; 3];
        if config.model_weights.len() == 3 {
            // Normalize the provided weights
            let sum: f32 = config.model_weights.iter().sum();
            for (i, weight) in config.model_weights.iter().enumerate().take(3) {
                model_probs[i] = weight / sum;
            }
        } else {
            // Equal probabilities
            model_probs = [1.0/3.0, 1.0/3.0, 1.0/3.0];
        };
        
        // Create transition matrix (default to identity with small off-diagonal elements)
        let mut transition_matrix = [[0.0; 3]; 3];
        for (i, row) in transition_matrix.iter_mut().enumerate() {
            for (j, val) in row.iter_mut().enumerate() {
                if i == j {
                    *val = 0.98; // Diagonal elements
                } else {
                    *val = 0.01; // Off-diagonal elements
                }
            }
        }
        
        // Initialize mixing probabilities
        let mut mixing_probs = [[0.0; 3]; 3];
        for (i, row) in mixing_probs.iter_mut().enumerate() {
            row[i] = 1.0;  // Identity matrix
        }
        
        // Initialize model states with the same initial state
        let initial_state = StateVector::new();
        let model_states = [initial_state.clone(), initial_state.clone(), initial_state];
        
        Ok(Self {
            models,
            model_probs,
            transition_matrix,
            mixing_probs,
            model_states,
        })
    }
    
    /// Get the model probabilities
    pub fn model_probabilities(&self) -> &[f32; 3] {
        &self.model_probs
    }
    
    /// Get the most likely model index
    pub fn most_likely_model(&self) -> usize {
        // Find the model with the highest probability
        let mut max_prob = 0.0;
        let mut max_idx = 0;
        
        for i in 0..3 {
            if self.model_probs[i] > max_prob {
                max_prob = self.model_probs[i];
                max_idx = i;
            }
        }
        
        max_idx
    }
    
    /// Update the model probabilities based on likelihoods
    pub fn update_model_probabilities(&mut self, likelihoods: &[f32]) -> AhrsResult<()> {
        if likelihoods.len() != 3 {
            return Err(AhrsError::InvalidState);
        }
        
        // c_j = sum_i(p_ij * mu_i(k-1))
        let mut predicted_probs = [0.0; 3];
        for (j, pred_prob) in predicted_probs.iter_mut().enumerate() {
            for i in 0..3 {
                *pred_prob += self.transition_matrix[i][j] * self.model_probs[i];
            }
        }
        
        // mu_j(k) = c_j * L_j / c
        let mut updated_probs = [0.0; 3];
        let mut normalization_factor = 0.0;
        
        for (j, (updated_prob, likelihood)) in updated_probs.iter_mut().zip(likelihoods.iter()).enumerate() {
            *updated_prob = predicted_probs[j] * likelihood;
            normalization_factor += *updated_prob;
        }
        
        if normalization_factor < 1e-10 {
            return Err(AhrsError::FilterDivergence);
        }
        
        // Normalize
        for (j, prob) in self.model_probs.iter_mut().enumerate() {
            *prob = updated_probs[j] / normalization_factor;
        }
        
        Ok(())
    }
    
    /// Calculate mixing probabilities
    pub fn calculate_mixing_probabilities(&mut self) -> AhrsResult<()> {
        // Calculate mixing probabilities: mu_ij = (p_ij * mu_i) / c_j
        for j in 0..3 {
            let mut c_j = 0.0;
            
            // Calculate c_j = sum_i(p_ij * mu_i)
            for i in 0..3 {
                c_j += self.transition_matrix[i][j] * self.model_probs[i];
            }
            
            if c_j < 1e-10 {
                return Err(AhrsError::FilterDivergence);
            }
            
            // Calculate mu_ij
            for i in 0..3 {
                self.mixing_probs[i][j] = 
                    self.transition_matrix[i][j] * self.model_probs[i] / c_j;
            }
        }
        
        Ok(())
    }
    
    /// Mix the states before prediction
    pub fn mix_states(&mut self, base_state: &StateVector) -> [StateVector; 3] {
        let mut mixed_states = [base_state.clone(), base_state.clone(), base_state.clone()];
        
        // For each model, calculate the mixed state
        for (j, mixed_state) in mixed_states.iter_mut().enumerate() {
            // Initialize with zero
            let mut pos = na::Vector3::zeros();
            let mut vel = na::Vector3::zeros();
            let mut accel_bias = na::Vector3::zeros();
            let mut gyro_bias = na::Vector3::zeros();
            
            // Mix the states based on mixing probabilities
            for i in 0..3 {
                let prob = self.mixing_probs[i][j];
                if prob > 1e-10 {
                    pos += self.model_states[i].position * prob;
                    vel += self.model_states[i].velocity * prob;
                    accel_bias += self.model_states[i].accel_bias * prob;
                    gyro_bias += self.model_states[i].gyro_bias * prob;
                }
            }
            
            // Update the mixed state
            mixed_state.position = pos;
            mixed_state.velocity = vel;
            mixed_state.accel_bias = accel_bias;
            mixed_state.gyro_bias = gyro_bias;
            
            // Note: attitude mixing is more complex due to quaternions
            // For simplicity, we'll use the base state's attitude
        }
        
        mixed_states
    }
    
    /// Predict using all models
    pub fn predict(&mut self, base_state: &StateVector, dt: f32) -> [StateVector; 3] {
        // Calculate mixing probabilities
        if let Err(e) = self.calculate_mixing_probabilities() {
            eprintln!("Error calculating mixing probabilities: {:?}", e);
            // If mixing fails, just use the base state for all models
            return [base_state.clone(), base_state.clone(), base_state.clone()];
        }
        
        // Mix the states
        let mixed_states = self.mix_states(base_state);
        
        // Predict using each model
        let mut predicted_states = [StateVector::new(), StateVector::new(), StateVector::new()];
        for i in 0..3 {
            predicted_states[i] = self.models[i].predict(&mixed_states[i], dt);
        }
        
        // Store the predicted states
        self.model_states = predicted_states.clone();
        
        predicted_states
    }
    
    /// Combine the states from all models into a single state
    pub fn combine_states(&self) -> StateVector {
        let mut result = StateVector::new();
        
        // Initialize with zeros
        let mut pos = na::Vector3::zeros();
        let mut vel = na::Vector3::zeros();
        let mut accel_bias = na::Vector3::zeros();
        let mut gyro_bias = na::Vector3::zeros();
        
        // Weighted sum of states
        for i in 0..3 {
            let prob = self.model_probs[i];
            if prob > 1e-10 {
                pos += self.model_states[i].position * prob;
                vel += self.model_states[i].velocity * prob;
                accel_bias += self.model_states[i].accel_bias * prob;
                gyro_bias += self.model_states[i].gyro_bias * prob;
            }
        }
        
        // Set the combined state
        result.position = pos;
        result.velocity = vel;
        result.accel_bias = accel_bias;
        result.gyro_bias = gyro_bias;
        
        // Use the most likely model's attitude
        let most_likely = self.most_likely_model();
        result.attitude = self.model_states[most_likely].attitude;
        
        // Compute combined covariance (simplified)
        // In a full implementation, we would need to properly combine the covariances
        result.position_covariance = self.model_states[most_likely].position_covariance;
        result.velocity_covariance = self.model_states[most_likely].velocity_covariance;
        result.attitude_covariance = self.model_states[most_likely].attitude_covariance;
        
        result
    }
    
    /// Get the name of the most likely model
    pub fn most_likely_model_name(&self) -> &'static str {
        let idx = self.most_likely_model();
        self.models[idx].name()
    }
    
    /// Get the probability of a specific model
    pub fn model_probability(&self, model_idx: usize) -> f32 {
        if model_idx < 3 {
            self.model_probs[model_idx]
        } else {
            0.0 // Invalid index
        }
    }
} 