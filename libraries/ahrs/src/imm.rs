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
    pub fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector> {
        // Validate input
        if dt <= 0.0 || dt.is_nan() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }
        
        let result = match self {
            ModelType::CV(model) => model.predict(state, dt),
            ModelType::CA(model) => model.predict(state, dt),
            ModelType::CT(model) => model.predict(state, dt),
        };
        
        // Validate output
        let state_result = match result {
            Ok(state) => {
                if state.position.iter().any(|v| v.is_nan() || v.is_infinite()) || 
                   state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
                    Err(AhrsError::InvalidState)
                } else {
                    Ok(state)
                }
            },
            Err(e) => Err(e),
        };
        
        state_result
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
        if config.model_weights.len() >= 3 {
            // Normalize the provided weights
            let sum: f32 = config.model_weights.iter().take(3).sum();
            if sum > 0.0 {
                for (i, weight) in config.model_weights.iter().enumerate().take(3) {
                    model_probs[i] = weight / sum;
                }
            } else {
                // Default equal probabilities if sum is 0 or negative
                model_probs = [1.0/3.0, 1.0/3.0, 1.0/3.0];
            }
        } else {
            // Default equal probabilities if not enough weights provided
            model_probs = [1.0/3.0, 1.0/3.0, 1.0/3.0];
        }
        
        // Initialize transition matrix with default values
        // This represents the probability of switching from model i to model j
        let transition_matrix = [
            [0.95, 0.025, 0.025], // From CV to [CV, CA, CT]
            [0.025, 0.95, 0.025], // From CA to [CV, CA, CT]
            [0.025, 0.025, 0.95], // From CT to [CV, CA, CT]
        ];
        
        // Initialize mixing probabilities
        let mixing_probs = [[0.0; 3]; 3];
        
        // Initialize model states
        let model_states = [
            StateVector::new(),
            StateVector::new(),
            StateVector::new(),
        ];
        
        Ok(Self {
            models,
            model_probs,
            transition_matrix,
            mixing_probs,
            model_states,
        })
    }
    
    /// Predict the state using all models
    pub fn predict(&mut self, state: &StateVector, dt: f32) -> AhrsResult<Vec<StateVector>> {
        // Validate input
        if dt <= 0.0 || dt.is_nan() {
            return Err(AhrsError::TimingError(format!("Invalid time step: {}", dt)));
        }
        
        // Calculate mixing probabilities
        if let Err(e) = self.update_mixing_probabilities() {
            return Err(e);
        }
        
        // Initialize mixed states
        let mixed_states = vec![state.clone(); 3];
        
        // Step 1: Mixing (interaction)
        // Mix the states for each model based on mixing probabilities
        
        // Step 2: Mode-matched filtering
        // Predict using each model
        let mut predicted_states = Vec::with_capacity(3);
        let mut model_failure_indices = Vec::new();

        for (i, model) in self.models.iter().enumerate() {
            match model.predict(&mixed_states[i], dt) {
                Ok(predicted) => {
                    self.model_states[i] = predicted.clone();
                    predicted_states.push(predicted);
                },
                Err(e) => {
                    // If a model fails, use the input state instead
                    // and mark for probability reduction
                    self.model_states[i] = state.clone();
                    model_failure_indices.push(i);
                    
                    // Add the original state to predicted states for consistency
                    predicted_states.push(state.clone());
                    
                    // Log the error but continue with other models
                    // In a real implementation, you might want to handle this differently
                    eprintln!("Error in model {}: {:?}", i, e);
                }
            }
        }

        // Now that we're done with the immutable borrow of self.models, we can modify probabilities
        for i in &model_failure_indices {
            self.model_probs[*i] *= 0.5;
        }
        
        // Normalize probabilities if there were any failures
        if !model_failure_indices.is_empty() {
            self.normalize_probabilities();
        }
        
        // Update model probabilities based on likelihood
        // In a real implementation, you would calculate likelihood based on measurement innovations
        self.update_model_probabilities()?;
        
        Ok(predicted_states)
    }
    
    /// Update the mixing probabilities
    fn update_mixing_probabilities(&mut self) -> AhrsResult<()> {
        // For each target model j
        for j in 0..3 {
            // Calculate normalization factor
            let mut c_j = 0.0;
            for i in 0..3 {
                c_j += self.transition_matrix[i][j] * self.model_probs[i];
            }
            
            // Avoid division by zero
            if c_j.abs() < 1e-10 {
                return Err(AhrsError::InvalidState);
            }
            
            // Calculate mixing probabilities
            for i in 0..3 {
                self.mixing_probs[i][j] = self.transition_matrix[i][j] * self.model_probs[i] / c_j;
            }
        }
        
        Ok(())
    }
    
    /// Update model probabilities based on measurements
    fn update_model_probabilities(&mut self) -> AhrsResult<()> {
        // In a real implementation, this would use measurement likelihoods
        // For now, we just use some heuristic based on model state consistency
        
        // Calculate likelihood for each model
        let mut likelihoods = [0.0; 3];
        for i in 0..3 {
            // Simple heuristic based on state consistency
            // A real implementation would use measurement innovations
            let state = &self.model_states[i];
            
            // Check if the state is valid
            if state.position.iter().any(|v| v.is_nan() || v.is_infinite()) ||
               state.velocity.iter().any(|v| v.is_nan() || v.is_infinite()) {
                likelihoods[i] = 0.01; // Very low likelihood for invalid states
            } else {
                // Some arbitrary likelihood calculation
                likelihoods[i] = 1.0;
            }
        }
        
        // Update model probabilities
        let mut sum = 0.0;
        for i in 0..3 {
            self.model_probs[i] *= likelihoods[i];
            sum += self.model_probs[i];
        }
        
        // Normalize probabilities to avoid numerical issues
        // Avoid division by zero
        if sum < 1e-10 {
            // Reset to equal probabilities if sum is too small
            self.model_probs = [1.0/3.0, 1.0/3.0, 1.0/3.0];
        } else {
            for i in 0..3 {
                self.model_probs[i] /= sum;
            }
        }
        
        Ok(())
    }
    
    /// Ensure model probabilities sum to 1.0
    fn normalize_probabilities(&mut self) {
        let sum: f32 = self.model_probs.iter().sum();
        
        if sum > 0.0 {
            for prob in self.model_probs.iter_mut() {
                *prob /= sum;
            }
        } else {
            // Reset to equal probabilities if sum is zero or negative
            self.model_probs = [1.0/3.0, 1.0/3.0, 1.0/3.0];
        }
    }
    
    /// Combine the states from all models to get the final state estimate
    pub fn combine_states(&self) -> StateVector {
        let mut combined = StateVector::new();
        
        // Reset to zeros for combining
        combined.position = na::Vector3::zeros();
        combined.velocity = na::Vector3::zeros();
        
        // Combined state is weighted sum of individual model states
        for i in 0..3 {
            let weight = self.model_probs[i];
            combined.position += self.model_states[i].position * weight;
            combined.velocity += self.model_states[i].velocity * weight;
            
            // For quaternions, we should use spherical linear interpolation (SLERP)
            // But for simplicity, we'll use the state from the most likely model
        }
        
        // For non-additive states like quaternions, use the most likely model
        let most_likely_idx = self.most_likely_model();
        combined.attitude = self.model_states[most_likely_idx].attitude;
        combined.accel_bias = self.model_states[most_likely_idx].accel_bias;
        combined.gyro_bias = self.model_states[most_likely_idx].gyro_bias;
        
        // Covariance should be combined with proper mathematics
        // This is a simplification
        combined.position_covariance = self.model_states[most_likely_idx].position_covariance;
        combined.velocity_covariance = self.model_states[most_likely_idx].velocity_covariance;
        combined.attitude_covariance = self.model_states[most_likely_idx].attitude_covariance;
        
        combined
    }
    
    /// Get the index of the most likely model
    pub fn most_likely_model(&self) -> usize {
        let mut max_prob = self.model_probs[0];
        let mut max_idx = 0;
        
        for i in 1..3 {
            if self.model_probs[i] > max_prob {
                max_prob = self.model_probs[i];
                max_idx = i;
            }
        }
        
        max_idx
    }
    
    /// Get the name of the most likely model
    pub fn most_likely_model_name(&self) -> &'static str {
        self.models[self.most_likely_model()].name()
    }
    
    /// Get the probability of a specific model
    pub fn model_probability(&self, model_idx: usize) -> f32 {
        if model_idx < 3 {
            self.model_probs[model_idx]
        } else {
            0.0
        }
    }
} 