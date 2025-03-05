use crate::models::{ModelFilter, create_model_filters, ModelFilterEnum};
use crate::sensors::{BaroData, GpsData, ImuData};
use crate::{AhrsConfig, AhrsResult, StateVector};
use crate::error::helpers;
use nalgebra as na;

/// The number of model filters used in the IMM
pub const NUM_MODELS: usize = 4;

/// Default model transition probability to stay in the same model
const DEFAULT_STAY_PROBABILITY: f32 = 0.85;

/// Minimum allowed probability value to prevent numerical issues
const MINIMUM_PROBABILITY_THRESHOLD: f32 = 0.001;

/// Numerical stability threshold for near-zero values
const NUMERICAL_STABILITY_THRESHOLD: f32 = 1e-10;

/// Minimum allowed value for minimum probability
const MIN_ALLOWED_PROBABILITY: f32 = 0.0;

/// Maximum allowed value for minimum probability
const MAX_ALLOWED_PROBABILITY: f32 = 0.5;

/// Standard Interacting Multiple Model (IMM) filter implementation
/// This implementation follows the standard IMM algorithm:
/// 1. Interaction/Mixing: Compute mixed initial conditions for each filter
/// 2. Filtering: Apply each filter to its mixed initial state
/// 3. Mode probability update: Update the probability of each model
/// 4. Combination: Combine state estimates and covariances across models
pub struct Imm {
    /// The model filters - fixed array of motion models using stack allocation
    model_filters: [ModelFilterEnum; NUM_MODELS],

    /// Model probabilities
    model_probs: [f32; NUM_MODELS],

    /// Model transition matrix - 4x4 matrix of transition probabilities
    transition_matrix: [[f32; NUM_MODELS]; NUM_MODELS],

    /// Mixing probabilities - 4x4 matrix
    mixing_probs: [[f32; NUM_MODELS]; NUM_MODELS],

    /// Mixed states for each model
    mixed_states: [StateVector; NUM_MODELS],
    
    /// Flag indicating if the filter has been initialized
    initialized: bool,
    
    /// Minimum allowed model probability
    min_probability: f32,
}

impl Imm {
    /// Create a new IMM instance with the standard model set
    pub fn new(config: &AhrsConfig) -> AhrsResult<Self> {
        // Create model filters
        let model_filters = create_model_filters(config)?;
        
        // Initialize model probabilities with the provided weights or defaults
        let mut model_probs = [1.0 / (NUM_MODELS as f32); NUM_MODELS]; // Default to equal probabilities
        
        // Use the provided model weights from config
        let sum: f32 = config.model_weights.iter().sum();
        if sum > 0.0 {
            for i in 0..NUM_MODELS {
                model_probs[i] = config.model_weights[i] / sum;
            }
        }
        
        // Initialize transition matrix with default values
        // This represents the probability of switching from model i to model j
        let mut transition_matrix = [[0.0; NUM_MODELS]; NUM_MODELS];
        
        // Default: high probability of staying in same model, low probability of transition
        let stay_prob = DEFAULT_STAY_PROBABILITY;
        let transition_prob = (1.0 - stay_prob) / (NUM_MODELS as f32 - 1.0);
        
        for i in 0..NUM_MODELS {
            for j in 0..NUM_MODELS {
                if i == j {
                    transition_matrix[i][j] = stay_prob;
                } else {
                    transition_matrix[i][j] = transition_prob;
                }
            }
        }
        
        // Initialize mixing probabilities matrix
        let mixing_probs = [[0.0; NUM_MODELS]; NUM_MODELS];
        
        // Initialize each model with a default state
        let default_state = StateVector::default();
        let mixed_states = [default_state.clone(), default_state.clone(), 
                          default_state.clone(), default_state.clone()];
        
        Ok(Self {
            model_filters,
            model_probs,
            transition_matrix,
            mixing_probs,
            mixed_states,
            initialized: false,
            min_probability: MINIMUM_PROBABILITY_THRESHOLD, // Default minimum probability
        })
    }
    
    /// Initialize the IMM with an initial state
    pub fn initialize(&mut self, initial_state: &StateVector) -> AhrsResult<()> {
        // Initialize each filter with the initial state
        for filter in &mut self.model_filters {
            filter.initialize(initial_state)?;
        }
        
        // Reset mixed states
        self.mixed_states = [initial_state.clone(), initial_state.clone(), 
                          initial_state.clone(), initial_state.clone()];
        
        self.initialized = true;
        
        Ok(())
    }
    
    /// Update mixing probabilities based on model probabilities and transition matrix
    fn update_mixing_probabilities(&mut self) -> AhrsResult<()> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("update_mixing_probabilities".to_string()),
            ));
        }
        
        // For each target model j
        for j in 0..NUM_MODELS {
            // Calculate normalization factor c_j
            let mut c_j = 0.0;
            for i in 0..NUM_MODELS {
                c_j += self.transition_matrix[i][j] * self.model_probs[i];
            }
            
            // Avoid division by zero
            if c_j < NUMERICAL_STABILITY_THRESHOLD {
                // If normalization is near zero, set uniform mixing probabilities
                for i in 0..NUM_MODELS {
                    self.mixing_probs[i][j] = 1.0 / (NUM_MODELS as f32);
                }
            } else {
                // Calculate mixing probabilities μ_{i|j}
                for i in 0..NUM_MODELS {
                    self.mixing_probs[i][j] = 
                        self.transition_matrix[i][j] * self.model_probs[i] / c_j;
                }
            }
        }
        
        Ok(())
    }
    
    /// Compute mixed initial states for each filter
    fn mix_states(&mut self) -> AhrsResult<()> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("mix_states".to_string()),
            ));
        }
        
        // For each model j, compute the mixed state
        for j in 0..NUM_MODELS {
            // Reset the mixed state
            let mut mixed_state = StateVector::new();
            
            // For position, velocity and biases, we can do weighted averaging
            mixed_state.position = na::Vector3::zeros();
            mixed_state.velocity = na::Vector3::zeros();
            mixed_state.accel_bias = na::Vector3::zeros();
            mixed_state.gyro_bias = na::Vector3::zeros();
            
            // Weighted average of position, velocity, and biases
            for i in 0..NUM_MODELS {
                let state = self.model_filters[i].get_state();
                let weight = self.mixing_probs[i][j];
                
                mixed_state.position += state.position * weight;
                mixed_state.velocity += state.velocity * weight;
                mixed_state.accel_bias += state.accel_bias * weight;
                mixed_state.gyro_bias += state.gyro_bias * weight;
            }
            
            // For quaternions, we need to use weighted quaternion averaging
            // We'll use a simplified approach here: weighted sum of quaternion components then normalize
            let mut q_sum = na::Quaternion::new(0.0, 0.0, 0.0, 0.0);
            let mut total_weight = 0.0;
            
            for i in 0..NUM_MODELS {
                let state = self.model_filters[i].get_state();
                let weight = self.mixing_probs[i][j];
                
                if weight > 1e-6 {
                    // Get the quaternion
                    let q = state.attitude.into_inner();
                    
                    // Ensure quaternions are in the same hemisphere (dot product > 0)
                    let reference_q = if j > 0 {
                        self.model_filters[0].get_state().attitude.into_inner()
                    } else {
                        na::Quaternion::identity()
                    };
                    
                    let dot = q.dot(&reference_q);
                    let q_aligned = if dot < 0.0 {
                        -q // Flip to the same hemisphere
                    } else {
                        q
                    };
                    
                    // Add weighted quaternion
                    q_sum.coords += q_aligned.coords * weight;
                    total_weight += weight;
                }
            }
            
            // Normalize the quaternion
            if total_weight > 1e-6 {
                q_sum.coords /= total_weight;
                mixed_state.attitude = na::UnitQuaternion::from_quaternion(
                    na::Quaternion::from_parts(q_sum.scalar(), q_sum.vector())
                );
            } else {
                mixed_state.attitude = na::UnitQuaternion::identity();
            }
            
            // Also mix the covariance matrices
            self.mix_covariance_matrices(&mut mixed_state, j)?;
            
            // Store mixed state
            self.mixed_states[j] = mixed_state;
            
            // Update the filter with the mixed state
            self.model_filters[j].set_state(&self.mixed_states[j]);
        }
        
        Ok(())
    }
    
    /// Mix covariance matrices for a given model
    fn mix_covariance_matrices(&self, mixed_state: &mut StateVector, model_idx: usize) -> AhrsResult<()> {
        // Initialize mixed covariance matrices to zero
        let mut mixed_pos_cov = na::Matrix3::zeros();
        let mut mixed_vel_cov = na::Matrix3::zeros();
        let mut mixed_att_cov = na::Matrix3::zeros();
        
        // Calculate weighted sum of covariances
        for i in 0..NUM_MODELS {
            let state = self.model_filters[i].get_state();
            let weight = self.mixing_probs[i][model_idx];
            
            if weight > 1e-6 {
                // Add weighted covariance
                mixed_pos_cov += state.position_covariance * weight;
                mixed_vel_cov += state.velocity_covariance * weight;
                mixed_att_cov += state.attitude_covariance * weight;
            }
        }
        
        // Assign mixed covariances to the mixed state
        mixed_state.position_covariance = mixed_pos_cov;
        mixed_state.velocity_covariance = mixed_vel_cov;
        mixed_state.attitude_covariance = mixed_att_cov;
        
        Ok(())
    }
    
    /// Find the model with highest mixing probability for a given target model
    fn most_likely_model_for_mixing(&self, target_model: usize) -> usize {
        let mut max_prob = 0.0;
        let mut max_idx = 0;
        
        for i in 0..NUM_MODELS {
            if self.mixing_probs[i][target_model] > max_prob {
                max_prob = self.mixing_probs[i][target_model];
                max_idx = i;
            }
        }
        
        max_idx
    }
    
    /// Update model probabilities based on measurement likelihoods
    fn update_model_probabilities(&mut self, innovations: &[na::Vector6<f32>; NUM_MODELS], 
                                innovation_covariances: &[na::Matrix6<f32>; NUM_MODELS]) -> AhrsResult<()> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("update_model_probabilities".to_string()),
            ));
        }
        
        // Calculate likelihood for each model using fixed-size array
        let mut likelihoods = [0.0; NUM_MODELS];
        for j in 0..NUM_MODELS {
            likelihoods[j] = self.model_filters[j].likelihood(&innovations[j], &innovation_covariances[j]);
        }
        
        // Calculate new model probabilities
        let mut sum = 0.0;
        for j in 0..NUM_MODELS {
            self.model_probs[j] *= likelihoods[j];
            sum += self.model_probs[j];
        }
        
        // Normalize probabilities
        if sum < NUMERICAL_STABILITY_THRESHOLD {
            // If all probabilities are near zero, reset to equal probabilities
            for j in 0..NUM_MODELS {
                self.model_probs[j] = 1.0 / (NUM_MODELS as f32);
            }
        } else {
            for j in 0..NUM_MODELS {
                self.model_probs[j] /= sum;
                
                // Apply minimum probability constraint
                if self.model_probs[j] < self.min_probability {
                    self.model_probs[j] = self.min_probability;
                }
            }
            
            // Re-normalize after applying minimum probability constraint
            sum = self.model_probs.iter().sum();
            for j in 0..NUM_MODELS {
                self.model_probs[j] /= sum;
            }
        }
        
        Ok(())
    }
    
    /// Run prediction step for all models
    pub fn predict(&mut self, imu_data: &ImuData, dt: f32) -> AhrsResult<StateVector> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("predict".to_string()),
            ));
        }
        
        // Step 1: Calculate mixing probabilities
        self.update_mixing_probabilities()?;
        
        // Step 2: Mix states
        self.mix_states()?;
        
        // Step 3: Run prediction for each model
        for filter in &mut self.model_filters {
            filter.predict(imu_data, dt)?;
        }
        
        // Step 4: Combine state estimates
        Ok(self.combine_states())
    }
    
    /// Run update step for all models with GPS data
    pub fn update_gps(&mut self, gps_data: &GpsData) -> AhrsResult<StateVector> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("update_gps".to_string()),
            ));
        }
        
        // Use fixed-size arrays instead of Vec
        let mut innovations = [na::Vector6::zeros(); NUM_MODELS];
        let mut innovation_covariances = [na::Matrix6::zeros(); NUM_MODELS];
        
        for i in 0..NUM_MODELS {
            let (innovation, innovation_cov) = self.model_filters[i].gps_innovation(gps_data);
            innovations[i] = innovation;
            innovation_covariances[i] = innovation_cov;
        }
        
        // Update each model
        for filter in &mut self.model_filters {
            filter.update_gps(gps_data)?;
        }
        
        // Update model probabilities
        self.update_model_probabilities(&innovations, &innovation_covariances)?;
        
        // Combine state estimates
        Ok(self.combine_states())
    }
    
    /// Run update step for all models with barometer data
    pub fn update_baro(&mut self, baro_data: &BaroData) -> AhrsResult<StateVector> {
        if !self.initialized {
            return Err(helpers::init_error(
                "IMM filter not initialized".to_string(),
                Some("update_baro".to_string()),
            ));
        }
        
        // Update each model
        for filter in &mut self.model_filters {
            filter.update_baro(baro_data)?;
        }
        
        // Probabilities can't be updated without innovations
        // For barometer, we don't adjust model probabilities
        
        // Combine state estimates
        Ok(self.combine_states())
    }
    
    /// Combine state estimates from all models
    pub fn combine_states(&self) -> StateVector {
        // Initialize combined state
        let mut combined_state = StateVector::new();
        
        // For position, velocity and biases, we can do weighted averaging
        combined_state.position = na::Vector3::zeros();
        combined_state.velocity = na::Vector3::zeros();
        combined_state.accel_bias = na::Vector3::zeros();
        combined_state.gyro_bias = na::Vector3::zeros();
        
        // Combined covariance matrices (initialized to zero)
        let mut combined_pos_cov = na::Matrix3::zeros();
        let mut combined_vel_cov = na::Matrix3::zeros();
        let mut combined_att_cov = na::Matrix3::zeros();
        
        // First calculate weighted means
        for j in 0..NUM_MODELS {
            let state = self.model_filters[j].get_state();
            let weight = self.model_probs[j];
            
            // Skip if weight is too small
            if weight < 1e-6 {
                continue;
            }
            
            // Add weighted components
            combined_state.position += state.position * weight;
            combined_state.velocity += state.velocity * weight;
            combined_state.accel_bias += state.accel_bias * weight;
            combined_state.gyro_bias += state.gyro_bias * weight;
        }
        
        // Special handling for attitude (quaternion)
        // We use the SLERP approach for combining multiple quaternions
        
        // First identify the most likely model to use as reference
        let most_likely_idx = self.most_likely_model();
        let reference_q = self.model_filters[most_likely_idx].get_state().attitude;
        
        // Initialize weighted quaternion
        let mut q_sum = na::Quaternion::new(0.0, 0.0, 0.0, 0.0);
        let mut total_weight = 0.0;
        
        // Combine quaternions
        for j in 0..NUM_MODELS {
            let state = self.model_filters[j].get_state();
            let weight = self.model_probs[j];
            
            if weight > 1e-6 {
                // Get the quaternion
                let q = state.attitude.into_inner();
                
                // Ensure quaternions are in the same hemisphere
                let dot = q.dot(&reference_q.into_inner());
                let q_aligned = if dot < 0.0 {
                    -q // Flip to the same hemisphere
                } else {
                    q
                };
                
                // Add weighted quaternion
                q_sum.coords += q_aligned.coords * weight;
                total_weight += weight;
            }
        }
        
        // Normalize the quaternion
        if total_weight > 1e-6 {
            q_sum.coords /= total_weight;
            combined_state.attitude = na::UnitQuaternion::from_quaternion(
                na::Quaternion::from_parts(q_sum.scalar(), q_sum.vector())
            );
        } else {
            combined_state.attitude = reference_q;
        }
        
        // Calculate combined covariance with spread of means
        for j in 0..NUM_MODELS {
            let state = self.model_filters[j].get_state();
            let weight = self.model_probs[j];
            
            if weight < 1e-6 {
                continue;
            }
            
            // Within-model covariance contribution
            combined_pos_cov += state.position_covariance * weight;
            combined_vel_cov += state.velocity_covariance * weight;
            combined_att_cov += state.attitude_covariance * weight;
            
            // Between-model spread contribution (for position)
            let pos_diff = state.position - combined_state.position;
            let pos_spread = pos_diff * pos_diff.transpose() * weight;
            combined_pos_cov += pos_spread;
            
            // Between-model spread contribution (for velocity)
            let vel_diff = state.velocity - combined_state.velocity;
            let vel_spread = vel_diff * vel_diff.transpose() * weight;
            combined_vel_cov += vel_spread;
            
            // For attitude, we need a different approach - using Euler angles for simplicity
            let state_euler = state.attitude.euler_angles();
            let combined_euler = combined_state.attitude.euler_angles();
            
            let att_diff = na::Vector3::new(
                state_euler.0 - combined_euler.0,
                state_euler.1 - combined_euler.1,
                state_euler.2 - combined_euler.2,
            );
            
            let att_spread = att_diff * att_diff.transpose() * weight;
            combined_att_cov += att_spread;
        }
        
        // Assign combined covariances
        combined_state.position_covariance = combined_pos_cov;
        combined_state.velocity_covariance = combined_vel_cov;
        combined_state.attitude_covariance = combined_att_cov;
        
        combined_state
    }
    
    /// Get the index of the most likely model
    pub fn most_likely_model(&self) -> usize {
        let mut max_prob = 0.0;
        let mut max_idx = 0;
        
        for (i, &prob) in self.model_probs.iter().enumerate() {
            if prob > max_prob {
                max_prob = prob;
                max_idx = i;
            }
        }
        
        max_idx
    }
    
    /// Get the name of the most likely model
    pub fn most_likely_model_name(&self) -> &str {
        let idx = self.most_likely_model();
        self.model_filters[idx].model_name()
    }
    
    /// Get current model probabilities
    pub fn get_model_probabilities(&self) -> &[f32] {
        &self.model_probs
    }
    
    /// Set custom transition matrix
    pub fn set_transition_matrix(&mut self, matrix: [[f32; NUM_MODELS]; NUM_MODELS]) -> AhrsResult<()> {
        // Validate each row sums to 1.0
        for row in &matrix {
            let sum: f32 = row.iter().sum();
            if (sum - 1.0).abs() > NUMERICAL_STABILITY_THRESHOLD {
                return Err(helpers::init_error(
                    format!("Transition matrix row does not sum to 1.0: {}", sum),
                    Some("set_transition_matrix".to_string()),
                ));
            }
        }
        
        // Copy values from the input array
        self.transition_matrix = matrix;
        
        Ok(())
    }
    
    /// Set custom transition matrix from slices
    pub fn set_transition_matrix_from_slices(&mut self, matrix: &[&[f32]]) -> AhrsResult<()> {
        // Validate matrix dimensions
        if matrix.len() != NUM_MODELS {
            return Err(helpers::init_error(
                format!("Invalid transition matrix size: expected {} rows", NUM_MODELS),
                Some("set_transition_matrix_from_slices".to_string()),
            ));
        }
        
        for row in matrix {
            if row.len() != NUM_MODELS {
                return Err(helpers::init_error(
                    format!("Invalid transition matrix size: expected {} columns", NUM_MODELS),
                    Some("set_transition_matrix_from_slices".to_string()),
                ));
            }
        }
        
        // Validate each row sums to 1.0
        for row in matrix {
            let sum: f32 = row.iter().sum();
            if (sum - 1.0).abs() > NUMERICAL_STABILITY_THRESHOLD {
                return Err(helpers::init_error(
                    format!("Transition matrix row does not sum to 1.0: {}", sum),
                    Some("set_transition_matrix_from_slices".to_string()),
                ));
            }
        }
        
        // Copy values from the slices
        for i in 0..NUM_MODELS {
            for j in 0..NUM_MODELS {
                self.transition_matrix[i][j] = matrix[i][j];
            }
        }
        
        Ok(())
    }
    
    /// Set the minimum allowed model probability
    pub fn set_min_probability(&mut self, min_prob: f32) -> AhrsResult<()> {
        // Validate min probability
        if min_prob < MIN_ALLOWED_PROBABILITY || min_prob > MAX_ALLOWED_PROBABILITY || min_prob.is_nan() {
            return Err(helpers::config_error(
                format!("Invalid minimum probability value: {}", min_prob),
                Some("min_probability".to_string())
            ));
        }
        
        self.min_probability = min_prob;
        Ok(())
    }
}
