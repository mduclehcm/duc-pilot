// Autotune PID parameters
use crate::pid::PID;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum AutotuneError {
    #[error("Insufficient data for autotuning")]
    InsufficientData,
    
    #[error("No oscillation detected")]
    NoOscillation,
    
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),
}

/// AutotuneMethod defines the algorithm used for PID autotuning
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AutotuneMethod {
    /// Ziegler-Nichols method
    ZieglerNichols,
    /// Cohen-Coon method
    CohenCoon,
    /// AMIGO (Approximate M-constrained Integral Gain Optimization)
    AMIGO,
    /// Tyreus-Luyben method
    TyreusLuyben,
}

/// PIDType defines different types of PID controllers
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PIDType {
    P,     // Proportional only
    PI,    // Proportional + Integral
    PD,    // Proportional + Derivative
    PID,   // Full PID
}

/// Results of a successful autotuning operation
#[derive(Debug, Clone)]
pub struct AutotuneResult {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub ultimate_gain: f32,      // Ku
    pub ultimate_period: f32,    // Tu (seconds)
}

/// Struct for PID autotuning using the Relay method
pub struct Autotune {
    // Configuration
    method: AutotuneMethod,
    pid_type: PIDType,
    relay_amplitude: f32,
    test_time: f32,              // Maximum test time in seconds
    sample_time: f32,            // Sample time in seconds
    noise_band: f32,             // To filter out noise
    
    // Internal state
    setpoint: f32,
    process_variable: Vec<f32>,  // History of process variable
    time_stamps: Vec<f32>,       // Timestamps of measurements
    relay_output: Vec<f32>,      // History of relay outputs
    relay_state: bool,           // Current state of the relay
    last_peak_time: f32,         // Time of the last peak
    last_zero_crossing_time: f32,// Time of the last zero crossing
    peaks: Vec<(f32, f32)>,      // Peaks found (time, value)
    zero_crossings: Vec<f32>,    // Times of zero crossings
    
    // Analysis
    is_complete: bool,
    result: Option<AutotuneResult>,
}

impl Autotune {
    /// Create a new Autotune instance
    pub fn new(method: AutotuneMethod, pid_type: PIDType, setpoint: f32) -> Self {
        Self {
            method,
            pid_type,
            relay_amplitude: 1.0,
            test_time: 300.0,          // 5 minutes by default
            sample_time: 0.01,         // 10ms by default
            noise_band: 0.1,           // 10% of relay amplitude
            
            setpoint,
            process_variable: Vec::new(),
            time_stamps: Vec::new(),
            relay_output: Vec::new(),
            relay_state: false,
            last_peak_time: 0.0,
            last_zero_crossing_time: 0.0,
            peaks: Vec::new(),
            zero_crossings: Vec::new(),
            
            is_complete: false,
            result: None,
        }
    }
    
    /// Set the relay amplitude
    pub fn with_relay_amplitude(mut self, amplitude: f32) -> Result<Self, AutotuneError> {
        if amplitude <= 0.0 {
            return Err(AutotuneError::InvalidParameter("Relay amplitude must be positive".to_string()));
        }
        self.relay_amplitude = amplitude;
        Ok(self)
    }
    
    /// Set the test time
    pub fn with_test_time(mut self, test_time: f32) -> Result<Self, AutotuneError> {
        if test_time <= 0.0 {
            return Err(AutotuneError::InvalidParameter("Test time must be positive".to_string()));
        }
        self.test_time = test_time;
        Ok(self)
    }
    
    /// Set the sample time
    pub fn with_sample_time(mut self, sample_time: f32) -> Result<Self, AutotuneError> {
        if sample_time <= 0.0 {
            return Err(AutotuneError::InvalidParameter("Sample time must be positive".to_string()));
        }
        self.sample_time = sample_time;
        Ok(self)
    }
    
    /// Set the noise band
    pub fn with_noise_band(mut self, noise_band: f32) -> Result<Self, AutotuneError> {
        if noise_band < 0.0 {
            return Err(AutotuneError::InvalidParameter("Noise band must be non-negative".to_string()));
        }
        self.noise_band = noise_band;
        Ok(self)
    }
    
    /// Process a new measurement and get the relay output
    pub fn process(&mut self, pv: f32, time: f32) -> f32 {
        // Store data
        self.process_variable.push(pv);
        self.time_stamps.push(time);
        
        // Calculate error
        let error = self.setpoint - pv;
        
        // Check for zero crossing (sign change of error)
        if self.process_variable.len() > 1 {
            let last_error = self.setpoint - self.process_variable[self.process_variable.len() - 2];
            if error * last_error <= 0.0 && (error != 0.0 || last_error != 0.0) {
                self.zero_crossings.push(time);
                self.last_zero_crossing_time = time;
            }
        }
        
        // Implement the relay with hysteresis
        if error > self.noise_band {
            self.relay_state = true;
        } else if error < -self.noise_band {
            self.relay_state = false;
        }
        
        let output = if self.relay_state {
            self.relay_amplitude
        } else {
            -self.relay_amplitude
        };
        
        self.relay_output.push(output);
        
        // Detect peaks (local extrema)
        if self.process_variable.len() >= 3 {
            let n = self.process_variable.len();
            let prev = self.process_variable[n - 3];
            let curr = self.process_variable[n - 2];
            let next = self.process_variable[n - 1];
            
            // Check if the middle point is a peak
            if (curr > prev && curr > next) || (curr < prev && curr < next) {
                let peak_time = self.time_stamps[n - 2];
                let peak_value = curr;
                self.peaks.push((peak_time, peak_value));
                self.last_peak_time = peak_time;
            }
        }
        
        // Check if we can complete the autotuning
        if time >= self.test_time || self.is_ready_to_complete() {
            if !self.is_complete {
                self.complete();
            }
        }
        
        output
    }
    
    /// Check if we have enough data to complete the autotuning
    fn is_ready_to_complete(&self) -> bool {
        // We need at least 4 peaks to have 2 full oscillations
        if self.peaks.len() < 4 {
            return false;
        }
        
        // We need at least 3 zero crossings to measure the period
        if self.zero_crossings.len() < 3 {
            return false;
        }
        
        true
    }
    
    /// Complete the autotuning process and calculate PID parameters
    fn complete(&mut self) {
        // Calculate the period from zero crossings
        let mut periods = Vec::new();
        for i in 1..self.zero_crossings.len() {
            periods.push(self.zero_crossings[i] - self.zero_crossings[i - 1]);
        }
        
        // Calculate amplitude of oscillation
        let mut peak_to_peak = 0.0;
        if self.peaks.len() >= 2 {
            // Find the average peak-to-peak amplitude
            let mut amplitudes = Vec::new();
            let mut i = 0;
            while i + 1 < self.peaks.len() {
                amplitudes.push((self.peaks[i].1 - self.peaks[i+1].1).abs());
                i += 2; // Skip to next pair
            }
            if !amplitudes.is_empty() {
                peak_to_peak = amplitudes.iter().sum::<f32>() / amplitudes.len() as f32;
            }
        }
        
        // Check if peak_to_peak is too small (near zero)
        if peak_to_peak < 1e-6 {
            // Not enough oscillation detected, we cannot complete autotuning
            self.is_complete = false;
            self.result = None;
            return;
        }
        
        // Calculate critical gain (Ku)
        // Ku = (4 * relay_amplitude) / (π * peak_to_peak)
        let ultimate_gain = (4.0 * self.relay_amplitude) / (std::f32::consts::PI * peak_to_peak);
        
        // Handle potential NaN or Infinity values
        if ultimate_gain.is_nan() || ultimate_gain.is_infinite() {
            self.is_complete = false;
            self.result = None;
            return;
        }
        
        // Check if periods vector is empty
        if periods.is_empty() {
            self.is_complete = false;
            self.result = None;
            return;
        }
        
        // Average period (2 zero crossings = 1 half-period)
        let avg_half_period: f32 = periods.iter().sum::<f32>() / periods.len() as f32;
        let ultimate_period = avg_half_period * 2.0;
        
        // Handle potential NaN or Infinity values
        if ultimate_period.is_nan() || ultimate_period.is_infinite() || ultimate_period <= 0.0 {
            self.is_complete = false;
            self.result = None;
            return;
        }
        
        // Calculate PID parameters based on the selected method
        let params = self.calculate_params(ultimate_gain, ultimate_period);
        
        self.is_complete = true;
        self.result = Some(params);
    }
    
    /// Calculate PID parameters using the selected method
    fn calculate_params(&self, ku: f32, tu: f32) -> AutotuneResult {
        let (kp, ki, kd) = match (self.method, self.pid_type) {
            (AutotuneMethod::ZieglerNichols, PIDType::P) => (0.5 * ku, 0.0, 0.0),
            (AutotuneMethod::ZieglerNichols, PIDType::PI) => (0.45 * ku, 0.54 * ku / tu, 0.0),
            (AutotuneMethod::ZieglerNichols, PIDType::PD) => (0.8 * ku, 0.0, 0.1 * ku * tu),
            (AutotuneMethod::ZieglerNichols, PIDType::PID) => (0.6 * ku, 1.2 * ku / tu, 0.075 * ku * tu),
            
            (AutotuneMethod::TyreusLuyben, PIDType::PI) => (0.31 * ku, 0.31 * ku / (2.2 * tu), 0.0),
            (AutotuneMethod::TyreusLuyben, PIDType::PID) => (0.45 * ku, 0.45 * ku / (2.2 * tu), 0.45 * ku * tu / 6.3),
            
            (AutotuneMethod::AMIGO, PIDType::PI) => (0.4 * ku, 0.4 * ku / (0.8 * tu), 0.0),
            (AutotuneMethod::AMIGO, PIDType::PID) => (0.35 * ku, 0.35 * ku / (0.8 * tu), 0.35 * ku * tu / 8.0),
            
            // Cohen-Coon method implementation
            (AutotuneMethod::CohenCoon, PIDType::P) => {
                // Cohen-Coon P controller
                let kp = 1.0 / (ku * tu) * (0.9 + tu / 12.0);
                (kp, 0.0, 0.0)
            },
            (AutotuneMethod::CohenCoon, PIDType::PI) => {
                // Cohen-Coon PI controller
                let kp = 0.9 / (ku * tu) * (1.0 + tu / 10.0);
                let ti = tu * (30.0 + 3.0 * tu) / (9.0 + 20.0 * tu);
                let ki = kp / ti;
                (kp, ki, 0.0)
            },
            (AutotuneMethod::CohenCoon, PIDType::PID) => {
                // Cohen-Coon PID controller
                let kp = 1.35 / (ku * tu) * (1.0 + tu / 5.0);
                let ti = tu * (40.0 + 5.0 * tu) / (13.0 + 45.0 * tu);
                let td = tu * 4.0 / (11.0 + 2.0 * tu);
                let ki = kp / ti;
                let kd = kp * td;
                (kp, ki, kd)
            },
            
            // Fall back to Ziegler-Nichols for any other combination
            (_, _) => {
                let kp = 0.6 * ku;
                let ki = 1.2 * ku / tu;
                let kd = 0.075 * ku * tu;
                (kp, ki, kd)
            }
        };
        
        AutotuneResult {
            kp,
            ki,
            kd,
            ultimate_gain: ku,
            ultimate_period: tu,
        }
    }
    
    /// Check if autotuning is complete
    pub fn is_complete(&self) -> bool {
        self.is_complete
    }
    
    /// Get the result of autotuning
    pub fn result(&self) -> Option<&AutotuneResult> {
        self.result.as_ref()
    }
    
    /// Get the result and create a PID controller with the tuned parameters
    pub fn get_tuned_pid(&self) -> Result<PID, AutotuneError> {
        match &self.result {
            Some(result) => Ok(PID::new(result.kp, result.ki, result.kd, 0.0)),
            None => Err(AutotuneError::InsufficientData),
        }
    }
    
    /// Get the raw data collected during autotuning
    pub fn get_data(&self) -> (&[f32], &[f32], &[f32]) {
        (&self.time_stamps, &self.process_variable, &self.relay_output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cohen_coon_calculation() {
        // Create a mock Autotune instance for testing
        let autotune = Autotune::new(AutotuneMethod::CohenCoon, PIDType::PID, 1.0);
        
        // Simulate completing the tuning with some realistic values
        let ultimate_gain = 2.0;      // Ku
        let ultimate_period = 1.5;    // Tu
        
        // Calculate PID parameters using Cohen-Coon method
        let result = autotune.calculate_params(ultimate_gain, ultimate_period);
        
        // Verify the results are reasonable and correct
        assert!(result.kp > 0.0, "Cohen-Coon kp should be positive");
        assert!(result.ki > 0.0, "Cohen-Coon ki should be positive");
        assert!(result.kd > 0.0, "Cohen-Coon kd should be positive");
        
        // Test for other PID types
        let pi_autotune = Autotune::new(AutotuneMethod::CohenCoon, PIDType::PI, 1.0);
        let pi_result = pi_autotune.calculate_params(ultimate_gain, ultimate_period);
        assert!(pi_result.kp > 0.0, "Cohen-Coon PI kp should be positive");
        assert!(pi_result.ki > 0.0, "Cohen-Coon PI ki should be positive");
        assert_eq!(pi_result.kd, 0.0, "Cohen-Coon PI kd should be zero");
        
        let p_autotune = Autotune::new(AutotuneMethod::CohenCoon, PIDType::P, 1.0);
        let p_result = p_autotune.calculate_params(ultimate_gain, ultimate_period);
        assert!(p_result.kp > 0.0, "Cohen-Coon P kp should be positive");
        assert_eq!(p_result.ki, 0.0, "Cohen-Coon P ki should be zero");
        assert_eq!(p_result.kd, 0.0, "Cohen-Coon P kd should be zero");
    }
    
    #[test]
    fn test_autotuning_zero_prevention() {
        // Create an autotune instance
        let mut autotune = Autotune::new(AutotuneMethod::ZieglerNichols, PIDType::PID, 1.0);
        
        // This is a simple mock to test our division by zero prevention
        // We're not setting up a full autotuning process
        
        // Simulate a process with no oscillation (which would lead to peak_to_peak = 0)
        for i in 0..100 {
            let time = i as f32 * 0.01;
            let pv = 1.0; // Constant process variable (no oscillation)
            autotune.process(pv, time);
        }
        
        // After processing, it should not have completed due to lack of oscillation
        assert!(!autotune.is_complete(), "Autotune should not complete with no oscillation");
        assert!(autotune.result().is_none(), "Autotune result should be None with no oscillation");
    }
}

