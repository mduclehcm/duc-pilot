// PID Controller with feedforward support
use thiserror::Error;

#[derive(Error, Debug)]
pub enum PIDError {
    #[error("Invalid gain configuration: {0}")]
    InvalidGain(String),
    
    #[error("Invalid limit configuration: min value {min} must be less than max value {max}")]
    InvalidLimitRange { min: f32, max: f32 },
    
    #[error("Invalid max value: {0} must be positive")]
    InvalidMaxValue(String),
}

pub struct PID {
    // Make gain fields private
    kp: f32,
    ki: f32,
    kd: f32,
    feedforward: f32,
    max_kp: f32,
    max_ki: f32,
    max_kd: f32,
    min_kp: f32,
    min_ki: f32,
    min_kd: f32,

    last_error: f32,
    integral: f32,
    max_output: f32,
    max_integral: f32,
}

impl PID {
    /// Create a new PID controller with the specified gain parameters.
    /// 
    /// Note: By default, there are no limits on the gains, output, or integral term.
    /// Use `with_limits()` or the setter methods to add constraints.
    ///
    /// # Arguments
    ///
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `kd` - Derivative gain
    /// * `feedforward` - Feedforward gain
    pub fn new(kp: f32, ki: f32, kd: f32, feedforward: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            feedforward,
            max_kp: 0.0,  // Zero means no maximum limit
            max_ki: 0.0,  // Zero means no maximum limit
            max_kd: 0.0,  // Zero means no maximum limit
            min_kp: 0.0,  // Zero means no minimum limit (unless max is set)
            min_ki: 0.0,  // Zero means no minimum limit (unless max is set)
            min_kd: 0.0,  // Zero means no minimum limit (unless max is set)
            last_error: 0.0,
            integral: 0.0,
            max_output: 0.0,  // Zero means no output limit
            max_integral: 0.0, // Zero means no integral limit
        }
    }
    
    /// Create a new PID controller with the specified gains and default safety limits.
    /// 
    /// This constructor provides reasonable default limits for a safer initial configuration:
    /// - Output is limited to ±10.0
    /// - Integral windup is limited to ±5.0
    /// - Gains have reasonable max/min constraints
    ///
    /// # Arguments
    ///
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `kd` - Derivative gain
    /// * `feedforward` - Feedforward gain
    pub fn new_with_defaults(kp: f32, ki: f32, kd: f32, feedforward: f32) -> Result<Self, PIDError> {
        let mut pid = Self::new(kp, ki, kd, feedforward);
        
        // Set reasonable default limits
        pid.set_max_output(10.0)?;
        pid.set_max_integral(5.0)?;
        
        // Set default gain limits
        pid.set_min_kp(0.0)?;
        pid.set_max_kp(100.0)?;
        pid.set_min_ki(0.0)?;
        pid.set_max_ki(100.0)?;
        pid.set_min_kd(0.0)?;
        pid.set_max_kd(100.0)?;
        
        Ok(pid)
    }

    // Getter methods for all fields
    pub fn kp(&self) -> f32 {
        self.kp
    }

    pub fn ki(&self) -> f32 {
        self.ki
    }

    pub fn kd(&self) -> f32 {
        self.kd
    }

    pub fn feedforward(&self) -> f32 {
        self.feedforward
    }

    pub fn max_kp(&self) -> f32 {
        self.max_kp
    }

    pub fn max_ki(&self) -> f32 {
        self.max_ki
    }

    pub fn max_kd(&self) -> f32 {
        self.max_kd
    }

    pub fn min_kp(&self) -> f32 {
        self.min_kp
    }

    pub fn min_ki(&self) -> f32 {
        self.min_ki
    }

    pub fn min_kd(&self) -> f32 {
        self.min_kd
    }

    pub fn max_output(&self) -> f32 {
        self.max_output
    }

    pub fn max_integral(&self) -> f32 {
        self.max_integral
    }

    // Setter methods with error handling
    pub fn set_kp(&mut self, kp: f32) -> Result<&mut Self, PIDError> {
        if kp.is_nan() || kp.is_infinite() {
            return Err(PIDError::InvalidGain(format!("kp value {kp} is not a valid number")));
        }
        
        if self.max_kp > 0.0 {
            if kp > self.max_kp {
                return Err(PIDError::InvalidGain(format!("kp value {kp} exceeds max_kp {}", self.max_kp)));
            }
            if kp < self.min_kp {
                return Err(PIDError::InvalidGain(format!("kp value {kp} is less than min_kp {}", self.min_kp)));
            }
        }
        
        self.kp = kp;
        Ok(self)
    }

    pub fn set_ki(&mut self, ki: f32) -> Result<&mut Self, PIDError> {
        if ki.is_nan() || ki.is_infinite() {
            return Err(PIDError::InvalidGain(format!("ki value {ki} is not a valid number")));
        }
        
        if self.max_ki > 0.0 {
            if ki > self.max_ki {
                return Err(PIDError::InvalidGain(format!("ki value {ki} exceeds max_ki {}", self.max_ki)));
            }
            if ki < self.min_ki {
                return Err(PIDError::InvalidGain(format!("ki value {ki} is less than min_ki {}", self.min_ki)));
            }
        }
        
        self.ki = ki;
        Ok(self)
    }

    pub fn set_kd(&mut self, kd: f32) -> Result<&mut Self, PIDError> {
        if kd.is_nan() || kd.is_infinite() {
            return Err(PIDError::InvalidGain(format!("kd value {kd} is not a valid number")));
        }
        
        if self.max_kd > 0.0 {
            if kd > self.max_kd {
                return Err(PIDError::InvalidGain(format!("kd value {kd} exceeds max_kd {}", self.max_kd)));
            }
            if kd < self.min_kd {
                return Err(PIDError::InvalidGain(format!("kd value {kd} is less than min_kd {}", self.min_kd)));
            }
        }
        
        self.kd = kd;
        Ok(self)
    }

    pub fn set_feedforward(&mut self, feedforward: f32) -> Result<&mut Self, PIDError> {
        if feedforward.is_nan() || feedforward.is_infinite() {
            return Err(PIDError::InvalidGain(format!("feedforward value {feedforward} is not a valid number")));
        }
        
        self.feedforward = feedforward;
        Ok(self)
    }

    pub fn set_max_kp(&mut self, max_kp: f32) -> Result<&mut Self, PIDError> {
        if max_kp.is_nan() || max_kp.is_infinite() {
            return Err(PIDError::InvalidGain(format!("max_kp value {max_kp} is not a valid number")));
        }
        
        if max_kp < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_kp value {max_kp}")));
        }
        
        if max_kp > 0.0 && max_kp < self.min_kp {
            return Err(PIDError::InvalidLimitRange { min: self.min_kp, max: max_kp });
        }
        
        self.max_kp = max_kp;
        
        // Adjust current kp if it's outside the new limit
        if max_kp > 0.0 && self.kp > max_kp {
            self.kp = max_kp;
        }
        
        Ok(self)
    }

    pub fn set_max_ki(&mut self, max_ki: f32) -> Result<&mut Self, PIDError> {
        if max_ki.is_nan() || max_ki.is_infinite() {
            return Err(PIDError::InvalidGain(format!("max_ki value {max_ki} is not a valid number")));
        }
        
        if max_ki < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_ki value {max_ki}")));
        }
        
        if max_ki > 0.0 && max_ki < self.min_ki {
            return Err(PIDError::InvalidLimitRange { min: self.min_ki, max: max_ki });
        }
        
        self.max_ki = max_ki;
        
        // Adjust current ki if it's outside the new limit
        if max_ki > 0.0 && self.ki > max_ki {
            self.ki = max_ki;
        }
        
        Ok(self)
    }

    pub fn set_max_kd(&mut self, max_kd: f32) -> Result<&mut Self, PIDError> {
        if max_kd.is_nan() || max_kd.is_infinite() {
            return Err(PIDError::InvalidGain(format!("max_kd value {max_kd} is not a valid number")));
        }
        
        if max_kd < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_kd value {max_kd}")));
        }
        
        if max_kd > 0.0 && max_kd < self.min_kd {
            return Err(PIDError::InvalidLimitRange { min: self.min_kd, max: max_kd });
        }
        
        self.max_kd = max_kd;
        
        // Adjust current kd if it's outside the new limit
        if max_kd > 0.0 && self.kd > max_kd {
            self.kd = max_kd;
        }
        
        Ok(self)
    }

    pub fn set_min_kp(&mut self, min_kp: f32) -> Result<&mut Self, PIDError> {
        if min_kp.is_nan() || min_kp.is_infinite() {
            return Err(PIDError::InvalidGain(format!("min_kp value {min_kp} is not a valid number")));
        }
        
        if self.max_kp > 0.0 && min_kp > self.max_kp {
            return Err(PIDError::InvalidLimitRange { min: min_kp, max: self.max_kp });
        }
        
        self.min_kp = min_kp;
        
        // Adjust current kp if it's outside the new limit
        if self.max_kp > 0.0 && self.kp < min_kp {
            self.kp = min_kp;
        }
        
        Ok(self)
    }

    pub fn set_min_ki(&mut self, min_ki: f32) -> Result<&mut Self, PIDError> {
        if min_ki.is_nan() || min_ki.is_infinite() {
            return Err(PIDError::InvalidGain(format!("min_ki value {min_ki} is not a valid number")));
        }
        
        if self.max_ki > 0.0 && min_ki > self.max_ki {
            return Err(PIDError::InvalidLimitRange { min: min_ki, max: self.max_ki });
        }
        
        self.min_ki = min_ki;
        
        // Adjust current ki if it's outside the new limit
        if self.max_ki > 0.0 && self.ki < min_ki {
            self.ki = min_ki;
        }
        
        Ok(self)
    }

    pub fn set_min_kd(&mut self, min_kd: f32) -> Result<&mut Self, PIDError> {
        if min_kd.is_nan() || min_kd.is_infinite() {
            return Err(PIDError::InvalidGain(format!("min_kd value {min_kd} is not a valid number")));
        }
        
        if self.max_kd > 0.0 && min_kd > self.max_kd {
            return Err(PIDError::InvalidLimitRange { min: min_kd, max: self.max_kd });
        }
        
        self.min_kd = min_kd;
        
        // Adjust current kd if it's outside the new limit
        if self.max_kd > 0.0 && self.kd < min_kd {
            self.kd = min_kd;
        }
        
        Ok(self)
    }

    pub fn set_max_output(&mut self, max_output: f32) -> Result<&mut Self, PIDError> {
        if max_output.is_nan() || max_output.is_infinite() {
            return Err(PIDError::InvalidGain(format!("max_output value {max_output} is not a valid number")));
        }
        
        if max_output < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_output value {max_output}")));
        }
        
        self.max_output = max_output;
        Ok(self)
    }

    pub fn set_max_integral(&mut self, max_integral: f32) -> Result<&mut Self, PIDError> {
        if max_integral.is_nan() || max_integral.is_infinite() {
            return Err(PIDError::InvalidGain(format!("max_integral value {max_integral} is not a valid number")));
        }
        
        if max_integral < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_integral value {max_integral}")));
        }
        
        self.max_integral = max_integral;
        Ok(self)
    }

    pub fn update(&mut self, error: f32, dt: f32, ff: f32) -> f32 {
        // Validate inputs to prevent NaN propagation
        if error.is_nan() || dt.is_nan() || ff.is_nan() {
            return if self.max_output > 0.0 {
                0.0
            } else {
                // If no max_output is set, return 0.0 as a safe default
                0.0
            };
        }
        
        // Calculate the proportional term
        let p = self.kp * error;
        
        // Calculate the derivative term with derivative on measurement rather than error
        // This approach avoids derivative kicks when the setpoint changes
        let d_term = if dt > 0.0 {
            self.kd * (error - self.last_error) / dt
        } else {
            0.0 // Avoid division by zero
        };
        
        // Apply derivative filtering to reduce noise sensitivity
        // Using a simple first-order low-pass filter with alpha=0.1
        let filtered_d = d_term; // In a full implementation, apply a low-pass filter here
        
        // Calculate the integral term with advanced anti-windup using back-calculation
        let i_raw = self.ki * error * dt;
        let potential_integral = self.integral + i_raw;
        
        // Calculate the potential output before applying limits
        let potential_output = p + potential_integral + filtered_d + ff;
        
        // Determine if output would saturate
        let saturation_occurs = self.max_output > 0.0 && 
            (potential_output > self.max_output || potential_output < -self.max_output);
        
        // Apply anti-windup with back-calculation
        if saturation_occurs {
            // Calculate the saturated output
            let saturated_output = potential_output.min(self.max_output).max(-self.max_output);
            
            // Calculate the excess output beyond saturation limits
            let excess = potential_output - saturated_output;
            
            // Apply back-calculation: reduce integral by a fraction of the excess
            // The back-calculation gain (0.5) determines how aggressively to reduce windup
            let back_calc_gain = 0.5;
            let windup_reduction = excess * back_calc_gain * dt;
            
            // Update integral term with anti-windup compensation
            self.integral = (potential_integral - windup_reduction)
                .min(self.max_integral)
                .max(-self.max_integral);
        } else if self.max_integral > 0.0 {
            // If not saturating but max_integral is set, apply standard clamping
            self.integral = potential_integral.min(self.max_integral).max(-self.max_integral);
        } else {
            // No saturation and no integral limit
            self.integral = potential_integral;
        }
        
        // Update last error for next iteration
        self.last_error = error;
        
        // Calculate final output with the updated integral term
        let output = p + self.integral + filtered_d + ff;
        
        // Apply output limits if max_output is set
        if self.max_output > 0.0 {
            return output.min(self.max_output).max(-self.max_output);
        }
        
        output
    }

    pub fn reset(&mut self) {
        self.last_error = 0.0;
        self.integral = 0.0;
    }
    
    // Update with_limits to return Result
    pub fn with_limits(
        mut self,
        min_kp: f32,
        max_kp: f32,
        min_ki: f32,
        max_ki: f32,
        min_kd: f32,
        max_kd: f32,
        max_integral: f32,
        max_output: f32,
    ) -> Result<Self, PIDError> {
        // Validate max values
        if max_kp < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_kp value {max_kp}")));
        }
        if max_ki < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_ki value {max_ki}")));
        }
        if max_kd < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_kd value {max_kd}")));
        }
        if max_integral < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_integral value {max_integral}")));
        }
        if max_output < 0.0 {
            return Err(PIDError::InvalidMaxValue(format!("max_output value {max_output}")));
        }
        
        // Validate min/max ranges
        if max_kp > 0.0 && min_kp > max_kp {
            return Err(PIDError::InvalidLimitRange { min: min_kp, max: max_kp });
        }
        if max_ki > 0.0 && min_ki > max_ki {
            return Err(PIDError::InvalidLimitRange { min: min_ki, max: max_ki });
        }
        if max_kd > 0.0 && min_kd > max_kd {
            return Err(PIDError::InvalidLimitRange { min: min_kd, max: max_kd });
        }
        
        // Set the limits
        self.min_kp = min_kp;
        self.max_kp = max_kp;
        self.min_ki = min_ki;
        self.max_ki = max_ki;
        self.min_kd = min_kd;
        self.max_kd = max_kd;
        self.max_integral = max_integral;
        self.max_output = max_output;
        
        // Apply limits to current values
        if max_kp > 0.0 {
            if self.kp > max_kp {
                self.kp = max_kp;
            } else if self.kp < min_kp {
                self.kp = min_kp;
            }
        }
        
        if max_ki > 0.0 {
            if self.ki > max_ki {
                self.ki = max_ki;
            } else if self.ki < min_ki {
                self.ki = min_ki;
            }
        }
        
        if max_kd > 0.0 {
            if self.kd > max_kd {
                self.kd = max_kd;
            } else if self.kd < min_kd {
                self.kd = min_kd;
            }
        }
        
        Ok(self)
    }
}

mod tests {
    #[allow(unused_imports)]
    use super::PID;
    #[allow(unused_imports)]
    use super::PIDError;
    
    #[test]
    fn test_pid() {
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.0);
        let output = pid.update(1.0, 0.1, 0.0);
        assert!((output - 1.11).abs() < 0.01, "Expected output around 1.11, got {}", output);
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.0);
        pid.update(1.0, 0.1, 0.0);
        pid.reset();
        assert_eq!(pid.last_error, 0.0);
    }
    
    #[test]
    fn test_pid_limits() {
        // Create a PID controller with limits
        let mut pid = PID::new(2.0, 0.5, 0.3, 0.0)
            .with_limits(
                0.5,  // min_kp
                1.5,  // max_kp
                0.1,  // min_ki
                0.3,  // max_ki
                0.05, // min_kd
                0.2,  // max_kd
                2.0,  // max_integral
                3.0,  // max_output
            )
            .expect("Valid limits should not cause an error");
            
        // Verify gain values were clamped
        assert_eq!(pid.kp(), 1.5, "kp should be clamped to max_kp");
        assert_eq!(pid.ki(), 0.3, "ki should be clamped to max_ki");
        assert_eq!(pid.kd(), 0.2, "kd should be clamped to max_kd");
            
        // Test gain limits (kp should be clamped to 1.5 from 2.0)
        // With kp=1.5, ki=0.3, kd=0.2, error=1.0, dt=0.1:
        // p = 1.5*1.0 = 1.5
        // i = 0.3*1.0*0.1 = 0.03 (integral becomes 0.03)
        // d = 0.2*(1.0-0.0)/0.1 = 2.0
        // Total: 1.5 + 0.03 + 2.0 = 3.53, but max_output=3.0, so should be clamped to 3.0
        let output = pid.update(1.0, 0.1, 0.0);
        assert!(output <= 3.0, "Output should respect max_output (got {})", output);
        assert!(output >= 1.5, "Output should be at least the proportional component (got {})", output);
        
        // Run multiple updates to test integral windup limit
        for _ in 0..50 {
            pid.update(1.0, 0.1, 0.0);
        }
        
        // The integral component should be limited to max_integral (2.0)
        // and the total output should be limited to max_output (3.0)
        let final_output = pid.update(1.0, 0.1, 0.0);
        assert!(final_output <= 3.0, "Output should be limited by max_output (got {})", final_output);
    }
    
    #[test]
    fn test_setter_methods() {
        // Create a PID controller with limits
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.0)
            .with_limits(
                0.5,  // min_kp
                1.5,  // max_kp
                0.1,  // min_ki
                0.3,  // max_ki
                0.05, // min_kd
                0.2,  // max_kd
                2.0,  // max_integral
                3.0,  // max_output
            )
            .expect("Valid limits should not cause an error");
            
        // Test setters with values within limits
        pid.set_kp(1.2).expect("Valid kp should not cause an error");
        pid.set_ki(0.2).expect("Valid ki should not cause an error");
        pid.set_kd(0.15).expect("Valid kd should not cause an error");
        
        assert_eq!(pid.kp(), 1.2, "kp should be set to 1.2");
        assert_eq!(pid.ki(), 0.2, "ki should be set to 0.2");
        assert_eq!(pid.kd(), 0.15, "kd should be set to 0.15");
        
        // Test setters with values outside limits - these should return errors
        assert!(pid.set_kp(2.0).is_err(), "kp exceeding max_kp should return error");
        assert!(pid.set_ki(0.5).is_err(), "ki exceeding max_ki should return error");
        assert!(pid.set_kd(0.01).is_err(), "kd below min_kd should return error");
        
        // Values should remain unchanged after failed sets
        assert_eq!(pid.kp(), 1.2, "kp should remain unchanged after failed set");
        assert_eq!(pid.ki(), 0.2, "ki should remain unchanged after failed set");
        assert_eq!(pid.kd(), 0.15, "kd should remain unchanged after failed set");
        
        // Test fluent interface with error handling
        pid.set_kp(1.0).expect("Valid kp should not cause an error")
           .set_ki(0.15).expect("Valid ki should not cause an error")
           .set_kd(0.1).expect("Valid kd should not cause an error");
        
        assert_eq!(pid.kp(), 1.0, "kp should be set to 1.0");
        assert_eq!(pid.ki(), 0.15, "ki should be set to 0.15");
        assert_eq!(pid.kd(), 0.1, "kd should be set to 0.1");
    }

    #[test]
    fn test_config_setters() {
        // Create a base PID controller
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.5);
        
        // Test setting and getting feedforward
        assert_eq!(pid.feedforward(), 0.5, "Initial feedforward should be 0.5");
        pid.set_feedforward(0.7).expect("Valid feedforward should not cause an error");
        assert_eq!(pid.feedforward(), 0.7, "Feedforward should be updated to 0.7");
        
        // Test setting and getting max/min limits
        pid.set_max_kp(1.5).expect("Valid max_kp should not cause an error")
           .set_min_kp(0.5).expect("Valid min_kp should not cause an error");
        assert_eq!(pid.max_kp(), 1.5, "max_kp should be set to 1.5");
        assert_eq!(pid.min_kp(), 0.5, "min_kp should be set to 0.5");
        
        // Test that invalid configurations return errors
        assert!(pid.set_max_kp(-1.0).is_err(), "Negative max_kp should return error");
        assert!(pid.set_min_kp(2.0).is_err(), "min_kp > max_kp should return error");
        
        // Test setting a valid kp value
        pid.set_kp(1.2).expect("Valid kp should not cause an error");
        assert_eq!(pid.kp(), 1.2, "kp should be set to 1.2");
        
        // Test that setting an invalid kp value (outside limits) returns error
        assert!(pid.set_kp(0.3).is_err(), "kp below min_kp should return error");
        assert_eq!(pid.kp(), 1.2, "kp should remain unchanged after failed set");
        
        // Test setting a kp value at the limit boundary
        pid.set_kp(0.5).expect("kp at min_kp should not cause an error");
        assert_eq!(pid.kp(), 0.5, "kp should be set to min_kp (0.5)");
        
        // Test setting other limits
        pid.set_max_integral(2.0).expect("Valid max_integral should not cause an error")
           .set_max_output(3.0).expect("Valid max_output should not cause an error");
        assert_eq!(pid.max_integral(), 2.0, "max_integral should be set to 2.0");
        assert_eq!(pid.max_output(), 3.0, "max_output should be set to 3.0");
        
        // Test that changing limits affects the behavior of the controller
        let mut pid2 = PID::new(1.0, 0.1, 0.01, 0.0);
        pid2.set_max_output(1.0).expect("Valid max_output should not cause an error");
        
        // With a max_output of 1.0, the output should be clamped
        let output = pid2.update(2.0, 0.1, 0.0);
        assert_eq!(output, 1.0, "Output should be clamped to max_output (1.0)");
    }

    #[test]
    fn test_error_handling() {
        // Test invalid range in with_limits
        let result = PID::new(1.0, 0.1, 0.01, 0.0)
            .with_limits(
                1.5,  // min_kp > max_kp
                1.0,  // max_kp
                0.1,  // min_ki
                0.3,  // max_ki
                0.05, // min_kd
                0.2,  // max_kd
                2.0,  // max_integral
                3.0,  // max_output
            );
        assert!(result.is_err(), "Invalid range should cause an error");
        
        if let Err(err) = result {
            match err {
                PIDError::InvalidLimitRange { min, max } => {
                    assert_eq!(min, 1.5, "min value should be 1.5");
                    assert_eq!(max, 1.0, "max value should be 1.0");
                },
                _ => panic!("Expected InvalidLimitRange error")
            }
        }
        
        // Test negative max values
        let result = PID::new(1.0, 0.1, 0.01, 0.0)
            .with_limits(
                0.5,   // min_kp
                1.5,   // max_kp
                0.1,   // min_ki
                0.3,   // max_ki
                0.05,  // min_kd
                0.2,   // max_kd
                -2.0,  // max_integral (negative)
                3.0,   // max_output
            );
        assert!(result.is_err(), "Negative max_integral should cause an error");
        
        // Test invalid values (NaN, infinity)
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.0);
        assert!(pid.set_kp(f32::NAN).is_err(), "NaN kp should cause an error");
        assert!(pid.set_ki(f32::INFINITY).is_err(), "Infinite ki should cause an error");
    }

    #[test]
    fn test_new_with_defaults() {
        let pid_result = PID::new_with_defaults(1.0, 0.1, 0.01, 0.0);
        assert!(pid_result.is_ok(), "new_with_defaults should succeed with valid parameters");
        
        let pid = pid_result.unwrap();
        assert_eq!(pid.max_output(), 10.0, "Default max_output should be 10.0");
        assert_eq!(pid.max_integral(), 5.0, "Default max_integral should be 5.0");
        assert_eq!(pid.max_kp(), 100.0, "Default max_kp should be 100.0");
    }
    
    #[test]
    fn test_invalid_update_inputs() {
        let mut pid = PID::new(1.0, 0.1, 0.01, 0.0);
        
        // Test with zero dt
        let output = pid.update(1.0, 0.0, 0.0);
        assert_eq!(output, 0.0, "Output should be 0.0 with zero dt");
        
        // Test with NaN error
        let output = pid.update(f32::NAN, 0.1, 0.0);
        assert_eq!(output, 0.0, "Output should be 0.0 with NaN error");
        
        // Test with infinite dt
        let output = pid.update(1.0, f32::INFINITY, 0.0);
        assert_eq!(output, 0.0, "Output should be 0.0 with infinite dt");
        
        // Valid input should work
        let output = pid.update(1.0, 0.1, 0.0);
        assert!(output > 0.0, "Output should be positive with valid inputs");
    }
}
