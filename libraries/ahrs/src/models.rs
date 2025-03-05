use crate::{AhrsError, AhrsResult, StateVector};
use nalgebra as na;

/// Trait for motion models used in the AHRS system
pub trait MotionModel {
    /// Predict the next state based on the current state and time step
    fn predict(&self, state: &StateVector, dt: f32) -> AhrsResult<StateVector>;

    /// Get the process noise covariance for the given time step
    fn process_noise(
        &self,
        dt: f32,
    ) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>;

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

        if state.position.iter().any(|v| v.is_nan() || v.is_infinite())
            || state.velocity.iter().any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        let mut new_state = state.clone();

        // Position update: p_new = p + v * dt
        new_state.position += state.velocity * dt;

        // In real systems, gyro_bias is the estimated bias error, so the actual angular rate
        // is the measured rate minus this bias
        let gyro_rate = -state.gyro_bias;

        // Proper quaternion integration for attitude updates
        if gyro_rate.norm() > 1e-6 {
            // Convert angular velocity to angle-axis representation
            let angle = gyro_rate.norm() * dt;
            let axis = gyro_rate.normalize();

            // Create rotation quaternion using the angle-axis representation
            let delta_q =
                na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(axis), angle);

            // Apply the rotation (right-multiplication preserves the reference frame)
            new_state.attitude = state.attitude * delta_q;

            // Normalize to prevent numerical drift
            new_state.attitude =
                na::UnitQuaternion::from_quaternion(new_state.attitude.into_inner().normalize());
        }

        // Validate output
        if new_state
            .position
            .iter()
            .any(|v| v.is_nan() || v.is_infinite())
            || new_state
                .velocity
                .iter()
                .any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        Ok(new_state)
    }

    fn process_noise(
        &self,
        dt: f32,
    ) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q =
            na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();

        // Position noise (grows with time squared for constant velocity)
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

        // Although accelerometer bias isn't directly part of the 12-element state vector,
        // it affects the velocity through the attitude transformation. We need to account for
        // this effect in the velocity components (3-5).
        for i in 3..6 {
            // Add additional velocity uncertainty due to accelerometer bias
            q[(i, i)] += 0.5 * self.accel_bias_noise * dt * dt;
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

        if state.position.iter().any(|v| v.is_nan() || v.is_infinite())
            || state.velocity.iter().any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        let mut new_state = state.clone();

        // The acceleration in this model should represent the true acceleration that affects
        // the vehicle's motion in the NED frame.

        // Step 1: Get the specific force measurements in body frame
        // In a complete implementation, this would come from accelerometer measurements
        // Here we use the negative of the bias as a simple approximation
        let specific_force_body = -state.accel_bias;

        // Step 2: Transform from body frame to navigation (NED) frame using the current attitude
        let specific_force_ned = state.attitude * specific_force_body;

        // Step 3: Account for gravity to get true acceleration
        // In NED frame, gravity points down (+z), so we add gravity to compute true acceleration
        let gravity = na::Vector3::new(0.0, 0.0, 9.81); // Standard gravity in m/s²
        let acceleration = specific_force_ned + gravity;

        // Safety check - limit acceleration to reasonable values to prevent numerical issues
        let acceleration = na::Vector3::new(
            acceleration.x.clamp(-100.0, 100.0),
            acceleration.y.clamp(-100.0, 100.0),
            acceleration.z.clamp(-100.0, 100.0),
        );

        // Position update with constant acceleration
        // p_new = p + v*dt + 0.5*a*dt^2
        new_state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;

        // Velocity update with constant acceleration
        // v_new = v + a*dt
        new_state.velocity += acceleration * dt;

        // Update attitude using proper quaternion integration with gyro measurements
        let gyro_rate = -state.gyro_bias; // Correct sign for angular rate

        if gyro_rate.norm() > 1e-6 {
            // Convert angular velocity to angle-axis representation
            let angle = gyro_rate.norm() * dt;
            let axis = gyro_rate.normalize();

            // Create rotation quaternion using the angle-axis representation
            let delta_q =
                na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(axis), angle);

            // Apply the rotation (right-multiplication preserves the reference frame)
            new_state.attitude = state.attitude * delta_q;

            // Normalize to prevent numerical drift
            new_state.attitude =
                na::UnitQuaternion::from_quaternion(new_state.attitude.into_inner().normalize());
        }

        // Validate output
        if new_state
            .position
            .iter()
            .any(|v| v.is_nan() || v.is_infinite())
            || new_state
                .velocity
                .iter()
                .any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        Ok(new_state)
    }

    fn process_noise(
        &self,
        dt: f32,
    ) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q =
            na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();

        // Position noise (grows with time cubed for constant acceleration model)
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt * dt;
        }

        // Velocity noise (grows with time squared)
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt * dt;
        }

        // Acceleration noise (grows with time)
        // This affects the velocity part of the state
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

        // Accelerometer bias impacts both position and velocity
        // For velocity, the effect is proportional to dt
        for i in 3..6 {
            q[(i, i)] += self.accel_bias_noise * dt;
        }

        // For position, the effect is proportional to dt^2
        for i in 0..3 {
            q[(i, i)] += 0.5 * self.accel_bias_noise * dt * dt;
        }

        // Cross-correlation between position and velocity due to accelerometer bias
        // This captures how errors in acceleration affect both position and velocity
        for i in 0..3 {
            // Position-velocity correlation
            q[(i, i + 3)] = 0.5 * self.accel_bias_noise * dt * dt;
            q[(i + 3, i)] = 0.5 * self.accel_bias_noise * dt * dt;
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
    pub fn new(pos_noise: f32, vel_noise: f32, turn_rate_noise: f32, att_noise: f32) -> Self {
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

        if state.position.iter().any(|v| v.is_nan() || v.is_infinite())
            || state.velocity.iter().any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        let mut new_state = state.clone();

        // Extract turn rate from gyro measurements
        // In a proper system, this would come from calibrated gyro measurements
        // Here we use the negative of the bias to simulate the actual measurements
        let gyro_rate = -state.gyro_bias;
        let turn_rate = gyro_rate.z; // Z-axis rotation for coordinated turns

        // First update the attitude using proper quaternion integration
        if gyro_rate.norm() > 1e-6 {
            // Convert angular velocity to angle-axis representation
            let angle = gyro_rate.norm() * dt;
            let axis = gyro_rate.normalize();

            // Create rotation quaternion using the angle-axis representation
            let delta_q =
                na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(axis), angle);

            // Apply the rotation
            new_state.attitude = state.attitude * delta_q;

            // Normalize to prevent numerical drift
            new_state.attitude =
                na::UnitQuaternion::from_quaternion(new_state.attitude.into_inner().normalize());
        }

        // Check for very small turn rate to avoid division by zero
        if turn_rate.abs() < 1e-6 {
            // If turn rate is very small, use constant velocity model
            new_state.position += state.velocity * dt;
        } else {
            // Coordinated turn equations for horizontal motion
            let speed_horizontal = na::Vector2::new(state.velocity.x, state.velocity.y).norm();

            // Only apply coordinated turn model if we have meaningful velocity
            if speed_horizontal > 1e-3 {
                let heading = f32::atan2(state.velocity.y, state.velocity.x);

                // New heading after turn
                let new_heading = heading + turn_rate * dt;

                // New velocity components (preserve speed but change direction)
                new_state.velocity.x = speed_horizontal * f32::cos(new_heading);
                new_state.velocity.y = speed_horizontal * f32::sin(new_heading);

                // Position update using trapezoidal integration
                let avg_vel_x = 0.5 * (state.velocity.x + new_state.velocity.x);
                let avg_vel_y = 0.5 * (state.velocity.y + new_state.velocity.y);

                new_state.position.x += avg_vel_x * dt;
                new_state.position.y += avg_vel_y * dt;

                // Vertical motion updated using constant velocity model
                new_state.position.z += state.velocity.z * dt;
            } else {
                // If horizontal velocity is too small, use constant velocity model
                new_state.position += state.velocity * dt;
            }
        }

        // Validate output
        if new_state
            .position
            .iter()
            .any(|v| v.is_nan() || v.is_infinite())
            || new_state
                .velocity
                .iter()
                .any(|v| v.is_nan() || v.is_infinite())
        {
            return Err(AhrsError::InvalidState);
        }

        Ok(new_state)
    }

    fn process_noise(
        &self,
        dt: f32,
    ) -> na::Matrix<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>> {
        let mut q =
            na::Matrix::<f32, na::Const<12>, na::Const<12>, na::ArrayStorage<f32, 12, 12>>::zeros();

        // Position noise - in coordinated turns, the uncertainty grows more rapidly
        // due to the complex motion model
        for i in 0..3 {
            q[(i, i)] = self.pos_noise * dt * dt;
        }

        // Additional position uncertainty due to turn dynamics
        q[(0, 0)] += 0.25 * self.turn_rate_noise * dt * dt * dt;
        q[(1, 1)] += 0.25 * self.turn_rate_noise * dt * dt * dt;

        // Velocity noise
        for i in 3..6 {
            q[(i, i)] = self.vel_noise * dt;
        }

        // Additional velocity uncertainty due to turn dynamics
        q[(3, 3)] += 0.5 * self.turn_rate_noise * dt * dt;
        q[(4, 4)] += 0.5 * self.turn_rate_noise * dt * dt;

        // Cross-correlation between position and velocity
        // This is more significant in coordinated turns
        q[(0, 3)] = 0.25 * self.turn_rate_noise * dt * dt;
        q[(3, 0)] = 0.25 * self.turn_rate_noise * dt * dt;
        q[(1, 4)] = 0.25 * self.turn_rate_noise * dt * dt;
        q[(4, 1)] = 0.25 * self.turn_rate_noise * dt * dt;

        // Attitude noise - more significant in the yaw component for coordinated turns
        for i in 6..9 {
            q[(i, i)] = self.att_noise * dt;
        }

        // Add extra noise for z-axis rotation (turn rate)
        q[(8, 8)] += self.turn_rate_noise * dt;

        // Cross-correlation between turn rate and horizontal velocity components
        q[(3, 8)] = 0.1 * self.turn_rate_noise * dt;
        q[(8, 3)] = 0.1 * self.turn_rate_noise * dt;
        q[(4, 8)] = 0.1 * self.turn_rate_noise * dt;
        q[(8, 4)] = 0.1 * self.turn_rate_noise * dt;

        // Gyro bias noise - using smaller values for x/y axes than z-axis
        q[(9, 9)] = 1e-5 * dt; // x-axis gyro bias
        q[(10, 10)] = 1e-5 * dt; // y-axis gyro bias
        q[(11, 11)] = 3e-5 * dt; // z-axis gyro bias (higher for turn coordination)

        q
    }

    fn name(&self) -> &'static str {
        "Coordinated Turn Model"
    }
}
