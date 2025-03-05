use crate::{Attitude, Board, ControlInput, ControlOutput, RcInput, RcMapper, PID};

#[derive(Default)]
pub struct Vehicle {
    current_attitude: Attitude,
    desired_attitude: Attitude,
    control_input: ControlInput,
    control_output: ControlOutput,
    armed: bool,

    roll_angle_pid: PID,
    pitch_angle_pid: PID,
    yaw_angle_pid: PID,

    roll_rate_pid: PID,
    pitch_rate_pid: PID,
    yaw_rate_pid: PID,

    rc_mapper: RcMapper,
}

impl Vehicle {
    pub fn new(board: impl Board) -> Self {
        Vehicle {
            rc_mapper: RcMapper::new(),
            ..Default::default()
        }
    }
    // Update the input from the RC receiver
    // This function is when the RC receiver sends a new input
    // Embed: call in rc_input loop
    // SITL: call in flight_axis loop
    pub fn update_rc_input(&mut self, rc_input: RcInput) {
        self.control_input = self.rc_mapper.map(rc_input);
    }

    // Get the mapped RC input
    pub fn get_rc_input(&self) -> &ControlInput {
        &self.control_input
    }

    pub fn set_desired_attitude(&mut self, attitude: Attitude) {
        self.desired_attitude = attitude;
    }

    pub fn get_attitude(&self) -> &Attitude {
        &self.current_attitude
    }
    pub fn get_pwm_output(&self) -> &[u16; 16] {
        unimplemented!()
    }
    pub fn get_mode(&self) -> u8 {
        unimplemented!()
    }
    pub fn set_mode(&mut self, mode: u8) {
        unimplemented!()
    }
    pub fn get_armed(&self) -> bool {
        unimplemented!()
    }
    pub fn set_armed(&mut self, armed: bool) {
        unimplemented!()
    }
    pub fn update(&mut self, dt: f32) {
        // Update roll PID controllers
        let roll_angle_error = self.desired_attitude.roll - self.current_attitude.roll;
        let desired_roll_rate = self.roll_angle_pid.update(roll_angle_error, dt);
        let roll_rate_error = desired_roll_rate - self.current_attitude.roll_rate;
        let roll_output = self.roll_rate_pid.update(roll_rate_error, dt);
        self.control_output.roll = ControlOutput::scale(roll_output);

        // Update pitch PID controllers
        let pitch_angle_error = self.desired_attitude.pitch - self.current_attitude.pitch;
        let desired_pitch_rate = self.pitch_angle_pid.update(pitch_angle_error, dt);
        let pitch_rate_error = desired_pitch_rate - self.current_attitude.pitch_rate;
        let pitch_output = self.pitch_rate_pid.update(pitch_rate_error, dt);
        self.control_output.pitch = ControlOutput::scale(pitch_output);

        // Update yaw PID controllers
        let yaw_angle_error = self.desired_attitude.yaw - self.current_attitude.yaw;
        let desired_yaw_rate = self.yaw_angle_pid.update(yaw_angle_error, dt);
        let yaw_rate_error = desired_yaw_rate - self.current_attitude.yaw_rate;
        let yaw_output = self.yaw_rate_pid.update(yaw_rate_error, dt);
        self.control_output.yaw = ControlOutput::scale(yaw_output);

        // Update throttle
        if self.armed {
            self.control_output.throttle = self.control_input.throttle;
        } else {
            self.control_output.throttle = 0;
        }
    }
}
