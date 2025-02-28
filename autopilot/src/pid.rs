#[derive(Default, Debug)]
pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    ff: f32,
    integral: f32,
    last_error: f32,
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32, ff: f32) -> Self {
        PID {
            kp,
            ki,
            kd,
            ff,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += error * dt;
        let derivative = (error - self.last_error) / dt;
        self.last_error = error;

        self.kp * error + self.ki * self.integral + self.kd * derivative + self.ff * error
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32, ff: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.ff = ff;
    }

    pub fn get_gains(&self) -> (f32, f32, f32, f32) {
        (self.kp, self.ki, self.kd, self.ff)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }
}
