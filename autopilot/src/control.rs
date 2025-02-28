#[derive(Default, Debug, Clone)]
pub struct ControlInput {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: i16,
    pub flap: i16,
}

#[derive(Default, Debug)]
pub struct ControlOutput {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: i16,
    pub flap: i16,
}

impl ControlOutput {
    pub fn new(roll: i16, pitch: i16, yaw: i16, throttle: i16, flap: i16) -> Self {
        assert!(roll >= -1000 && roll <= 1000);
        assert!(pitch >= -1000 && pitch <= 1000);
        assert!(yaw >= -1000 && yaw <= 1000);
        assert!(throttle >= 0 && throttle <= 1000);
        assert!(flap >= -1000 && flap <= 1000);
        ControlOutput {
            roll,
            pitch,
            yaw,
            throttle,
            flap,
        }
    }

    pub fn throttle_cut_off(&mut self) {
        self.throttle = 0;
    }

    pub fn scale(value: f32) -> i16 {
        (value * 1000.0) as i16
    }
}
