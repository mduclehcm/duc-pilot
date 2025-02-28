use crate::ControlInput;

#[derive(Default, Debug, Clone, Copy)]
pub struct RcInput {
    pub channels: [i16; 32],
}

#[derive(Default, Debug)]
pub struct RcMapper {}

impl RcMapper {
    pub fn new() -> Self {
        RcMapper {}
    }

    pub fn map(&self, input: RcInput) -> ControlInput {
        ControlInput {
            roll: input.channels[0],
            pitch: input.channels[1],
            yaw: input.channels[2],
            throttle: input.channels[3],
            flap: input.channels[5],
        }
    }
}
