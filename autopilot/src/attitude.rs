#[derive(Default, Debug, Clone, Copy)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
}
