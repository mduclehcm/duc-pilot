pub struct SitlBoard {}

impl SitlBoard {
    pub fn new() -> Self {
        SitlBoard {}
    }
}

impl autopilot::Board for SitlBoard {
    fn name(&self) -> &str {
        "SITL"
    }
}
