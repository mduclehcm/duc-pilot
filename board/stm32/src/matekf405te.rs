#[derive(Default)]
pub struct FlightControllerBoard {}

impl FlightControllerBoard {
    pub fn new() -> Self {
        FlightControllerBoard {}
    }
}

impl autopilot::Board for FlightControllerBoard {
    fn get_name(&self) -> &str {
        "MatekF405-TE"
    }

    fn split_resources(self) -> autopilot::Resources {
        todo!()
    }
}
