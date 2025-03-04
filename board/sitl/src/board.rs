use autopilot::Resources;

pub struct FlightControllerBoard {}

impl FlightControllerBoard {
    pub fn new() -> Self {
        FlightControllerBoard {}
    }
}

impl autopilot::Board for FlightControllerBoard {
    fn get_name(&self) -> &str {
        "SITL"
    }

    fn split_resources(self) -> Resources {
        unimplemented!()
    }
}
