use crate::flight_axis::FlightAxis;

pub struct SitlApp {
    state: State,
    flight_axis: FlightAxis,
}

impl SitlApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            state: State::Initializing,
            flight_axis: FlightAxis::new(),
        }
    }

    fn connect_to_flight_axis(&mut self) {
        self.state = State::Running;
        self.flight_axis.start();
    }

    fn stop_sitl(&mut self) {
        self.state = State::Stopping;
        self.flight_axis.stop();
    }
}

impl eframe::App for SitlApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        match self.state {
            State::Initializing => {
                self.connect_to_flight_axis();
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("Initializing DucPilot SITL");
                });
            }
            State::Connecting => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("Connecting to DucPilot SITL");
                });
            }
            State::Running => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ctx.request_repaint_after(std::time::Duration::from_millis(1000 / 50)); // 50 Hz
                    let state = self.flight_axis.state();
                    self.flight_axis.set_output(state.input);
                    // table of flight axis state
                    // roll rate
                    // pitch rate
                    // yaw rate
                    // acceleration x
                    // acceleration y
                    // acceleration z

                    ui.horizontal(|ui| {
                        ui.label("Roll Rate:");
                        ui.label(format!("{:.2} rad/s", state.roll_rate));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Pitch Rate:");
                        ui.label(format!("{:.2} rad/s", state.pitch_rate));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Yaw Rate:");
                        ui.label(format!("{:.2} rad/s", state.yaw_rate));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Acceleration X:");
                        ui.label(format!("{:.2} m/s²", state.acceleration_x));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Acceleration Y:");
                        ui.label(format!("{:.2} m/s²", state.acceleration_y));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Acceleration Z:");
                        ui.label(format!("{:.2} m/s²", state.acceleration_z));
                    });
                });
            }
            State::Stopping => {
                self.stop_sitl();
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("Stopping DucPilot SITL");
                });
            }
            State::Stopped => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("DucPilot SITL Stopped");
                });
            }
        }
    }
}

enum State {
    Initializing,
    Connecting,
    Running,
    Stopping,
    Stopped,
}
