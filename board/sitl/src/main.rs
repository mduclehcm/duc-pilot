use sitl::app::SitlApp;

fn main() -> eframe::Result {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0]),
        ..Default::default()
    };
    eframe::run_native(
        "DucPilot SITL",
        native_options,
        Box::new(|cc| Ok(Box::new(SitlApp::new(cc)))),
    )
}
