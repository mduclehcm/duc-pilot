use autopilot::Vehicle;

mod sitl;

fn main() {
    let mut board = sitl::SitlBoard::new();
    let plane = Vehicle::new(board);

    loop {}
}
