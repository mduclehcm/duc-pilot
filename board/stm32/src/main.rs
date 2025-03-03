#[cfg(feature = "micoair743v2")]
mod micoair743v2;
#[cfg(feature = "micoair743v2")]
pub use micoair743v2::*;

#[cfg(feature = "matekf405te")]
mod matekf405te;
#[cfg(feature = "matekf405te")]
pub use matekf405te::*;

use autopilot::Board;

#[cfg(any(feature = "micoair743v2", feature = "matekf405te"))]
fn main() {
    let board = FlightControllerBoard::new();

    println!("Board: {}", board.get_name());
}

#[cfg(not(any(feature = "micoair743v2", feature = "matekf405te")))]
fn main() {
    compile_error!("No board feature selected");
}
