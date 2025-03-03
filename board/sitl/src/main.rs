use std::time::Duration;

use autopilot::Board;
use sitl::FlightControllerBoard;
mod flight_axis_helper;
mod sitl;

enum State {
    Initializing,
    Connecting,
    Running,
    Stopping,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut last_update_time = std::time::Instant::now();
    let mut minimum_elapsed_duration = Duration::from_millis(1000); // 1 Hz
    let mut state = State::Initializing;
    let mut flight_axis = flight_axis_helper::FlightAxis::new();
    let mut update_cycle_count = 0;
    loop {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_update_time);
        if dt < minimum_elapsed_duration {
            let remaining_duration = minimum_elapsed_duration - dt;
            tokio::time::sleep(remaining_duration).await;
            continue;
        }

        last_update_time = now;

        update_cycle_count += 1;

        match state {
            State::Initializing => {
                println!("Initializing...");
                state = State::Connecting;
            }
            State::Connecting => {
                println!("Connecting...");
                match flight_axis.update().await {
                    Ok(_) => {
                        println!("Connected");
                        state = State::Running;
                    }
                    Err(e) => {
                        println!("Error: {:?}", e);
                    }
                }
            }
            State::Running => {
                minimum_elapsed_duration = Duration::from_millis(200); // 5 Hz
                match flight_axis.update().await {
                    Ok(_) => {
                        println!("State: {:?}", flight_axis.state.input);
                        let output = flight_axis.state.input;
                        flight_axis.set_output(output);
                    }
                    Err(e) => {
                        println!("Error: {:?}", e);
                        state = State::Stopping;
                    }
                }
            }
            State::Stopping => {
                println!("Stopping");
                break;
            }
        }
    }
    Ok(())
}
