use std::time::Duration;

use autopilot::Board;
use sitl::FlightControllerBoard;
mod flight_axis;
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
    let mut flight_axis_con = Option::None;
    let mut update_cycle_count = 0;
    loop {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_update_time);
        last_update_time = now;

        if dt < minimum_elapsed_duration {
            let remaining_duration = minimum_elapsed_duration - dt;
            tokio::time::sleep(remaining_duration).await;
            continue;
        }

        update_cycle_count += 1;

        match state {
            State::Initializing => {
                println!("Initializing...");
                match flight_axis::FlightAxis::connect().await {
                    Ok(flight_axis) => {
                        flight_axis_con = Some(flight_axis);
                        state = State::Running;
                        println!("Connected to flight axis");
                    }
                    Err(err) => {
                        eprintln!("Failed to connect to flight axis: {:?}", err);
                        state = State::Stopping;
                    }
                }
            }
            State::Connecting => {
                println!("Connecting...");
                minimum_elapsed_duration = Duration::from_secs(1); // 1 Hz
            }
            State::Running => {
                minimum_elapsed_duration = Duration::from_millis(1000 / 100);
                let flight_axis_connection = flight_axis_con.as_mut().unwrap();
                let f_state = flight_axis_connection.get_state().await?;
                println!("State: {:?}", f_state);
                state = State::Stopping;

            }
            State::Stopping => {
                println!("Stopping");
                break;
            }
        }
    }
    Ok(())
}
