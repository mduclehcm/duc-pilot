# Flight Controller Firmware in Rust

## Overview
This project is a **flight controller firmware** written in **Rust**, designed for learning both **Rust programming** and **flight control algorithms** simultaneously. While not intended to be a fully complete flight control software like ArduPilot or PX4, it aims to implement essential features needed for stable flight.

## Features
- **Sensor Integration**: Read IMU (gyroscope & accelerometer) and barometer data.
- **State Estimation**: Sensor fusion for estimating aircraft attitude and velocity.
- **PID Control**: Implementing PID-based stabilization.
- **Actuator Output**: Controlling servos and motors via PWM.
- **Radio Input**: Parsing commands from an RC transmitter.
- **Failsafe Mechanism**: Basic emergency handling in case of signal loss or sensor failure.

## Why Rust?
Rust is chosen for its:
- **Memory safety**: Eliminates common memory errors like buffer overflows.
- **Concurrency**: Safe handling of multi-threading and real-time operations.
- **Embedded Support**: Compatible with microcontrollers used in flight controllers.

## Hardware Support
- **MCUs**: STM32F4 (targeting for Cortex-M4 architecture) or similar.
- **Sensors**: MPU6050, BMI088, MS5611, or any SPI/I2C-based IMU.
- **Actuators**: Standard PWM ESCs and servos.
- **Communication**: Serial/UART, I2C, SPI.

## Getting Started
### Prerequisites
- **Rust toolchain**: Install using [rustup](https://rustup.rs/)
- **Embedded Rust setup**: Install `cargo-binutils`, `probe-rs`, `flip-link`, etc.
- **Hardware debugger**: ST-Link or J-Link for flashing and debugging.

### Building and Flashing
1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/flight-controller-rust.git
   cd flight-controller-rust
   ```
2. Build the firmware:
   ```sh
   cargo build --release
   ```
3. Flash the firmware to the target:
   ```sh
   cargo flash --chip STM32F405RG --release
   ```

## Roadmap
- [x] Basic IMU integration
- [ ] Sensor fusion using complementary/Kalman filter
- [ ] PID controller for stabilization
- [ ] PWM output for actuators
- [ ] Telemetry support (e.g., MAVLink, custom protocol)

## Contributing
This project is in active development and open for contributions. If you’re learning Rust and interested in flight control, feel free to submit issues or pull requests.

## License
This project is licensed under the **MIT License**.
