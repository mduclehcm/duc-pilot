# STM32 Hardware Drivers in Rust

## Overview
This crate provides a collection of STM32 hardware drivers implemented in a consistent and safe manner. It is designed to work seamlessly in an asynchronous environment using [Embassy](https://embassy.dev/) while prioritizing interrupt-driven operation for efficiency.

## Features
- **Unified Design**: All drivers follow a consistent implementation style.
- **Internal State Management**: Each driver maintains a snapshot of sensor data in its internal state.
- **Safe Asynchronous Access**: Uses appropriate guard mechanisms to ensure safe concurrent access in an `embassy` async environment.
- **Interrupt-Driven Operation**: Prioritizes handling data in an interrupt-driven manner for optimal performance.

## Getting Started
### Dependencies
Ensure you have the required dependencies in your `Cargo.toml`:

## Design Principles
### 1. **Consistent Implementation Style**
   - All drivers follow a uniform API structure to ensure maintainability and ease of integration.

### 2. **Snapshot-Based Data Management**
   - Each driver holds a snapshot of the latest sensor data within its internal state, reducing redundant I/O operations.

### 3. **Safe Access in Asynchronous Contexts**
   - Uses appropriate locking mechanisms (e.g., `Mutex`, `RefCell`, `CortexMutex`) to avoid race conditions in an `embassy` async environment.

### 4. **Interrupt-Based Data Acquisition**
   - Implements interrupt-driven operation whenever possible to minimize CPU load and improve real-time performance.
