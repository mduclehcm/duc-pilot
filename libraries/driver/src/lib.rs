#![no_std]

mod ublox;

// Export GPS driver types
pub use ublox::GpsData;
pub use ublox::UbloxGps;
