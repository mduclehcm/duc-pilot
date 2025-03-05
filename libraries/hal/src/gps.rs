/// GPS sensor interface

/// GPS position data
#[derive(Debug, Clone, Copy)]
pub struct GpsPosition {
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters above sea level
    pub altitude: f64,
    /// Horizontal accuracy in meters
    pub accuracy: f32,
    /// Number of satellites used for this fix
    pub satellites: u8,
}

/// GPS velocity data
#[derive(Debug, Clone, Copy)]
pub struct GpsVelocity {
    /// Speed over ground in meters per second
    pub speed: f32,
    /// Course over ground in degrees (0-359, true north)
    pub course: f32,
    /// Vertical velocity in meters per second (positive up)
    pub vertical: f32,
    /// Velocity accuracy estimate in meters per second
    pub accuracy: f32,
}

/// GPS time data
#[derive(Debug, Clone, Copy)]
pub struct GpsTime {
    /// Year (e.g. 2023)
    pub year: u16,
    /// Month (1-12)
    pub month: u8,
    /// Day (1-31)
    pub day: u8,
    /// Hour (0-23)
    pub hour: u8,
    /// Minute (0-59)
    pub minute: u8,
    /// Second (0-59)
    pub second: u8,
    /// Millisecond (0-999)
    pub millisecond: u16,
    /// Whether time is valid and synchronized
    pub valid: bool,
}

/// GPS interface
pub trait GpsSensor {
    /// Initialize the GPS sensor
    fn init(&mut self) -> bool;
    
    /// Get the current GPS position
    fn get_position(&self) -> GpsPosition;
    
    /// Check if GPS has a valid fix
    fn has_fix(&self) -> bool;
    
    /// Calibrate the GPS sensor
    fn calibrate(&mut self) -> bool;
    
    /// Reset the GPS sensor to default state
    fn reset(&mut self) -> bool;
    
    /// Set the update rate in milliseconds
    fn set_update_rate(&mut self, rate_ms: u32) -> bool;
    
    /// Enable or disable power saving mode
    fn set_power_mode(&mut self, low_power: bool) -> bool;
    
    /// Get detailed information about visible satellites
    /// Returns an array of satellite information with the count of valid items
    fn get_satellite_info(&self, buffer: &mut [SatelliteInfo; 32]) -> u8;
    
    /// Get the time to first fix in seconds
    fn get_time_to_first_fix(&self) -> Option<f32>;
    
    /// Configure the minimum satellite count required for a valid fix
    fn set_min_satellites(&mut self, count: u8) -> bool;
    
    /// Get the current velocity information
    fn get_velocity(&self) -> GpsVelocity;
    
    /// Get the current GPS time
    fn get_time(&self) -> GpsTime;
    
    /// Get the horizontal dilution of precision (HDOP)
    /// Lower values indicate better positional accuracy
    fn get_hdop(&self) -> f32;
    
    /// Get the vertical dilution of precision (VDOP)
    /// Lower values indicate better vertical accuracy
    fn get_vdop(&self) -> f32;
    
    /// Enable or disable Satellite Based Augmentation System (SBAS)
    fn set_sbas_enabled(&mut self, enabled: bool) -> bool;
    
    /// Configure which GNSS constellations to use
    /// (GPS, GLONASS, Galileo, BeiDou, etc.)
    fn configure_gnss(&mut self, config: GnssConfig) -> bool;
    
    /// Send a raw NMEA command to the GPS module
    fn send_nmea_command(&mut self, command: &str) -> bool;
    
    /// Parse a raw NMEA sentence
    fn parse_nmea(&mut self, nmea: &str) -> bool;
    
    /// Get the estimated heading accuracy in degrees
    fn get_heading_accuracy(&self) -> f32;
    
    /// Enable or disable dead reckoning capabilities if available
    fn set_dead_reckoning(&mut self, enabled: bool) -> bool;
    
    /// Get the GPS status information
    fn get_status(&self) -> GpsStatus;
}

/// Information about a satellite
pub struct SatelliteInfo {
    /// Satellite ID number
    pub id: u16,
    /// Signal to noise ratio (dB)
    pub signal_strength: f32,
    /// Elevation angle in degrees (0-90)
    pub elevation: f32,
    /// Azimuth angle in degrees (0-359)
    pub azimuth: f32,
    /// Whether this satellite is used in position calculation
    pub in_use: bool,
}

/// GNSS constellation configuration
#[derive(Debug, Clone, Copy)]
pub struct GnssConfig {
    /// Use GPS constellation
    pub use_gps: bool,
    /// Use GLONASS constellation
    pub use_glonass: bool,
    /// Use Galileo constellation
    pub use_galileo: bool,
    /// Use BeiDou constellation
    pub use_beidou: bool,
    /// Use QZSS constellation
    pub use_qzss: bool,
}

/// GPS status information
#[derive(Debug, Clone)]
pub struct GpsStatus {
    /// Whether the GPS is functioning properly
    pub healthy: bool,
    
    /// The current fix type
    pub fix_type: GpsFixType,
    
    /// Number of satellites used for the current solution
    pub satellites_used: u8,
    
    /// Number of satellites visible
    pub satellites_visible: u8,
    
    /// Horizontal dilution of precision
    pub hdop: f32,
    
    /// Vertical dilution of precision
    pub vdop: f32,
    
    /// Whether SBAS is being used
    pub sbas_active: bool,
    
    /// Time since last fix in milliseconds
    pub time_since_fix_ms: u32,
}

/// GPS fix types
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsFixType {
    /// No fix available
    NoFix,
    /// 2D fix (no altitude)
    Fix2D,
    /// 3D fix (with altitude)
    Fix3D,
    /// Differential GPS fix
    DGps,
    /// Real Time Kinematic fix (centimeter accuracy)
    Rtk,
    /// Float RTK (decimeter accuracy)
    RtkFloat,
} 