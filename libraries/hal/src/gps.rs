extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;

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

impl Default for GpsPosition {
    fn default() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            accuracy: 0.0,
            satellites: 0,
        }
    }
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

impl Default for GpsVelocity {
    fn default() -> Self {
        Self {
            speed: 0.0,
            course: 0.0,
            vertical: 0.0,
            accuracy: 0.0,
        }
    }
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

impl Default for GpsTime {
    fn default() -> Self {
        Self {
            year: 0,
            month: 0,
            day: 0,
            hour: 0,
            minute: 0,
            second: 0,
            millisecond: 0,
            valid: false,
        }
    }
}

/// Error type for GPS operations
/// These errors are used to provide detailed information about what went wrong
/// during GPS operations, allowing applications to handle different error cases
/// appropriately.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsError {
    /// Device initialization failed
    /// This can happen due to communication issues or incompatible hardware
    InitFailed,
    /// Communication error with the GPS device
    /// This can occur due to serial connection issues, I/O errors, or protocol violations
    CommError,
    /// Operation timed out
    /// This happens when the GPS device does not respond within the expected timeframe
    Timeout,
    /// Error applying configuration to the GPS device
    /// This can happen if the device rejects a configuration or the command format is invalid
    ConfigError,
}

/// GPS device configuration
/// This structure contains information about the GPS device's capabilities and
/// current configuration settings. It is returned by the initialization process
/// to inform the application about the device's capabilities.
#[derive(Debug, Clone)]
pub struct GpsDeviceConfig {
    /// Device model name or identifier
    pub device_name: String,
    /// Firmware version of the GPS module
    pub firmware_version: String,
    /// Update rate in milliseconds - how frequently the device produces new position updates
    pub update_rate_ms: u32,
    /// Whether the device supports Satellite Based Augmentation System for improved accuracy
    pub sbas_supported: bool,
    /// List of GNSS constellations currently enabled on the device
    pub enabled_constellations: Vec<GnssConstellation>,
}

/// GNSS constellation types
/// Global Navigation Satellite Systems (GNSS) consist of different satellite constellations
/// operated by various countries. Modern GPS receivers often support multiple constellations
/// to improve accuracy and reliability.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GnssConstellation {
    /// GPS (United States) - The original and most widely used satellite navigation system
    Gps,
    /// GLONASS (Russia) - Russian alternative to GPS, second fully operational GNSS
    Glonass,
    /// Galileo (Europe) - European Union's global navigation system
    Galileo,
    /// BeiDou (China) - Chinese navigation system, also known as COMPASS
    BeiDou,
    /// QZSS (Japan) - Regional navigation system covering Japan and Asia-Oceania regions
    Qzss,
    /// SBAS (Satellite Based Augmentation System) - Includes WAAS, EGNOS, MSAS, GAGAN
    /// These systems improve accuracy through correction data transmitted by satellites
    Sbas,
}

/// Core GPS interface with essential functionality
pub trait GpsSensor {
    /// Initialize the GPS sensor
    /// 
    /// This asynchronous method configures the GPS module with default settings
    /// and prepares it for operation. It should be called before any other methods.
    ///
    /// Returns Ok with the detected GPS configuration on success, or Err on failure.
    /// The configuration contains information about the device capabilities and settings.
    async fn init(&mut self) -> Result<GpsDeviceConfig, GpsError>;
    
    /// Get the current GPS position
    fn get_position(&self) -> GpsPosition;
    
    /// Check if GPS has a valid fix
    fn has_fix(&self) -> bool;
    
    /// Get the current velocity information
    fn get_velocity(&self) -> GpsVelocity;
    
    /// Get the current GPS time
    fn get_time(&self) -> GpsTime;
    
    /// Get the GPS status information
    fn get_status(&self) -> GpsStatus;
    
    /// Get detailed information about visible satellites
    /// Returns an array of satellite information with the count of valid items
    fn get_satellite_info(&self, buffer: &mut [SatelliteInfo; 32]) -> u8;
    
    /// Get the horizontal dilution of precision (HDOP)
    /// Lower values indicate better positional accuracy
    fn get_hdop(&self) -> f32;
    
    /// Wait until new data is available from the GPS device
    /// 
    /// This asynchronous method blocks until new GPS data is received or a timeout occurs.
    /// It is useful for applications that need to process each new GPS reading as it arrives,
    /// without polling or busy-waiting.
    /// 
    /// Returns Ok(true) if new data was received, or Err(GpsError::Timeout) if the wait timed out,
    /// or another error if a communication problem occurred.
    async fn wait_data(&mut self) -> Result<bool, GpsError>;
}

/// Extended GPS functionality for advanced configuration
/// 
/// This trait provides methods for configuring advanced GPS features.
/// Implementation is optional and driver-specific, as not all GPS modules
/// support these configuration options.
///
/// Applications should check if a GPS driver implements this trait before
/// attempting to use these methods. Drivers can implement this trait to
/// expose hardware-specific configuration options.
pub trait GpsAdvancedConfig {
    /// Reset the GPS sensor to default state
    /// 
    /// This performs a factory reset of the GPS module, clearing all custom
    /// configurations and returning it to its default state. This is useful
    /// when the module is in an unknown or problematic state.
    ///
    /// Returns true if the reset was successful, false otherwise.
    fn reset(&mut self) -> bool;
    
    /// Set the update rate in milliseconds
    /// 
    /// Configure how frequently the GPS module produces new position updates.
    /// Lower values provide more frequent updates but consume more power.
    /// Typical values range from 100ms (10Hz) to 1000ms (1Hz).
    ///
    /// Returns true if the update rate was successfully set, false otherwise.
    fn set_update_rate(&mut self, rate_ms: u32) -> bool;
    
    /// Configure the minimum satellite count required for a valid fix
    /// 
    /// Sets the minimum number of satellites required for the GPS to report
    /// a valid position fix. Higher values generally increase accuracy but
    /// may reduce availability in challenging environments.
    ///
    /// Returns true if the setting was applied, false otherwise.
    fn set_min_satellites(&mut self, count: u8) -> bool;
    
    /// Configure which GNSS constellations to use
    /// 
    /// Enables or disables specific satellite constellations like GPS, GLONASS,
    /// Galileo, BeiDou, etc. Using multiple constellations typically improves
    /// accuracy and availability but may increase power consumption.
    ///
    /// Returns true if the configuration was applied, false otherwise.
    fn configure_gnss(&mut self, config: GnssConfig) -> bool;
}

/// Extended GPS functionality for performance monitoring
/// Implementations are optional and driver-specific
pub trait GpsPerformanceMonitoring {
    /// Get the time to first fix in seconds
    fn get_time_to_first_fix(&self) -> Option<f32>;
    
    /// Get the vertical dilution of precision (VDOP)
    /// Lower values indicate better vertical accuracy
    fn get_vdop(&self) -> f32;
    
    /// Get the estimated heading accuracy in degrees
    fn get_heading_accuracy(&self) -> f32;
    
    /// Calibrate the GPS sensor
    fn calibrate(&mut self) -> bool;
}

/// Extended GPS functionality for power management
/// Implementations are optional and driver-specific
pub trait GpsPowerManagement {
    /// Enable or disable power saving mode
    fn set_power_mode(&mut self, low_power: bool) -> bool;
}

/// Extended GPS functionality for special features
/// Implementations are optional and driver-specific
pub trait GpsSpecialFeatures {
    /// Enable or disable Satellite Based Augmentation System (SBAS)
    fn set_sbas_enabled(&mut self, enabled: bool) -> bool;
    
    /// Send a raw NMEA command to the GPS module
    fn send_nmea_command(&mut self, command: &str) -> bool;
    
    /// Parse a raw NMEA sentence
    fn parse_nmea(&mut self, nmea: &str) -> bool;
    
    /// Enable or disable dead reckoning capabilities if available
    fn set_dead_reckoning(&mut self, enabled: bool) -> bool;
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

impl Default for SatelliteInfo {
    fn default() -> Self {
        Self {
            id: 0,
            signal_strength: 0.0,
            elevation: 0.0,
            azimuth: 0.0,
            in_use: false,
        }
    }
}

// Make SatelliteInfo Copy to fix array initialization
impl Copy for SatelliteInfo {}
impl Clone for SatelliteInfo {
    fn clone(&self) -> Self {
        *self
    }
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

    /// Whether the GPS has a valid fix
    pub has_fix: bool,
}

impl Default for GpsStatus {
    fn default() -> Self {
        Self {
            healthy: false,
            fix_type: GpsFixType::NoFix,
            satellites_used: 0,
            satellites_visible: 0,
            hdop: 0.0,
            vdop: 0.0,
            sbas_active: false,
            time_since_fix_ms: 0,
            has_fix: false,
        }
    }
}

impl Copy for GpsStatus {}

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