/// RC Input interface

/// RC Input interface
pub trait RcInput {
    /// Initialize the RC input
    fn init(&mut self) -> bool;
    
    /// Get the number of available channels
    fn get_channel_count(&self) -> usize;
    
    /// Get the value of a specific channel (typically 1000-2000)
    fn get_channel(&self, channel: usize) -> u16;
    
    /// Check if the RC input is currently available
    fn is_available(&self) -> bool;
    
    /// Calibrate the RC input 
    ///
    /// Typically involves recording min, center, and max values for each channel
    /// Returns true if calibration was successful
    fn calibrate(&mut self) -> bool;
    
    /// Set the center/neutral position for a specific channel
    ///
    /// Returns true if the trim was successfully set
    fn set_trim(&mut self, channel: usize, trim_value: i16) -> bool;
    
    /// Get the current trim value for a specific channel
    fn get_trim(&self, channel: usize) -> i16;
    
    /// Check if the RC transmitter/receiver link is active
    fn is_link_active(&self) -> bool;
    
    /// Get the signal strength/quality (0-100%)
    fn get_signal_quality(&self) -> u8;
    
    /// Set failsafe values for all channels
    ///
    /// These values will be used if signal is lost
    /// Returns true if failsafe values were successfully set
    fn set_failsafe_values(&mut self, values: &[u16]) -> bool;
    
    /// Get the current failsafe values for all channels
    fn get_failsafe_values(&self, buffer: &mut [u16]) -> usize;
    
    /// Check if the receiver is currently in failsafe mode
    fn is_in_failsafe(&self) -> bool;
    
    /// Get the protocol type being used (e.g., PPM, SBUS, etc.)
    fn get_protocol_type(&self) -> RcProtocolType;
    
    /// Set the minimum and maximum limits for each channel
    ///
    /// Returns true if the limits were successfully set
    fn set_channel_limits(&mut self, channel: usize, min: u16, max: u16) -> bool;
    
    /// Get the current pulse width in microseconds
    fn get_pulse_width(&self, channel: usize) -> u16;
    
    /// Reset the RC input to default state
    ///
    /// Returns true if reset was successful
    fn reset(&mut self) -> bool;
}

/// RC protocol types
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RcProtocolType {
    /// Pulse Position Modulation
    PPM,
    /// Serial Bus
    SBUS,
    /// Digital Serial Protocol
    DSM,
    /// Crossfire Serial Protocol
    CRSF,
    /// Spectrum
    Spectrum,
    /// Unknown protocol
    Unknown,
}

/// RC Input status information
#[derive(Debug, Clone)]
pub struct RcInputStatus {
    /// Whether the RC input is functioning properly
    pub healthy: bool,
    
    /// The number of available channels
    pub channel_count: usize,
    
    /// The current signal quality (0-100%)
    pub signal_quality: u8,
    
    /// Whether failsafe mode is active
    pub failsafe_active: bool,
    
    /// The current protocol being used
    pub protocol: RcProtocolType,
    
    /// Time since last valid frame in milliseconds
    pub last_frame_ms: u32,
} 