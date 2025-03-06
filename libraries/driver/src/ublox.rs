//! u-blox GPS driver
//!
//! This module provides a driver for u-blox GPS receivers, supporting UBX protocol.
//! Following the design principles from README.md:
//! - Unified design and consistent implementation style
//! - Internal state management (snapshot of sensor data)
//! - Safe asynchronous access
//! - Interrupt-driven operation

extern crate alloc;

use embassy_futures::select::{select, Either};
use embassy_futures::yield_now;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Timer};
use hal::{GpsFixType, GpsPosition, GpsSensor, GpsStatus, GpsTime, GpsVelocity, SatelliteInfo, GpsDeviceConfig, GpsError, GnssConstellation};
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::{self, Debug, Display};

/// NMEA sentence buffer size
const NMEA_BUFFER_SIZE: usize = 128;

/// Default baud rate for uBlox GPS
const DEFAULT_BAUDRATE: u32 = 9600;

/// UBX Protocol constants
const UBX_SYNC_CHAR_1: u8 = 0xB5;
const UBX_SYNC_CHAR_2: u8 = 0x62;

/// UBX Message Classes
const UBX_CLASS_NAV: u8 = 0x01;
const UBX_CLASS_CFG: u8 = 0x06;
const UBX_CLASS_MON: u8 = 0x0A; // Monitoring Messages

/// UBX Message IDs
const UBX_NAV_PVT: u8 = 0x07; // Position, Velocity, Time Solution
const UBX_ID_CFG_PRT: u8 = 0x00; // Port Configuration
const UBX_ID_CFG_RATE: u8 = 0x08; // Navigation/Measurement Rate Settings
const UBX_ID_CFG_NAV5: u8 = 0x24; // Navigation Engine Settings
const UBX_ID_CFG_MSG: u8 = 0x01; // Set Message Rate
const UBX_ID_CFG_CFG: u8 = 0x09; // Clear/Save/Load Configuration
const UBX_ID_MON_VER: u8 = 0x04; // Monitor Version Information
const UBX_ID_CFG_GNSS: u8 = 0x3E; // GNSS Configuration

/// UBX Configuration Values
// Port configuration
const UBX_CFG_PRT_UART1: u8 = 0x01;
const UBX_CFG_PRT_MODE_8N1: u32 = 0x000008D0;
const UBX_CFG_PRT_PROTO_UBX_NMEA_RTCM: u32 = 0x00030007;

// Navigation configuration
const UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN: u8 = 0x03;
const UBX_CFG_NAV5_FIXMODE_AUTO: u8 = 0x00;
const UBX_CFG_NAV5_MASK_MIN_SV: u16 = 0x0001;  // Apply min SV settings
const UBX_CFG_NAV5_MASK_MIN_ELEV: u16 = 0x0002;  // Apply min elevation settings
const UBX_CFG_NAV5_MASK_PDOP: u16 = 0x0004;  // Apply PDOP mask settings

// Configuration save mask
const UBX_CFG_CFG_SAVE_ALL: u32 = 0x0000FFFF;
const UBX_CFG_CFG_DEV_BBR: u8 = 0x01;

// Message Rate (1 Hz)
const UBX_MSG_RATE_1HZ: u8 = 0x01;
const UBX_MSG_RATE_DISABLE: u8 = 0x00;

/// Default update rate (5Hz = 200ms)
const DEFAULT_UPDATE_RATE_MS: u16 = 200;

/// Default minimum satellites (use module default)
const DEFAULT_MIN_SATELLITES: u8 = 0;

/// Default minimum elevation angle in degrees
const DEFAULT_MIN_ELEVATION_DEG: u8 = 5;

/// Default position DOP mask (25.0 = 250)
const DEFAULT_PDOP_MASK: u16 = 250;

/// UBX-NAV-PVT message field positions
const NAV_PVT_YEAR_POS: usize = 4;
const NAV_PVT_MONTH_POS: usize = 6;
const NAV_PVT_DAY_POS: usize = 7;
const NAV_PVT_HOUR_POS: usize = 8;
const NAV_PVT_MINUTE_POS: usize = 9;
const NAV_PVT_SECOND_POS: usize = 10;
const NAV_PVT_FIX_TYPE_POS: usize = 20;
const NAV_PVT_SATELLITES_POS: usize = 23;
const NAV_PVT_LON_POS: usize = 24;
const NAV_PVT_LAT_POS: usize = 28;
const NAV_PVT_ALT_POS: usize = 36;
const NAV_PVT_HACC_POS: usize = 40;
const NAV_PVT_SPEED_POS: usize = 60;
const NAV_PVT_HEADING_POS: usize = 64;

/// Conversion factors
const LON_LAT_SCALE: f64 = 1e-7; // degrees scale factor
const ALT_HACC_SCALE: f64 = 1e-3; // mm to m
const SPEED_SCALE: f32 = 1e-3;    // mm/s to m/s
const HEADING_SCALE: f32 = 1e-5;  // deg * 1e-5 to deg
const VELOCITY_ACC_FACTOR: f32 = 0.1; // Approximate velocity accuracy
const HDOP_FACTOR: f32 = 5.0;     // Divisor for approximating HDOP

/// UBX fix type values
const FIX_TYPE_NO_FIX: u8 = 0;
const FIX_TYPE_2D: u8 = 1;
const FIX_TYPE_3D: u8 = 2;
const FIX_TYPE_DGPS: u8 = 3;
const FIX_TYPE_RTK: u8 = 4;
const FIX_TYPE_RTK_FLOAT: u8 = 5;

/// UBX message structure constants
const UBX_HEADER_SIZE: usize = 6;  // 2 sync + 1 class + 1 id + 2 length
const UBX_CHECKSUM_SIZE: usize = 2;
const UBX_MSG_OVERHEAD: usize = UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE;

// UBX message index constants
const UBX_SYNC1_POS: usize = 0;
const UBX_SYNC2_POS: usize = 1;
const UBX_CLASS_POS: usize = 2;
const UBX_ID_POS: usize = 3;
const UBX_LENGTH_LSB_POS: usize = 4;
const UBX_LENGTH_MSB_POS: usize = 5;
const UBX_PAYLOAD_POS: usize = 6;

// Bit masks
const BYTE_MASK: u16 = 0xFF;
const BYTE_SHIFT: u16 = 8;

/// Configuration for the UbloxGps module
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    /// UART baudrate for communication with the GPS module (default: 9600)
    pub baudrate: u32,

    /// Navigation measurement rate in milliseconds (default: 200ms = 5Hz)
    pub update_rate_ms: u16,

    /// Minimum number of satellites required for navigation (default: 0 = use module default)
    pub min_satellites: u8,

    /// Minimum elevation for satellites in degrees
    pub min_elevation_deg: u8,

    /// Position DOP mask, multiplied by 0.1 (default: 250 = 25.0)
    pub pdop_mask: u16,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            baudrate: DEFAULT_BAUDRATE,
            update_rate_ms: DEFAULT_UPDATE_RATE_MS,
            min_satellites: DEFAULT_MIN_SATELLITES,
            min_elevation_deg: DEFAULT_MIN_ELEVATION_DEG,
            pdop_mask: DEFAULT_PDOP_MASK,
        }
    }
}

impl GpsConfig {
    /// Create a new GpsConfig instance with the default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the baudrate
    pub fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
    }

    /// Set the update rate in milliseconds
    pub fn update_rate_ms(mut self, update_rate_ms: u16) -> Self {
        self.update_rate_ms = update_rate_ms;
        self
    }

    /// Set the minimum number of satellites required for navigation
    pub fn min_satellites(mut self, min_satellites: u8) -> Self {
        self.min_satellites = min_satellites;
        self
    }

    /// Set the minimum elevation for satellites in degrees
    pub fn min_elevation_deg(mut self, min_elevation_deg: u8) -> Self {
        self.min_elevation_deg = min_elevation_deg;
        self
    }

    /// Set the position DOP mask (multiplied by 0.1)
    pub fn pdop_mask(mut self, pdop_mask: u16) -> Self {
        self.pdop_mask = pdop_mask;
        self
    }
}

/// Internal state for UBX message parsing
#[derive(Debug, Clone, Copy, PartialEq)]
enum UbxParseState {
    /// Waiting for sync char 1 (0xB5)
    SyncChar1,
    /// Waiting for sync char 2 (0x62)
    SyncChar2,
    /// Reading class byte
    Class,
    /// Reading ID byte
    Id,
    /// Reading length byte 1 (LSB)
    Length1,
    /// Reading length byte 2 (MSB)
    Length2,
    /// Reading payload bytes
    Payload,
    /// Reading checksum byte A
    ChecksumA,
    /// Reading checksum byte B
    ChecksumB,
}

/// Driver state tracking
#[derive(Clone, Copy, PartialEq)]
pub enum UbloxState {
    /// Device not yet configured
    Unconfigured,
    /// Device configured and ready
    Configured,
    /// Configuration failed
    ConfigurationFailed,
}

/// Represents a u-blox GPS module
pub struct UbloxGps<'a> {
    /// UART interface
    uart: Uart<'a, Async>,

    /// Buffer for received data
    buffer: [u8; NMEA_BUFFER_SIZE],

    /// Current position in buffer
    buffer_idx: usize,

    /// GPS configuration
    config: GpsConfig,

    /// Current parse state
    parse_state: UbxParseState,

    /// Current message class
    msg_class: u8,

    /// Current message ID
    msg_id: u8,

    /// Payload length
    payload_length: u16,

    /// Bytes read from payload
    payload_bytes_read: u16,

    /// Current position
    position: GpsPosition,

    /// Current velocity
    velocity: GpsVelocity,

    /// Current time
    time: GpsTime,

    /// GPS status
    status: GpsStatus,

    /// Satellite information
    satellites_info: [SatelliteInfo; 32],

    /// Last update timestamp
    last_update: u64,

    /// Checksum A
    checksum_a: u8,
    
    /// Driver state
    state: UbloxState,

    /// Device info string (from MON-VER)
    device_info: String,
    
    /// Firmware version string (from MON-VER)
    firmware_version: String,
    
    /// GNSS configuration
    gnss_config: GnssConfig,
}

impl<'a> UbloxGps<'a> {
    /// Create a new UbloxGps instance with default configuration
    pub fn new(uart: Uart<'a, Async>) -> Self {
        Self::new_with_config(uart, GpsConfig::default())
    }

    /// Create a new UbloxGps instance with specific configuration
    pub fn new_with_config(uart: Uart<'a, Async>, config: GpsConfig) -> Self {
        Self {
            uart,
            buffer: [0; NMEA_BUFFER_SIZE],
            buffer_idx: 0,
            config,
            parse_state: UbxParseState::SyncChar1,
            msg_class: 0,
            msg_id: 0,
            payload_length: 0,
            payload_bytes_read: 0,
            position: GpsPosition::default(),
            velocity: GpsVelocity::default(),
            time: GpsTime::default(),
            status: GpsStatus::default(),
            satellites_info: [SatelliteInfo::default(); 32],
            last_update: 0,
            checksum_a: 0,
            state: UbloxState::Unconfigured,
            device_info: String::from("Unknown"),
            firmware_version: String::from("Unknown"),
            gnss_config: GnssConfig::default(),
        }
    }

    /// Updates internal state with new data received from the GPS
    /// Returns true if new data was successfully processed
    pub async fn update(&mut self, timeout_ms: u64) -> Result<bool, ()> {
        let start_time = embassy_time::Instant::now();
        let timeout_duration = Duration::from_millis(timeout_ms);
        let mut buffer = [0u8; 1];

        loop {
            // Calculate remaining time
            let elapsed = start_time.elapsed();
            if elapsed >= timeout_duration {
                // Timeout occurred
                return Ok(false);
            }

            // Calculate remaining time
            let remaining = timeout_duration - elapsed;

            // Set up timeout for this iteration
            let timeout = Timer::after(remaining);

            match select(self.uart.read(&mut buffer), timeout).await {
                Either::First(result) => {
                    match result {
                        Ok(_) => {
                            // Process the byte
                            if self.process_byte(buffer[0]) {
                                // Valid message processed
                                return Ok(true);
                            }

                            // Yield to other tasks
                            yield_now().await;
                        }
                        Err(_) => return Err(()),
                    }
                }
                Either::Second(_) => {
                    // Timeout occurred
                    return Ok(false);
                }
            }
        }
    }

    /// Process a received byte
    /// Returns true if a complete message was processed
    fn process_byte(&mut self, byte: u8) -> bool {
        match self.parse_state {
            UbxParseState::SyncChar1 => {
                if byte == UBX_SYNC_CHAR_1 {
                    self.parse_state = UbxParseState::SyncChar2;
                    self.buffer_idx = 0;
                }
            }
            UbxParseState::SyncChar2 => {
                if byte == UBX_SYNC_CHAR_2 {
                    self.parse_state = UbxParseState::Class;
                } else {
                    self.parse_state = UbxParseState::SyncChar1;
                }
            }
            UbxParseState::Class => {
                self.msg_class = byte;
                self.parse_state = UbxParseState::Id;
            }
            UbxParseState::Id => {
                self.msg_id = byte;
                self.parse_state = UbxParseState::Length1;
            }
            UbxParseState::Length1 => {
                self.payload_length = byte as u16;
                self.parse_state = UbxParseState::Length2;
            }
            UbxParseState::Length2 => {
                self.payload_length |= (byte as u16) << 8;
                self.payload_bytes_read = 0;

                // Check if payload exceeds buffer size
                if self.payload_length > self.buffer.len() as u16 {
                    // Reset to initial state if payload is too large
                    self.parse_state = UbxParseState::SyncChar1;
                } else if self.payload_length > 0 {
                    self.parse_state = UbxParseState::Payload;
                } else {
                    self.parse_state = UbxParseState::ChecksumA;
                }
            }
            UbxParseState::Payload => {
                if self.buffer_idx < self.buffer.len() {
                    self.buffer[self.buffer_idx] = byte;
                    self.buffer_idx += 1;
                }

                self.payload_bytes_read += 1;
                if self.payload_bytes_read >= self.payload_length {
                    self.parse_state = UbxParseState::ChecksumA;
                }
            }
            UbxParseState::ChecksumA => {
                // Store the first checksum byte
                self.checksum_a = byte;
                self.parse_state = UbxParseState::ChecksumB;
            }
            UbxParseState::ChecksumB => {
                // Validate checksum
                let checksum_b = byte;
                let message_data = [
                    self.msg_class,
                    self.msg_id,
                    (self.payload_length & 0xFF) as u8,
                    ((self.payload_length >> 8) & 0xFF) as u8,
                ];

                let (calc_ck_a, calc_ck_b) = Self::calculate_checksum(
                    &[
                        &message_data[..],
                        &self.buffer[..self.payload_bytes_read as usize],
                    ]
                    .concat(),
                );

                // Reset parse state
                self.parse_state = UbxParseState::SyncChar1;

                // Process message only if checksum is valid
                if calc_ck_a == self.checksum_a && calc_ck_b == checksum_b {
                    if self.msg_class == UBX_CLASS_NAV && self.msg_id == UBX_NAV_PVT {
                        self.process_nav_pvt();
                        return true;
                    }
                }
            }
        }

        false
    }

    /// Process NAV-PVT message and update internal state
    fn process_nav_pvt(&mut self) {
        // NAV-PVT message should be 92 bytes
        const NAV_PVT_LENGTH: usize = 92;

        if self.buffer_idx < NAV_PVT_LENGTH {
            // Not enough data, ignore this message
            return;
        }

        // Use a safer approach to extract time data
        let year = if self.buffer_idx >= NAV_PVT_YEAR_POS + 2 {
            u16::from_le_bytes([self.buffer[NAV_PVT_YEAR_POS], self.buffer[NAV_PVT_YEAR_POS + 1]])
        } else {
            0
        };

        let month = if self.buffer_idx >= NAV_PVT_MONTH_POS + 1 {
            self.buffer[NAV_PVT_MONTH_POS]
        } else {
            0
        };
        
        let day = if self.buffer_idx >= NAV_PVT_DAY_POS + 1 {
            self.buffer[NAV_PVT_DAY_POS]
        } else {
            0
        };
        
        let hour = if self.buffer_idx >= NAV_PVT_HOUR_POS + 1 {
            self.buffer[NAV_PVT_HOUR_POS]
        } else {
            0
        };
        
        let minute = if self.buffer_idx >= NAV_PVT_MINUTE_POS + 1 {
            self.buffer[NAV_PVT_MINUTE_POS]
        } else {
            0
        };
        
        let second = if self.buffer_idx >= NAV_PVT_SECOND_POS + 1 {
            self.buffer[NAV_PVT_SECOND_POS]
        } else {
            0
        };

        // Extract fix type and satellite count with bounds checking
        let fix_type = if self.buffer_idx >= NAV_PVT_FIX_TYPE_POS + 1 {
            self.buffer[NAV_PVT_FIX_TYPE_POS]
        } else {
            0
        };
        
        let satellites = if self.buffer_idx >= NAV_PVT_SATELLITES_POS + 1 {
            self.buffer[NAV_PVT_SATELLITES_POS]
        } else {
            0
        };

        // Only proceed with further parsing if we have enough data
        if self.buffer_idx >= NAV_PVT_LENGTH {
            // Extract position
            let lon_bits = u32::from_le_bytes([
                self.buffer[NAV_PVT_LON_POS],
                self.buffer[NAV_PVT_LON_POS + 1],
                self.buffer[NAV_PVT_LON_POS + 2],
                self.buffer[NAV_PVT_LON_POS + 3],
            ]);
            let longitude = (lon_bits as f64) * LON_LAT_SCALE;

            let lat_bits = u32::from_le_bytes([
                self.buffer[NAV_PVT_LAT_POS],
                self.buffer[NAV_PVT_LAT_POS + 1],
                self.buffer[NAV_PVT_LAT_POS + 2],
                self.buffer[NAV_PVT_LAT_POS + 3],
            ]);
            let latitude = (lat_bits as f64) * LON_LAT_SCALE;

            let alt_bits = i32::from_le_bytes([
                self.buffer[NAV_PVT_ALT_POS],
                self.buffer[NAV_PVT_ALT_POS + 1],
                self.buffer[NAV_PVT_ALT_POS + 2],
                self.buffer[NAV_PVT_ALT_POS + 3],
            ]);
            let altitude = (alt_bits as f64) * ALT_HACC_SCALE; // mm to m

            // Extract accuracy
            let h_acc = u32::from_le_bytes([
                self.buffer[NAV_PVT_HACC_POS],
                self.buffer[NAV_PVT_HACC_POS + 1],
                self.buffer[NAV_PVT_HACC_POS + 2],
                self.buffer[NAV_PVT_HACC_POS + 3],
            ]);
            let acc = (h_acc as f32) * SPEED_SCALE; // mm to m

            // Extract velocity data
            let speed_bits = u32::from_le_bytes([
                self.buffer[NAV_PVT_SPEED_POS],
                self.buffer[NAV_PVT_SPEED_POS + 1],
                self.buffer[NAV_PVT_SPEED_POS + 2],
                self.buffer[NAV_PVT_SPEED_POS + 3],
            ]);
            let speed = (speed_bits as f32) * SPEED_SCALE; // mm/s to m/s

            let heading_bits = u32::from_le_bytes([
                self.buffer[NAV_PVT_HEADING_POS],
                self.buffer[NAV_PVT_HEADING_POS + 1],
                self.buffer[NAV_PVT_HEADING_POS + 2],
                self.buffer[NAV_PVT_HEADING_POS + 3],
            ]);
            let course = (heading_bits as f32) * HEADING_SCALE; // deg * 1e-5 to deg

            // Update internal state
            self.position = GpsPosition {
                latitude,
                longitude,
                altitude,
                accuracy: acc,
                satellites,
            };

            self.velocity = GpsVelocity {
                speed,
                course,
                vertical: 0.0,       // Not provided in basic message
                accuracy: acc * VELOCITY_ACC_FACTOR, // Approximate velocity accuracy
            };

            // Update HDOP based on horizontal accuracy
            self.status.hdop = acc / HDOP_FACTOR; // Approximate HDOP from horizontal accuracy
        }

        // Update time state - separate from position since we might have partial data
        self.time = GpsTime {
            year,
            month,
            day,
            hour,
            minute,
            second,
            millisecond: 0, // Not provided with millisecond precision
            valid: fix_type > 0,
        };

        // Update status state
        self.status.has_fix = fix_type > 0;
        self.status.satellites_used = satellites;
        self.status.satellites_visible = satellites;
        self.status.fix_type = match fix_type {
            FIX_TYPE_NO_FIX => GpsFixType::NoFix,
            FIX_TYPE_2D => GpsFixType::Fix2D,
            FIX_TYPE_3D => GpsFixType::Fix3D,
            FIX_TYPE_DGPS => GpsFixType::DGps,
            FIX_TYPE_RTK => GpsFixType::Rtk,
            FIX_TYPE_RTK_FLOAT => GpsFixType::RtkFloat,
            _ => GpsFixType::NoFix,
        };

        self.last_update = embassy_time::Instant::now().as_millis();
    }

    /// Send a UBX protocol message
    pub async fn send_ubx_message(&mut self, class: u8, id: u8, payload: &[u8]) -> Result<(), UBloxError> {
        let mut checksum_a: u8 = 0;
        let mut checksum_b: u8 = 0;
        
        // Send header bytes (0xB5, 0x62)
        if let Err(_) = self.uart.write(&[0xB5, 0x62]).await {
            return Err(UBloxError::UartWriteError);
        }
        
        // Send class and ID
        if let Err(_) = self.uart.write(&[class, id]).await {
            return Err(UBloxError::UartWriteError);
        }
        
        // Update checksums
        checksum_a = checksum_a.wrapping_add(class);
        checksum_b = checksum_b.wrapping_add(checksum_a);
        
        checksum_a = checksum_a.wrapping_add(id);
        checksum_b = checksum_b.wrapping_add(checksum_a);
        
        // Send length (2 bytes, little endian)
        let len = payload.len() as u16;
        let len_bytes = [len as u8, (len >> 8) as u8];
        
        if let Err(_) = self.uart.write(&len_bytes).await {
            return Err(UBloxError::UartWriteError);
        }
        
        // Update checksums
        checksum_a = checksum_a.wrapping_add(len_bytes[0]);
        checksum_b = checksum_b.wrapping_add(checksum_a);
        
        checksum_a = checksum_a.wrapping_add(len_bytes[1]);
        checksum_b = checksum_b.wrapping_add(checksum_a);
        
        // Send payload
        if payload.len() > 0 {
            if let Err(_) = self.uart.write(payload).await {
                return Err(UBloxError::UartWriteError);
            }
            
            // Update checksums
            for &byte in payload {
                checksum_a = checksum_a.wrapping_add(byte);
                checksum_b = checksum_b.wrapping_add(checksum_a);
            }
        }
        
        // Send checksums
        if let Err(_) = self.uart.write(&[checksum_a, checksum_b]).await {
            return Err(UBloxError::UartWriteError);
        }
        
        Ok(())
    }

    /// Calculate UBX checksum
    fn calculate_checksum(data: &[u8]) -> (u8, u8) {
        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;

        for &byte in data {
            ck_a = ck_a.wrapping_add(byte);
            ck_b = ck_b.wrapping_add(ck_a);
        }

        (ck_a, ck_b)
    }

    /// Check if GPS is configured
    pub fn is_configured(&self) -> bool {
        self.state == UbloxState::Configured
    }

    /// Get the detailed configuration status
    pub fn get_configuration_status(&self) -> UbloxState {
        self.state
    }

    /// Initialize the GPS module asynchronously
    ///
    /// This method performs the complete initialization sequence:
    /// 1. Reset the GPS state
    /// 2. Auto-configure the device (query capabilities)
    /// 3. Apply any custom configuration if specified
    ///
    /// Returns a GpsDeviceConfig on success containing detailed device information
    pub async fn init_async(&mut self) -> Result<GpsDeviceConfig, UBloxError> {
        // Reset internal state
        self.reset_state();
        
        // Query device capabilities and current configuration
        let device_config = self.auto_configure().await?;
        
        // Mark the device as initialized
        self.state = UbloxState::Configured;
        
        Ok(device_config)
    }

    /// Configure the GPS module with custom settings
    ///
    /// This method updates the internal configuration and applies it immediately.
    /// After configuration, it queries the device to retrieve the updated settings.
    ///
    /// Args:
    ///   config: GpsConfig containing the desired configuration
    ///
    /// Returns:
    ///   GpsDeviceConfig containing the actual applied configuration
    pub async fn configure(&mut self, config: GpsConfig) -> Result<GpsDeviceConfig, UBloxError> {
        // Update internal configuration
        self.config = config;
        
        // Apply each configuration component
        // 1. Configure UART port settings
        if let Err(_) = self.send_ubx_message(
            UBX_CLASS_CFG, 
            UBX_ID_CFG_PRT, 
            &[
                UBX_CFG_PRT_UART1, 0x00, 0x00, 0x00, // Port ID (UART1)
                0x00, 0x00, 0x00, 0x00, // Reserved
                (UBX_CFG_PRT_MODE_8N1 & 0xFF) as u8,
                ((UBX_CFG_PRT_MODE_8N1 >> 8) & 0xFF) as u8,
                ((UBX_CFG_PRT_MODE_8N1 >> 16) & 0xFF) as u8,
                ((UBX_CFG_PRT_MODE_8N1 >> 24) & 0xFF) as u8,
                (self.config.baudrate & 0xFF) as u8,
                ((self.config.baudrate >> 8) & 0xFF) as u8,
                ((self.config.baudrate >> 16) & 0xFF) as u8,
                ((self.config.baudrate >> 24) & 0xFF) as u8,
                (UBX_CFG_PRT_PROTO_UBX_NMEA_RTCM & 0xFF) as u8,
                ((UBX_CFG_PRT_PROTO_UBX_NMEA_RTCM >> 8) & 0xFF) as u8,
                ((UBX_CFG_PRT_PROTO_UBX_NMEA_RTCM >> 16) & 0xFF) as u8,
                ((UBX_CFG_PRT_PROTO_UBX_NMEA_RTCM >> 24) & 0xFF) as u8,
                0x00, 0x00, 0x00, 0x00, // Flags
            ]
        ).await {
            return Err(UBloxError::ConfigurationFailed);
        }
        
        // Small delay to let the baudrate change take effect
        Timer::after(Duration::from_millis(100)).await;
        
        // 2. Configure navigation settings
        if self.config.min_satellites > 0 || self.config.min_elevation_deg > 0 || self.config.pdop_mask > 0 {
            // Calculate the mask based on which settings we're applying
            let mut mask = 0u16;
            if self.config.min_satellites > 0 {
                mask |= UBX_CFG_NAV5_MASK_MIN_SV;
            }
            if self.config.min_elevation_deg > 0 {
                mask |= UBX_CFG_NAV5_MASK_MIN_ELEV;
            }
            if self.config.pdop_mask > 0 {
                mask |= UBX_CFG_NAV5_MASK_PDOP;
            }
            
            if let Err(_) = self.send_ubx_message(
                UBX_CLASS_CFG, 
                UBX_ID_CFG_NAV5, 
                &[
                    (mask & 0xFF) as u8,
                    ((mask >> 8) & 0xFF) as u8, // Apply mask for selected settings
                    UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN, // Dynamic model: Pedestrian
                    UBX_CFG_NAV5_FIXMODE_AUTO, // Fix mode: 2D/3D auto
                    0x00, 0x00, 0x00, 0x00, // Fixed altitude
                    0x00, 0x00, 0x00, 0x00, // Fixed altitude variance
                    self.config.min_satellites, // Min SVs for navigation
                    self.config.min_elevation_deg, // Min elevation for SVs
                    0x00, // P-DOP mask (used if config is set)
                    (self.config.pdop_mask & 0xFF) as u8, // P-DOP mask LSB
                    ((self.config.pdop_mask >> 8) & 0xFF) as u8, // P-DOP mask MSB
                    0x00, 0x00, // Time DOP mask
                    0x00, 0x00, // Position accuracy mask
                    0x00, 0x00, // Time accuracy mask
                    0x00, // Static hold threshold
                    0x00, // DGNSS timeout
                    0x00, 0x00, 0x00, 0x00, // Reserved
                    0x00, 0x00, // Reserved
                    0x00, 0x00, // Reserved
                ]
            ).await {
                return Err(UBloxError::ConfigurationFailed);
            }
        }
        
        // 3. Set measurement rate
        if let Err(_) = self.send_ubx_message(
            UBX_CLASS_CFG, 
            UBX_ID_CFG_RATE, 
            &[
                (self.config.update_rate_ms & 0xFF) as u8,
                ((self.config.update_rate_ms >> 8) & 0xFF) as u8,
                UBX_MSG_RATE_1HZ, 0x00, // Navigation count
                UBX_MSG_RATE_1HZ, 0x00, // Time reference (GPS)
            ]
        ).await {
            return Err(UBloxError::ConfigurationFailed);
        }
        
        // 4. Configure which messages to receive
        if let Err(_) = self.send_ubx_message(
            UBX_CLASS_CFG, 
            UBX_ID_CFG_MSG, 
            &[
                UBX_CLASS_NAV, UBX_NAV_PVT, // Message class and ID
                UBX_MSG_RATE_1HZ, // Rate on port 0 (I2C)
                UBX_MSG_RATE_1HZ, // Rate on port 1 (UART1)
                UBX_MSG_RATE_DISABLE, // Rate on port 2 (UART2)
                UBX_MSG_RATE_DISABLE, // Rate on port 3 (USB)
                UBX_MSG_RATE_DISABLE, // Rate on port 4 (SPI)
                UBX_MSG_RATE_DISABLE, // Rate on port 5 (Reserved)
            ]
        ).await {
            return Err(UBloxError::ConfigurationFailed);
        }
        
        // 5. Save configuration to non-volatile memory
        if let Err(_) = self.send_ubx_message(
            UBX_CLASS_CFG, 
            UBX_ID_CFG_CFG, 
            &[
                0x00, 0x00, 0x00, 0x00, // Clear mask
                (UBX_CFG_CFG_SAVE_ALL & 0xFF) as u8,
                ((UBX_CFG_CFG_SAVE_ALL >> 8) & 0xFF) as u8,
                ((UBX_CFG_CFG_SAVE_ALL >> 16) & 0xFF) as u8,
                ((UBX_CFG_CFG_SAVE_ALL >> 24) & 0xFF) as u8, // Save mask (all settings)
                0x00, 0x00, 0x00, 0x00, // Load mask
                UBX_CFG_CFG_DEV_BBR, // Device mask (BBR)
            ]
        ).await {
            return Err(UBloxError::ConfigurationFailed);
        }
        
        // Configuration successful
        self.state = UbloxState::Configured;
        
        // Query the updated configuration to return accurate information
        let device_config = self.auto_configure().await?;
        
        Ok(device_config)
    }

    /// Auto-configures the device based on the available information
    /// 
    /// This method attempts to query the device for its configuration
    /// and uses that to populate a GpsDeviceConfig struct.
    /// 
    /// Returns the populated GpsDeviceConfig on success.
    pub async fn auto_configure(&mut self) -> Result<GpsDeviceConfig, UBloxError> {
        // Query version information
        self.query_version_info().await?;
        
        // Query GNSS configuration
        self.query_gnss_config().await?;
        
        // Build the device config
        let mut constellations = Vec::new();
        if self.gnss_config.gps_enabled {
            constellations.push(GnssConstellation::Gps);
        }
        if self.gnss_config.galileo_enabled {
            constellations.push(GnssConstellation::Galileo);
        }
        if self.gnss_config.glonass_enabled {
            constellations.push(GnssConstellation::Glonass);
        }
        if self.gnss_config.beidou_enabled {
            constellations.push(GnssConstellation::BeiDou);
        }
        if self.gnss_config.qzss_enabled {
            constellations.push(GnssConstellation::Qzss);
        }
        if self.gnss_config.sbas_active {
            constellations.push(GnssConstellation::Sbas);
        }
        
        // Create device config
        let device_config = GpsDeviceConfig {
            device_name: self.device_info.clone(),
            firmware_version: self.firmware_version.clone(),
            update_rate_ms: self.config.update_rate_ms as u32,
            sbas_supported: true,  // All modern u-blox modules support this
            enabled_constellations: constellations,
        };
        
        Ok(device_config)
    }

    /// Query version information from the device
    /// 
    /// Sends a UBX-MON-VER message to the device and waits for a response
    /// which contains the device name, hardware version, and firmware version.
    /// This information is stored in the device_info and firmware_version fields.
    ///
    /// Returns Ok(()) if successful, or an error otherwise.
    pub async fn query_version_info(&mut self) -> Result<(), UBloxError> {
        // Send UBX-MON-VER message (no payload)
        if let Err(_) = self.send_ubx_message(UBX_CLASS_MON, UBX_ID_MON_VER, &[]).await {
            return Err(UBloxError::UartWriteError);
        }
        
        // Wait for response with timeout of 1 second
        const TIMEOUT_MS: u64 = 1000;
        let start_time = embassy_time::Instant::now();
        
        // We'll keep track of whether we've received the version message
        let mut version_received = false;
        
        while embassy_time::Instant::now() - start_time < Duration::from_millis(TIMEOUT_MS) {
            // Read data from UART
            let mut byte = [0u8; 1];
            if let Ok(_) = self.uart.read(&mut byte).await {
                // Process the byte
                if self.process_byte(byte[0]) {
                    // If we've received a complete message, check if it's the version message
                    if self.msg_class == UBX_CLASS_MON && self.msg_id == UBX_ID_MON_VER {
                        // Parse MON-VER message which contains SW and HW version
                        // Format: 30 bytes for software version, then 10-byte blocks for extensions
                        // We extract the first part for device info and software version
                        if self.buffer_idx >= 40 {
                            // Extract SW Version (first 30 bytes)
                            let sw_version = core::str::from_utf8(&self.buffer[0..30])
                                .unwrap_or("Unknown")
                                .trim_end_matches('\0');
                            
                            // Extract HW Version (next 10 bytes)
                            let hw_version = core::str::from_utf8(&self.buffer[30..40])
                                .unwrap_or("Unknown")
                                .trim_end_matches('\0');
                            
                            // Combine for firmware version
                            self.firmware_version = String::from(sw_version);
                            
                            // Combine for device info
                            let mut device_info = String::from("u-blox ");
                            device_info.push_str(hw_version);
                            self.device_info = device_info;
                            
                            version_received = true;
                            break;
                        }
                    }
                }
            } else {
                // Read timeout or error
                yield_now().await;
            }
        }
        
        if version_received {
            Ok(())
        } else {
            Err(UBloxError::VersionQueryFailed)
        }
    }

    /// Query GNSS configuration from the device
    /// 
    /// Sends a UBX-CFG-GNSS message to the device to query which
    /// GNSS constellations are enabled.
    ///
    /// Returns Ok(()) if successful, or an error otherwise.
    pub async fn query_gnss_config(&mut self) -> Result<(), UBloxError> {
        // Send UBX-CFG-GNSS message (no payload for a poll request)
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_ID_CFG_GNSS, &[]).await {
            return Err(UBloxError::UartWriteError);
        }
        
        // Wait for response with timeout of 1 second
        const TIMEOUT_MS: u64 = 1000;
        let start_time = embassy_time::Instant::now();
        
        // We'll keep track of whether we've received the GNSS config message
        let mut config_received = false;
        
        while embassy_time::Instant::now() - start_time < Duration::from_millis(TIMEOUT_MS) {
            // Read data from UART
            let mut byte = [0u8; 1];
            if let Ok(_) = self.uart.read(&mut byte).await {
                // Process the byte
                if self.process_byte(byte[0]) {
                    // If we've received a complete message, check if it's the GNSS config
                    if self.msg_class == UBX_CLASS_CFG && self.msg_id == UBX_ID_CFG_GNSS {
                        // Parse CFG-GNSS message
                        // Format: 4 bytes header, then blocks of 8 bytes per GNSS
                        if self.buffer_idx >= 12 { // At least header + 1 block
                            // Reset configuration
                            self.gnss_config = GnssConfig::default();
                            
                            // Number of configs in message is in the first byte
                            let num_configs = self.buffer[0] as usize;
                            
                            // Each 8-byte block represents a GNSS configuration
                            for i in 0..num_configs {
                                if 4 + i * 8 + 8 <= self.buffer_idx {
                                    let block_start = 4 + i * 8;
                                    let gnss_id = self.buffer[block_start];
                                    let flags = self.buffer[block_start + 3];
                                    let enabled = (flags & 0x01) != 0;
                                    
                                    // Set the appropriate flag based on GNSS ID
                                    match gnss_id {
                                        0 => self.gnss_config.gps_enabled = enabled,
                                        1 => self.gnss_config.sbas_active = enabled,
                                        2 => self.gnss_config.galileo_enabled = enabled,
                                        3 => self.gnss_config.beidou_enabled = enabled,
                                        4 => self.gnss_config.qzss_enabled = enabled,
                                        6 => self.gnss_config.glonass_enabled = enabled,
                                        _ => {} // Unknown GNSS ID
                                    }
                                }
                            }
                            
                            config_received = true;
                            break;
                        }
                    }
                }
            } else {
                // Read timeout or error
                yield_now().await;
            }
        }
        
        if config_received {
            Ok(())
        } else {
            Err(UBloxError::GnssQueryFailed)
        }
    }

    /// Reset internal state
    fn reset_state(&mut self) {
        self.parse_state = UbxParseState::SyncChar1;
        self.msg_class = 0;
        self.msg_id = 0;
        self.payload_length = 0;
        self.payload_bytes_read = 0;
    }
}

impl<'a> GpsSensor for UbloxGps<'a> {
    /// Initialize the GPS sensor
    ///
    /// Asynchronously configures the GPS module with the current settings
    /// and prepares it for operation. This implementation uses the auto_configure
    /// method which handles the complete initialization process including:
    /// - Configuring communication parameters
    /// - Setting up navigation and measurement settings
    /// - Querying device capabilities
    ///
    /// Returns detailed configuration information on success, including:
    /// - Device name and firmware version (queried from device)
    /// - Update rate and enabled features
    /// - Active GNSS constellations
    async fn init(&mut self) -> Result<GpsDeviceConfig, GpsError> {
        // The auto_configure method now handles the complete initialization process
        // including querying device information
        self.auto_configure().await.map_err(|e| e.into())
    }

    /// Get current GPS position
    fn get_position(&self) -> GpsPosition {
        self.position
    }

    /// Get current GPS velocity
    fn get_velocity(&self) -> GpsVelocity {
        self.velocity
    }

    /// Get current GPS time
    fn get_time(&self) -> GpsTime {
        self.time
    }

    /// Check if GPS has a valid fix
    fn has_fix(&self) -> bool {
        self.status.has_fix
    }

    /// Get GPS status
    fn get_status(&self) -> GpsStatus {
        self.status.clone()
    }

    /// Get satellite information
    fn get_satellite_info(&self, satellites: &mut [SatelliteInfo; 32]) -> u8 {
        // Copy satellite information
        *satellites = self.satellites_info;
        self.status.satellites_visible
    }

    /// Get horizontal dilution of precision (HDOP)
    fn get_hdop(&self) -> f32 {
        self.status.hdop
    }
    
    /// Wait until new data is available from the GPS device
    /// 
    /// This asynchronous method blocks until new GPS data is received or a timeout occurs.
    /// It is useful for applications that need to process each new GPS reading as it arrives,
    /// without polling or busy-waiting.
    /// 
    /// Returns:
    /// - Ok(true): New valid GPS data was successfully received and parsed
    /// - Ok(false): Some data was received but no complete/valid message was parsed
    /// - Err(GpsError::Timeout): No data was received within the timeout period
    /// - Err(GpsError::CommError): A communication error occurred with the device
    async fn wait_data(&mut self) -> Result<bool, GpsError> {
        // Default timeout of 1000ms (1 second)
        const DEFAULT_TIMEOUT_MS: u64 = 1000;
        
        // Wait for new data from the GPS
        match self.update(DEFAULT_TIMEOUT_MS).await {
            Ok(received_data) => Ok(received_data),
            Err(_) => Err(GpsError::Timeout),
        }
    }
}

// Add GnssConfig struct to store satellite constellation configuration
#[derive(Debug, Clone)]
struct GnssConfig {
    gps_enabled: bool,
    glonass_enabled: bool,
    galileo_enabled: bool,
    beidou_enabled: bool,
    qzss_enabled: bool,
    sbas_active: bool,
}

impl Default for GnssConfig {
    fn default() -> Self {
        Self {
            gps_enabled: true,      // GPS is generally enabled by default
            glonass_enabled: false,
            galileo_enabled: false,
            beidou_enabled: false,
            qzss_enabled: false,
            sbas_active: false,
        }
    }
}

/// Error type for UBlox GPS operations
#[derive(Debug)]
pub enum UBloxError {
    /// Failed to write to UART
    UartWriteError,
    
    /// Failed to read from UART
    UartReadError,
    
    /// Timeout waiting for response
    Timeout,
    
    /// Version query failed
    VersionQueryFailed,
    
    /// GNSS query failed
    GnssQueryFailed,
    
    /// Configuration failed
    ConfigurationFailed,
}

impl Display for UBloxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UartWriteError => write!(f, "Failed to write to UART"),
            Self::UartReadError => write!(f, "Failed to read from UART"),
            Self::Timeout => write!(f, "Timeout waiting for response"),
            Self::VersionQueryFailed => write!(f, "Failed to query version information"),
            Self::GnssQueryFailed => write!(f, "Failed to query GNSS configuration"),
            Self::ConfigurationFailed => write!(f, "Failed to apply configuration"),
        }
    }
}

/// Convert from UBloxError to GpsError for API compatibility
impl From<UBloxError> for GpsError {
    fn from(err: UBloxError) -> Self {
        match err {
            UBloxError::UartWriteError | UBloxError::UartReadError => GpsError::CommError,
            UBloxError::Timeout => GpsError::Timeout,
            UBloxError::VersionQueryFailed | UBloxError::GnssQueryFailed => GpsError::InitFailed,
            UBloxError::ConfigurationFailed => GpsError::ConfigError,
        }
    }
}
