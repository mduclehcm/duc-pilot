//! u-blox GPS driver
//!
//! This module provides a driver for u-blox GPS receivers, supporting UBX protocol.
//! Following the design principles from README.md:
//! - Unified design and consistent implementation style
//! - Internal state management (snapshot of sensor data)
//! - Safe asynchronous access
//! - Interrupt-driven operation

use embassy_futures::select::{select, Either};
use embassy_futures::yield_now;
use embassy_stm32::usart::Uart;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use hal::{GnssConfig, GpsPosition, GpsSensor, GpsStatus, GpsTime, GpsVelocity, SatelliteInfo, GpsFixType};

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
const UBX_CLASS_MON: u8 = 0x0A;
const UBX_CLASS_ACK: u8 = 0x05;

/// UBX Message IDs
const UBX_CFG_PRT: u8 = 0x00; // Ports Configuration
const UBX_CFG_MSG: u8 = 0x01; // Message Configuration
const UBX_CFG_RATE: u8 = 0x08; // Navigation Rate
const UBX_CFG_NAV5: u8 = 0x24; // Navigation Engine Settings
const UBX_NAV_PVT: u8 = 0x07; // Position, Velocity, Time Solution
const UBX_NAV_SAT: u8 = 0x35; // Satellite Information
const UBX_MON_VER: u8 = 0x04; // Receiver/Software Version
const UBX_ACK_ACK: u8 = 0x01; // Message Acknowledged

/// Configuration for the UbloxGps module
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    /// UART baudrate for communication with the GPS module (default: 9600)
    pub baudrate: u32,

    /// Navigation measurement rate in milliseconds (default: 200ms = 5Hz)
    pub update_rate_ms: u16,

    /// Minimum number of satellites required for navigation (default: 0 = use module default)
    pub min_satellites: u8,

    /// Minimum elevation for satellites to be used in fix, in degrees (default: 5 degrees)
    pub min_elevation_deg: u8,

    /// Position DOP mask, multiplied by 0.1 (default: 250 = 25.0)
    pub pdop_mask: u16,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            baudrate: 9600,
            update_rate_ms: 200, // 5Hz
            min_satellites: 0,   // Use module default
            min_elevation_deg: 5,
            pdop_mask: 250, // 25.0
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
        }
    }

    /// Updates internal state with new data received from the GPS
    /// Returns true if new data was successfully processed
    pub async fn update(&mut self, timeout_ms: u64) -> Result<bool, ()> {
        let mut timeout = Timer::after(Duration::from_millis(timeout_ms));
        let mut buffer = [0u8; 1];
        let mut updated = false;

        loop {
            match select(self.uart.read(&mut buffer), timeout).await {
                Either::First(result) => {
                    match result {
                        Ok(_) => {
                            // Process the byte
                            if self.process_byte(buffer[0]) {
                                // Valid message processed
                                updated = true;
                                return Ok(true);
                            }

                            // Yield to other tasks
                            yield_now().await;

                            // Reset timeout for next byte
                            timeout = Timer::after(Duration::from_millis(10));
                        }
                        Err(_) => return Err(()),
                    }
                }
                Either::Second(_) => {
                    // Timeout occurred
                    return Ok(updated);
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

                if self.payload_length > 0 {
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
                // Skip checksum validation for simplicity
                self.parse_state = UbxParseState::ChecksumB;
            }
            UbxParseState::ChecksumB => {
                // Reset parse state
                self.parse_state = UbxParseState::SyncChar1;

                // Process message
                if self.msg_class == UBX_CLASS_NAV && self.msg_id == UBX_NAV_PVT {
                    self.process_nav_pvt();
                    return true;
                }
            }
        }

        false
    }

    /// Process NAV-PVT message and update internal state
    fn process_nav_pvt(&mut self) {
        if self.buffer_idx < 92 {
            return; // Not enough data
        }

        // Extract time
        let year = u16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        let month = self.buffer[6];
        let day = self.buffer[7];
        let hour = self.buffer[8];
        let minute = self.buffer[9];
        let second = self.buffer[10];

        // Extract fix type
        let fix_type = self.buffer[20];
        let satellites = self.buffer[23];

        // Extract position
        let lon_bits = u32::from_le_bytes([
            self.buffer[24],
            self.buffer[25],
            self.buffer[26],
            self.buffer[27],
        ]);
        let longitude = (lon_bits as f64) * 1e-7;

        let lat_bits = u32::from_le_bytes([
            self.buffer[28],
            self.buffer[29],
            self.buffer[30],
            self.buffer[31],
        ]);
        let latitude = (lat_bits as f64) * 1e-7;

        let alt_bits = i32::from_le_bytes([
            self.buffer[36],
            self.buffer[37],
            self.buffer[38],
            self.buffer[39],
        ]);
        let altitude = (alt_bits as f64) * 1e-3; // mm to m

        // Extract accuracy
        let h_acc = u32::from_le_bytes([
            self.buffer[40],
            self.buffer[41],
            self.buffer[42],
            self.buffer[43],
        ]);
        let acc = (h_acc as f32) * 1e-3; // mm to m

        // Extract velocity data
        let speed_bits = u32::from_le_bytes([
            self.buffer[60],
            self.buffer[61],
            self.buffer[62],
            self.buffer[63],
        ]);
        let speed = (speed_bits as f32) * 1e-3; // mm/s to m/s

        let heading_bits = u32::from_le_bytes([
            self.buffer[64],
            self.buffer[65],
            self.buffer[66],
            self.buffer[67],
        ]);
        let course = (heading_bits as f32) * 1e-5; // deg * 1e-5 to deg

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
            accuracy: acc * 0.1, // Approximate velocity accuracy
        };

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

        self.status.has_fix = fix_type > 0;
        self.status.satellites_used = satellites;
        self.status.satellites_visible = satellites;
        self.status.fix_type = match fix_type {
            0 => GpsFixType::NoFix,
            1 => GpsFixType::Fix2D,
            2 => GpsFixType::Fix3D,
            3 => GpsFixType::DGps,
            4 => GpsFixType::Rtk,
            5 => GpsFixType::RtkFloat,
            _ => GpsFixType::NoFix,
        };
        self.status.hdop = acc / 5.0; // Approximate HDOP from horizontal accuracy

        self.last_update = embassy_time::Instant::now().as_millis();
    }

    /// Send a UBX protocol message
    pub async fn send_ubx_message(&mut self, class: u8, id: u8, payload: &[u8]) -> Result<(), ()> {
        // Calculate message size
        let msg_len = 8 + payload.len(); // 2 sync + 1 class + 1 id + 2 length + payload + 2 checksum

        // Create message buffer
        let mut message = [0u8; 128]; // Max payload assumed to be 120 bytes
        if msg_len > message.len() {
            return Err(());
        }

        // Build message
        message[0] = UBX_SYNC_CHAR_1;
        message[1] = UBX_SYNC_CHAR_2;
        message[2] = class;
        message[3] = id;
        message[4] = (payload.len() & 0xFF) as u8;
        message[5] = ((payload.len() >> 8) & 0xFF) as u8;

        // Copy payload
        for (i, &byte) in payload.iter().enumerate() {
            message[6 + i] = byte;
        }

        // Calculate and add checksum
        let (ck_a, ck_b) = Self::calculate_checksum(&message[2..6 + payload.len()]);
        message[6 + payload.len()] = ck_a;
        message[7 + payload.len()] = ck_b;

        // Send message
        match self.uart.write(&message[..msg_len]).await {
            Ok(_) => Ok(()),
            Err(_) => Err(()),
        }
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
}

impl<'a> GpsSensor for UbloxGps<'a> {
    /// Initialize GPS module
    fn init(&mut self) -> bool {
        // Synchronous implementation
        true
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

    /// Calibrate the GPS module
    fn calibrate(&mut self) -> bool {
        true // Placeholder
    }

    /// Reset the GPS module
    fn reset(&mut self) -> bool {
        true // Placeholder
    }

    /// Set the update rate in milliseconds
    fn set_update_rate(&mut self, rate_ms: u32) -> bool {
        true // Placeholder
    }

    /// Set the power mode of the GPS module
    fn set_power_mode(&mut self, _power_on: bool) -> bool {
        true // Placeholder
    }

    /// Get satellite information
    fn get_satellite_info(&self, satellites: &mut [SatelliteInfo; 32]) -> u8 {
        0 // Placeholder: returning 0 satellites
    }

    /// Get time to first fix
    fn get_time_to_first_fix(&self) -> Option<f32> {
        None // Placeholder
    }

    /// Set the minimum number of satellites required for navigation
    fn set_min_satellites(&mut self, _count: u8) -> bool {
        true // Placeholder
    }

    /// Get horizontal dilution of precision (HDOP)
    fn get_hdop(&self) -> f32 {
        0.0 // Placeholder
    }

    /// Get vertical dilution of precision (VDOP)
    fn get_vdop(&self) -> f32 {
        0.0 // Placeholder
    }

    /// Set SBAS (Satellite Based Augmentation System) enabled
    fn set_sbas_enabled(&mut self, _enabled: bool) -> bool {
        true // Placeholder
    }

    /// Configure GNSS settings
    fn configure_gnss(&mut self, config: GnssConfig) -> bool {
        true // Placeholder
    }

    /// Send an NMEA command
    fn send_nmea_command(&mut self, _command: &str) -> bool {
        true // Placeholder
    }

    /// Parse an NMEA sentence
    fn parse_nmea(&mut self, data: &str) -> bool {
        true // Placeholder
    }

    /// Get heading accuracy
    fn get_heading_accuracy(&self) -> f32 {
        0.0 // Placeholder
    }

    /// Set dead reckoning enabled
    fn set_dead_reckoning(&mut self, _enabled: bool) -> bool {
        true // Placeholder
    }
}
