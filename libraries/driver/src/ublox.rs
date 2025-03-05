use embassy_stm32::usart::{Config, Uart, Instance};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use core::fmt::Write;
use heapless::Vec;
use hal::{GpsPosition, GpsVelocity, GpsTime};

/// NMEA sentence buffer size
const NMEA_BUFFER_SIZE: usize = 128;

/// Default baud rate for uBlox GPS
const DEFAULT_BAUDRATE: u32 = 9600;
/// High baud rate for faster communication
const HIGH_BAUDRATE: u32 = 115200;

/// UBX Protocol constants
const UBX_SYNC_CHAR_1: u8 = 0xB5;
const UBX_SYNC_CHAR_2: u8 = 0x62;

/// UBX Message Classes
const UBX_CLASS_NAV: u8 = 0x01;
const UBX_CLASS_CFG: u8 = 0x06;
const UBX_CLASS_MON: u8 = 0x0A;

/// UBX Message IDs
const UBX_CFG_PRT: u8 = 0x00;    // Ports Configuration
const UBX_CFG_MSG: u8 = 0x01;    // Message Configuration
const UBX_CFG_RATE: u8 = 0x08;   // Navigation Rate
const UBX_CFG_NAV5: u8 = 0x24;   // Navigation Engine Settings
const UBX_NAV_PVT: u8 = 0x07;    // Position, Velocity, Time Solution
const UBX_NAV_SAT: u8 = 0x35;    // Satellite Information
const UBX_MON_VER: u8 = 0x04;    // Receiver/Software Version

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
            update_rate_ms: 200,  // 5Hz
            min_satellites: 0,    // Use module default
            min_elevation_deg: 5,
            pdop_mask: 250,       // 25.0
        }
    }
}

/// Represents a uBlox GPS receiver connected via UART
pub struct UbloxGps<'d, T: Instance> {
    uart: Uart<'d, T>,
    buffer: [u8; NMEA_BUFFER_SIZE],
    buffer_idx: usize,
    
    // Configuration
    config: GpsConfig,
    
    // Position data
    latitude: f64,
    longitude: f64,
    altitude: f64,
    
    // Velocity data
    speed: f32,
    course: f32,
    
    // Fix data
    fix_type: u8,
    satellites: u8,
    hdop: f32,
    
    // Time data
    hour: u8,
    minute: u8,
    second: u8,
    day: u8,
    month: u8,
    year: u16,
}

impl<'d, T: Instance> UbloxGps<'d, T> {
    /// Creates a new UbloxGps instance with default configuration except baudrate
    pub fn new_with_baudrate(
        uart: T,
        tx_dma: impl embassy_stm32::dma::Instance,
        rx_dma: impl embassy_stm32::dma::Instance,
        baudrate: u32,
    ) -> Self {
        let mut config = GpsConfig::default();
        config.baudrate = baudrate;
        Self::new(uart, tx_dma, rx_dma, config)
    }
    
    /// Creates a new UbloxGps instance
    pub fn new(
        uart: T,
        tx_dma: impl embassy_stm32::dma::Instance,
        rx_dma: impl embassy_stm32::dma::Instance,
        config: GpsConfig,
    ) -> Self {
        let uart_config = Config::default();
        let uart = Uart::new(uart, tx_dma, rx_dma, Hertz(config.baudrate), uart_config);
        
        Self {
            uart,
            buffer: [0; NMEA_BUFFER_SIZE],
            buffer_idx: 0,
            
            config,
            
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            
            speed: 0.0,
            course: 0.0,
            
            fix_type: 0,
            satellites: 0,
            hdop: 0.0,
            
            hour: 0,
            minute: 0,
            second: 0,
            day: 0,
            month: 0,
            year: 0,
        }
    }
    
    /// Initialize the GPS with auto-configuration prioritizing UBX protocol
    pub async fn init(&mut self) -> Result<(), ()> {
        // Wait for the GPS module to boot
        Timer::after(Duration::from_millis(1000)).await;
        
        // First try to communicate at default baud rate
        if !self.test_communication().await {
            // If failed, try to configure at default baud rate
            self.configure_baud_rate(DEFAULT_BAUDRATE, HIGH_BAUDRATE).await?;
        }
        
        // Configure to use UBX protocol
        self.configure_ubx_protocol().await?;
        
        // Configure navigation settings
        self.configure_navigation_settings().await?;
        
        // Configure message rates
        self.configure_message_rates().await?;
        
        Ok(())
    }
    
    /// Test GPS communication using UBX protocol
    async fn test_communication(&mut self) -> bool {
        // Send UBX poll version message to test communication
        if let Err(_) = self.send_ubx_message(UBX_CLASS_MON, UBX_MON_VER, &[]).await {
            return false;
        }
        
        // Wait for response with timeout
        if self.wait_for_ubx_ack(UBX_CLASS_MON, UBX_MON_VER, 1000).await.is_ok() {
            return true;
        }
        
        false
    }
    
    /// Configure GPS to use UBX protocol
    async fn configure_ubx_protocol(&mut self) -> Result<(), ()> {
        // Configure UART port to use UBX protocol only
        // UBX-CFG-PRT message
        let port_config = [
            0x01,       // Port ID (1=UART1)
            0x00,       // Reserved
            0x00, 0x00, // txReady config
            0xD0, 0x08, 0x00, 0x00, // UART mode (8N1)
            0x00, 0xC2, 0x01, 0x00, // Baud rate (115200)
            0x01, 0x00, // Input protocols (UBX only)
            0x01, 0x00, // Output protocols (UBX only)
            0x00, 0x00, // Flags
            0x00, 0x00, // Reserved
        ];
        
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_CFG_PRT, &port_config).await {
            return Err(());
        }
        
        // Wait for acknowledgment
        self.wait_for_ubx_ack(UBX_CLASS_CFG, UBX_CFG_PRT, 1000).await?;
        
        // Reconfigure UART with new baud rate
        Timer::after(Duration::from_millis(100)).await;
        let config = Config::default();
        self.uart = Uart::new(self.uart.into_inner(), self.uart.tx_dma(), self.uart.rx_dma(), Hertz(HIGH_BAUDRATE), config);
        
        Ok(())
    }
    
    /// Configure navigation settings (dynamic model, min elevation, etc.)
    async fn configure_navigation_settings(&mut self) -> Result<(), ()> {
        // UBX-CFG-NAV5 message
        // Set dynamic model to Airborne with <2g acceleration (6)
        let nav_config = [
            0x01, 0x00, // Parameters bitmask
            0x06,       // Dynamic model (6=Airborne <2g)
            0x03,       // Fix mode (3=Auto 2D/3D)
            0x00, 0x00, 0x00, 0x00, // Fixed altitude
            0x00, 0x00, 0x00, 0x00, // Fixed altitude variance
            self.config.min_elevation_deg, // Min elevation for fix (degrees)
            0x00,       // Reserved
            (self.config.pdop_mask & 0xFF) as u8, (self.config.pdop_mask >> 8) as u8, // Position DOP mask
            0xFA, 0x00, // Time DOP mask
            0x64, 0x00, // Position accuracy mask
            0x2C, 0x01, // Time accuracy mask
            0x00,       // Static hold threshold
            0x00,       // DGNSS timeout
            self.config.min_satellites, // Number of satellites for navigation
            0x00,       // Reserved
            0x00, 0x00, // Reserved
            0x00, 0x00, // Reserved
            0x00, 0x00, // Reserved
        ];
        
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_CFG_NAV5, &nav_config).await {
            return Err(());
        }
        
        // Wait for acknowledgment
        self.wait_for_ubx_ack(UBX_CLASS_CFG, UBX_CFG_NAV5, 1000).await?;
        
        // Configure navigation rate (UBX-CFG-RATE)
        let rate_config = [
            (self.config.update_rate_ms & 0xFF) as u8, (self.config.update_rate_ms >> 8) as u8, // Measurement rate in ms
            0x01, 0x00, // Navigation rate in cycles (1)
            0x01, 0x00, // Time reference (1=GPS time)
        ];
        
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_CFG_RATE, &rate_config).await {
            return Err(());
        }
        
        // Wait for acknowledgment
        self.wait_for_ubx_ack(UBX_CLASS_CFG, UBX_CFG_RATE, 1000).await?;
        
        Ok(())
    }
    
    /// Configure message rates
    async fn configure_message_rates(&mut self) -> Result<(), ()> {
        // Enable UBX-NAV-PVT message (Position, Velocity, Time)
        let pvt_config = [
            UBX_CLASS_NAV, // Message class
            UBX_NAV_PVT,   // Message ID
            0x01,          // Rate on current port (1Hz)
        ];
        
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_CFG_MSG, &pvt_config).await {
            return Err(());
        }
        
        // Wait for acknowledgment
        self.wait_for_ubx_ack(UBX_CLASS_CFG, UBX_CFG_MSG, 1000).await?;
        
        // Enable UBX-NAV-SAT message (Satellite Information)
        let sat_config = [
            UBX_CLASS_NAV, // Message class
            UBX_NAV_SAT,   // Message ID
            0x01,          // Rate on current port (1Hz)
        ];
        
        if let Err(_) = self.send_ubx_message(UBX_CLASS_CFG, UBX_CFG_MSG, &sat_config).await {
            return Err(());
        }
        
        // Wait for acknowledgment
        self.wait_for_ubx_ack(UBX_CLASS_CFG, UBX_CFG_MSG, 1000).await?;
        
        Ok(())
    }
    
    /// Send UBX protocol message
    async fn send_ubx_message(&mut self, class: u8, id: u8, payload: &[u8]) -> Result<(), ()> {
        let mut message = heapless::Vec::<u8, 128>::new();
        
        // Add header
        message.push(UBX_SYNC_CHAR_1).unwrap();
        message.push(UBX_SYNC_CHAR_2).unwrap();
        message.push(class).unwrap();
        message.push(id).unwrap();
        
        // Add length (little endian)
        let len = payload.len() as u16;
        message.push((len & 0xFF) as u8).unwrap();
        message.push((len >> 8) as u8).unwrap();
        
        // Add payload
        for &byte in payload {
            message.push(byte).unwrap();
        }
        
        // Calculate and add checksum
        let (ck_a, ck_b) = self.calculate_ubx_checksum(&message[2..]);
        message.push(ck_a).unwrap();
        message.push(ck_b).unwrap();
        
        // Send message
        match self.uart.write(&message).await {
            Ok(_) => Ok(()),
            Err(_) => Err(()),
        }
    }
    
    /// Calculate UBX checksum
    fn calculate_ubx_checksum(&self, data: &[u8]) -> (u8, u8) {
        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;
        
        for &byte in data {
            ck_a = ck_a.wrapping_add(byte);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        
        (ck_a, ck_b)
    }
    
    /// Wait for UBX acknowledgment with timeout
    async fn wait_for_ubx_ack(&mut self, class: u8, id: u8, timeout_ms: u64) -> Result<(), ()> {
        let timeout = Timer::after(Duration::from_millis(timeout_ms));
        
        // Reset buffer
        self.buffer_idx = 0;
        
        // State machine for UBX ACK parsing
        let mut state = 0;
        
        loop {
            let read_future = self.read_byte();
            
            match select(read_future, timeout).await {
                Either::First(result) => {
                    match result {
                        Ok(byte) => {
                            match state {
                                0 => if byte == UBX_SYNC_CHAR_1 { state = 1; },
                                1 => if byte == UBX_SYNC_CHAR_2 { state = 2; } else { state = 0; },
                                2 => if byte == 0x05 { state = 3; } else { state = 0; }, // ACK class
                                3 => if byte == 0x01 { state = 4; } else { state = 0; }, // ACK-ACK ID
                                4 => if byte == 0x02 { state = 5; } else { state = 0; }, // Payload length (2)
                                5 => if byte == 0x00 { state = 6; } else { state = 0; }, // Payload length (0)
                                6 => if byte == class { state = 7; } else { state = 0; }, // ACK class
                                7 => if byte == id { return Ok(()); } else { state = 0; }, // ACK ID
                                _ => state = 0,
                            }
                        }
                        Err(_) => return Err(()),
                    }
                }
                Either::Second(_) => {
                    // Timeout occurred
                    return Err(());
                }
            }
        }
    }
    
    /// Read and process UBX data
    pub async fn update(&mut self, timeout_ms: u64) -> Result<bool, ()> {
        let timeout = Timer::after(Duration::from_millis(timeout_ms));
        
        // Reset buffer
        self.buffer_idx = 0;
        
        // State machine for UBX message parsing
        let mut state = 0;
        let mut msg_class = 0;
        let mut msg_id = 0;
        let mut payload_length = 0;
        let mut payload_bytes_read = 0;
        
        loop {
            let read_future = self.read_byte();
            
            match select(read_future, timeout).await {
                Either::First(result) => {
                    match result {
                        Ok(byte) => {
                            match state {
                                0 => if byte == UBX_SYNC_CHAR_1 { state = 1; },
                                1 => if byte == UBX_SYNC_CHAR_2 { state = 2; } else { state = 0; },
                                2 => { msg_class = byte; state = 3; },
                                3 => { msg_id = byte; state = 4; },
                                4 => { payload_length = byte as u16; state = 5; },
                                5 => { 
                                    payload_length |= (byte as u16) << 8; 
                                    payload_bytes_read = 0;
                                    state = 6; 
                                },
                                6 => {
                                    // Store payload in buffer
                                    if self.buffer_idx < NMEA_BUFFER_SIZE {
                                        self.buffer[self.buffer_idx] = byte;
                                        self.buffer_idx += 1;
                                    }
                                    
                                    payload_bytes_read += 1;
                                    if payload_bytes_read >= payload_length {
                                        state = 7; // Move to checksum
                                    }
                                },
                                7 => { /* skip checksum A */ state = 8; },
                                8 => { 
                                    // Message complete, process it
                                    if msg_class == UBX_CLASS_NAV && msg_id == UBX_NAV_PVT {
                                        self.parse_ubx_nav_pvt();
                                        return Ok(true);
                                    }
                                    state = 0; // Reset state machine
                                },
                                _ => state = 0,
                            }
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
    
    /// Parse UBX-NAV-PVT message
    fn parse_ubx_nav_pvt(&mut self) {
        if self.buffer_idx < 92 {
            return; // Not enough data
        }
        
        // Extract time fields
        self.year = u16::from_le_bytes([self.buffer[4], self.buffer[5]]);
        self.month = self.buffer[6];
        self.day = self.buffer[7];
        self.hour = self.buffer[8];
        self.minute = self.buffer[9];
        self.second = self.buffer[10];
        
        // Extract fix type
        self.fix_type = self.buffer[20];
        
        // Extract number of satellites
        self.satellites = self.buffer[23];
        
        // Extract position
        let lon_bits = u32::from_le_bytes([
            self.buffer[24], self.buffer[25], self.buffer[26], self.buffer[27]
        ]);
        self.longitude = (lon_bits as f64) * 1e-7;
        
        let lat_bits = u32::from_le_bytes([
            self.buffer[28], self.buffer[29], self.buffer[30], self.buffer[31]
        ]);
        self.latitude = (lat_bits as f64) * 1e-7;
        
        let alt_bits = u32::from_le_bytes([
            self.buffer[36], self.buffer[37], self.buffer[38], self.buffer[39]
        ]);
        self.altitude = (alt_bits as f64) * 1e-3; // mm to m
        
        // Extract horizontal accuracy (as hdop approximation)
        let h_acc = u32::from_le_bytes([
            self.buffer[40], self.buffer[41], self.buffer[42], self.buffer[43]
        ]);
        self.hdop = (h_acc as f32) * 1e-3; // mm to m
        
        // Extract speed
        let speed_bits = u32::from_le_bytes([
            self.buffer[60], self.buffer[61], self.buffer[62], self.buffer[63]
        ]);
        self.speed = (speed_bits as f32) * 1e-3; // mm/s to m/s
        
        // Extract heading
        let heading_bits = u32::from_le_bytes([
            self.buffer[64], self.buffer[65], self.buffer[66], self.buffer[67]
        ]);
        self.course = (heading_bits as f32) * 1e-5; // deg * 1e-5 to deg
    }
    
    /// Read a single byte from UART
    async fn read_byte(&mut self) -> Result<u8, ()> {
        let mut byte = [0u8; 1];
        match self.uart.read(&mut byte).await {
            Ok(_) => Ok(byte[0]),
            Err(_) => Err(()),
        }
    }
    
    /// Get the current GPS position data
    pub fn get_position(&self) -> GpsPosition {
        GpsPosition {
            latitude: self.latitude,
            longitude: self.longitude,
            altitude: self.altitude,
            accuracy: self.hdop * 5.0, // Approximate position accuracy from HDOP
            satellites: self.satellites,
        }
    }
    
    /// Get the current GPS velocity data
    pub fn get_velocity(&self) -> GpsVelocity {
        GpsVelocity {
            speed: self.speed,
            course: self.course,
            vertical: 0.0, // uBlox NMEA doesn't provide vertical velocity in basic messages
            accuracy: self.hdop * 0.5, // Approximate velocity accuracy from HDOP
        }
    }
    
    /// Get the current GPS time data
    pub fn get_time(&self) -> GpsTime {
        GpsTime {
            year: self.year,
            month: self.month,
            day: self.day,
            hour: self.hour,
            minute: self.minute,
            second: self.second,
            millisecond: 0, // NMEA doesn't provide millisecond precision
            valid: self.fix_type > 0,
        }
    }
    
    /// Check if GPS has a valid fix
    pub fn has_fix(&self) -> bool {
        self.fix_type > 0
    }
    
    /// Get number of satellites
    pub fn satellites(&self) -> u8 {
        self.satellites
    }
    
    /// Get horizontal dilution of precision
    pub fn hdop(&self) -> f32 {
        self.hdop
    }
}

/// Parse latitude from NMEA format
fn parse_latitude(lat_str: &str, dir: &str) -> Result<f32, ()> {
    if lat_str.is_empty() {
        return Err(());
    }
    
    // NMEA format: ddmm.mmmm
    // We need to convert to decimal degrees
    let degrees = match lat_str[0..2].parse::<f32>() {
        Ok(d) => d,
        Err(_) => return Err(()),
    };
    
    let minutes = match lat_str[2..].parse::<f32>() {
        Ok(m) => m,
        Err(_) => return Err(()),
    };
    
    let mut latitude = degrees + (minutes / 60.0);
    
    // Apply direction
    if dir == "S" {
        latitude = -latitude;
    }
    
    Ok(latitude)
}

/// Parse longitude from NMEA format
fn parse_longitude(lon_str: &str, dir: &str) -> Result<f32, ()> {
    if lon_str.is_empty() {
        return Err(());
    }
    
    // NMEA format: dddmm.mmmm
    // We need to convert to decimal degrees
    let degrees = match lon_str[0..3].parse::<f32>() {
        Ok(d) => d,
        Err(_) => return Err(()),
    };
    
    let minutes = match lon_str[3..].parse::<f32>() {
        Ok(m) => m,
        Err(_) => return Err(()),
    };
    
    let mut longitude = degrees + (minutes / 60.0);
    
    // Apply direction
    if dir == "W" {
        longitude = -longitude;
    }
    
    Ok(longitude)
} 