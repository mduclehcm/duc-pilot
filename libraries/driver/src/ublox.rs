use embassy_stm32::peripherals::USART1;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};
use core::fmt::Write;
use heapless::Vec;
use hal::{GpsPosition, GpsVelocity, GpsTime};

/// NMEA sentence buffer size
const NMEA_BUFFER_SIZE: usize = 128;

/// Represents a uBlox GPS receiver connected via UART
pub struct UbloxGps<'d> {
    uart: Uart<'d, USART1>,
    buffer: [u8; NMEA_BUFFER_SIZE],
    buffer_idx: usize,
    
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

impl<'d> UbloxGps<'d> {
    /// Creates a new UbloxGps instance
    pub fn new(
        uart: USART1,
        tx_dma: embassy_stm32::peripherals::DMA1_CH5,
        rx_dma: embassy_stm32::peripherals::DMA1_CH6,
        baudrate: u32
    ) -> Self {
        let config = Config::default();
        let uart = Uart::new(uart, tx_dma, rx_dma, Hertz(baudrate), config);
        
        Self {
            uart,
            buffer: [0; NMEA_BUFFER_SIZE],
            buffer_idx: 0,
            
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
    
    /// Initialize the GPS with default configuration
    pub async fn init(&mut self) -> Result<(), ()> {
        // Wait for the GPS module to boot
        Timer::after(Duration::from_millis(1000)).await;
        
        // Configure update rate to 5Hz
        let config_rate = "$PUBX,40,GLL,0,0,0,0*5C\r\n";
        let _ = self.uart.write(config_rate.as_bytes()).await;
        
        Timer::after(Duration::from_millis(100)).await;
        
        // Enable only the messages we need
        let enable_gga = "$PUBX,40,GGA,0,1,0,0*5B\r\n";
        let _ = self.uart.write(enable_gga.as_bytes()).await;
        
        Timer::after(Duration::from_millis(100)).await;
        
        let enable_rmc = "$PUBX,40,RMC,0,1,0,0*46\r\n";
        let _ = self.uart.write(enable_rmc.as_bytes()).await;
        
        Ok(())
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
    
    /// Reads and processes data from the GPS
    pub async fn update(&mut self, timeout_ms: u64) -> Result<bool, ()> {
        let timeout = Timer::after(Duration::from_millis(timeout_ms));
        
        // Reset buffer
        self.buffer_idx = 0;
        
        // Read until we get a complete NMEA sentence or timeout
        loop {
            let read_future = self.read_byte();
            
            match select(read_future, timeout).await {
                Either::First(result) => {
                    match result {
                        Ok(byte) => {
                            if byte == b'\n' {
                                // Complete NMEA sentence received
                                return self.parse_nmea_sentence();
                            } else if self.buffer_idx < NMEA_BUFFER_SIZE - 1 {
                                self.buffer[self.buffer_idx] = byte;
                                self.buffer_idx += 1;
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
    
    /// Read a single byte from UART
    async fn read_byte(&mut self) -> Result<u8, ()> {
        let mut byte = [0u8; 1];
        match self.uart.read(&mut byte).await {
            Ok(_) => Ok(byte[0]),
            Err(_) => Err(()),
        }
    }
    
    /// Parse a complete NMEA sentence
    fn parse_nmea_sentence(&mut self) -> Result<bool, ()> {
        let sentence = core::str::from_utf8(&self.buffer[0..self.buffer_idx]);
        if sentence.is_err() {
            return Ok(false);
        }
        
        let sentence = sentence.unwrap();
        
        // Check if this is a GGA or RMC sentence
        if sentence.starts_with("$GPGGA") {
            self.parse_gga(sentence);
            return Ok(true);
        } else if sentence.starts_with("$GPRMC") {
            self.parse_rmc(sentence);
            return Ok(true);
        }
        
        Ok(false)
    }
    
    /// Parse GGA sentence (position, altitude, satellites)
    fn parse_gga(&mut self, sentence: &str) -> bool {
        let fields: heapless::Vec<&str, 16> = sentence.split(',').collect();
        if fields.len() < 15 {
            return false;
        }
        
        // Parse latitude
        if let Some(lat_str) = fields.get(2) {
            if let Some(lat_dir) = fields.get(3) {
                if let Ok(lat_value) = parse_latitude(lat_str, lat_dir) {
                    self.latitude = lat_value as f64;
                }
            }
        }
        
        // Parse longitude
        if let Some(lon_str) = fields.get(4) {
            if let Some(lon_dir) = fields.get(5) {
                if let Ok(lon_value) = parse_longitude(lon_str, lon_dir) {
                    self.longitude = lon_value as f64;
                }
            }
        }
        
        // Parse fix type
        if let Some(fix_str) = fields.get(6) {
            if let Ok(fix) = fix_str.parse::<u8>() {
                self.fix_type = fix;
            }
        }
        
        // Parse number of satellites
        if let Some(sat_str) = fields.get(7) {
            if let Ok(sats) = sat_str.parse::<u8>() {
                self.satellites = sats;
            }
        }
        
        // Parse HDOP
        if let Some(hdop_str) = fields.get(8) {
            if let Ok(hdop) = hdop_str.parse::<f32>() {
                self.hdop = hdop;
            }
        }
        
        // Parse altitude
        if let Some(alt_str) = fields.get(9) {
            if let Ok(alt) = alt_str.parse::<f32>() {
                self.altitude = alt as f64;
            }
        }
        
        true
    }
    
    /// Parse RMC sentence (time, date, speed, course)
    fn parse_rmc(&mut self, sentence: &str) -> bool {
        let fields: heapless::Vec<&str, 16> = sentence.split(',').collect();
        if fields.len() < 12 {
            return false;
        }
        
        // Parse time
        if let Some(time_str) = fields.get(1) {
            if time_str.len() >= 6 {
                if let Ok(hr) = time_str[0..2].parse::<u8>() {
                    self.hour = hr;
                }
                if let Ok(min) = time_str[2..4].parse::<u8>() {
                    self.minute = min;
                }
                if let Ok(sec) = time_str[4..6].parse::<u8>() {
                    self.second = sec;
                }
            }
        }
        
        // Parse date
        if let Some(date_str) = fields.get(9) {
            if date_str.len() >= 6 {
                if let Ok(day) = date_str[0..2].parse::<u8>() {
                    self.day = day;
                }
                if let Ok(month) = date_str[2..4].parse::<u8>() {
                    self.month = month;
                }
                if let Ok(yr) = date_str[4..6].parse::<u16>() {
                    self.year = 2000 + yr; // Assumes 21st century
                }
            }
        }
        
        // Parse speed (knots -> m/s)
        if let Some(speed_str) = fields.get(7) {
            if let Ok(speed_knots) = speed_str.parse::<f32>() {
                // Convert knots to m/s (1 knot = 0.51444 m/s)
                self.speed = speed_knots * 0.51444;
            }
        }
        
        // Parse course/heading
        if let Some(course_str) = fields.get(8) {
            if let Ok(course) = course_str.parse::<f32>() {
                self.course = course;
            }
        }
        
        true
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