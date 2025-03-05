/// Hardware Manager interface
use crate::gps::GpsSensor;
use crate::imu::ImuSensor;
use crate::baro::BaroSensor;
use crate::rc_input::RcInput;

/// Hardware Manager trait that provides access to all hardware systems
pub trait HardwareManager {
    /// Get the GPS sensor implementation
    fn gps(&self) -> &dyn GpsSensor;
    
    /// Get the IMU sensor implementation
    fn imu(&self) -> &dyn ImuSensor;
    
    /// Get the barometer sensor implementation
    fn baro(&self) -> &dyn BaroSensor;
    
    /// Get the RC input implementation
    fn rc_input(&self) -> &dyn RcInput;
    
    /// Initialize all hardware components
    fn init_all(&mut self) -> bool;
}

/// Factory trait for creating hardware implementations
pub trait HardwareFactory {
    type Manager: HardwareManager;
    
    /// Create a new hardware manager instance
    fn create() -> Self::Manager;
} 