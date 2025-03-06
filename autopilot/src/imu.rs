#[allow(dead_code)]
pub trait IMU {
    fn read_accelerometer(&self) -> [f32; 3];
    fn read_gyroscope(&self) -> [f32; 3];
    fn read_magnetometer(&self) -> [f32; 3];
}
