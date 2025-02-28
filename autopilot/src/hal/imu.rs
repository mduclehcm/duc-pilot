pub trait Imu6Dof {
    fn read_accelerometer(&self) -> [f32; 3];
    fn read_gyroscope(&self) -> [f32; 3];
}
