use crate::hal::{gps::GPS, imu::Imu6Dof};

pub trait Board
where
    Self: Sized,
{
    fn name(&self) -> &str;

    fn split_resources(self) -> ResourceSplitter;
}

pub struct ResourceSplitter {
    pub imu: Option<Box<dyn Imu6Dof>>,
    pub gps: Option<Box<dyn GPS>>,
}
