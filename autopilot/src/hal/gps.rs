pub trait GPS {
    fn read(&self) -> [f32; 3];
}
