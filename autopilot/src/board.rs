pub trait Board
where
    Self: Sized,
{
    fn get_name(&self) -> &str;

    fn split_resources(self) -> Resources;
}

pub struct Resources {}
