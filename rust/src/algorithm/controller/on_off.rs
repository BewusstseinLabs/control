use std::{
    cmp::PartialOrd,
    ops::{ Add, Sub }
};

pub struct OnOffController<T>
where
    T: PartialOrd
        + Copy
        + Add<Output = T>
        + Sub<Output = T>
{
    hysteresis: T,
    output_on: bool,
}

impl<T> OnOffController<T>
where
    T: PartialOrd
        + Copy
        + Add<Output = T>
        + Sub<Output = T>
{
    pub fn new( hysteresis: T ) -> Self {
        Self {
            hysteresis,
            output_on: false,
        }
    }

    pub fn step( &mut self, target: T, state: T ) -> bool {
        if state < target - self.hysteresis {
            self.output_on = true;
        } else if state > target + self.hysteresis {
            self.output_on = false;
        }
        self.output_on
    }
}