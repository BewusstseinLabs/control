// Copyright 2024 Bewusstsein Labs

use std::ops::Mul;

pub struct ProportionalController<T>
where
    T: Copy
        + Mul<Output = T>
{
        gain: T,
}

impl<T> ProportionalController<T>
where
    T: Copy
        + Mul<Output = T>
{
    pub fn new( gain: T ) -> Self {
        Self { gain }
    }

    pub fn step( &self, error: T ) -> T {
        self.gain * error
    }
}