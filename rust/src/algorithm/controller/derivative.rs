// Copyright 2024 Bewusstsein Labs

use std::ops::{ Sub, Mul, Div };

pub struct DerivativeController<T>
where
    T: Default
        + Copy
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
{
    error_prior: T,
    gain: T,
}

impl<T> DerivativeController<T>
where
    T: Default
        + Copy
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
{
    pub fn new( gain: T ) -> Self {
        Self {
            error_prior: T::default(),
            gain,
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        let derivative = ( error - self.error_prior ) / dt;
        self.error_prior = error;
        self.gain * derivative
    }
}