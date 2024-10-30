// Copyright 2024 Bewusstsein Labs

use std::ops::{ Add, Mul };

pub struct IntegralController<T>
where
    T: Default
        + Copy
        + Add<Output = T>
        + Mul<Output = T>
{
    integral_prior: T,
    gain: T,
}

impl<T> IntegralController<T>
where
    T: Default
        + Copy
        + Add<Output = T>
        + Mul<Output = T>
{
    pub fn new( gain: T ) -> Self {
        Self {
            integral_prior: T::default(),
            gain,
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        self.integral_prior = self.integral_prior + error * dt;
        self.gain * self.integral_prior
    }
}