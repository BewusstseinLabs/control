// Copyright 2024 Bewusstsein Labs

use std::ops::{ Add, Sub, Mul, Div };

use crate::algorithm::controller::{
    proportional::ProportionalController as PController,
    derivative::DerivativeController as DController
};

pub struct PDController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy
{
    p_ctrl: PController<T>,
    d_ctrl: DController<T>,
    bias: T,
}

impl<T> PDController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy
{
    pub fn new( p_gain: T, d_gain: T, bias: T ) -> Self {
        Self {
            p_ctrl: PController::new( p_gain ),
            d_ctrl: DController::new( d_gain ),
            bias,
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        let p = self.p_ctrl.step( error );
        let d = self.d_ctrl.step( error, dt );
        p + d + self.bias
    }
}