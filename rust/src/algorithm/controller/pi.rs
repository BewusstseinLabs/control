// Copyright 2024 Bewusstsein Labs

use std::ops::{ Add, Mul, Div };

use crate::algorithm::controller::{
    proportional::ProportionalController as PController,
    integral::IntegralController as IController
};

pub struct PIController<T>
where
    T: Default
        + Add<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    p_ctrl: PController<T>,
    i_ctrl: IController<T>,
    bias: T,
}

impl<T> PIController<T>
where
    T: Default
        + Add<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    pub fn new( p_gain: T, i_gain: T, bias: T ) -> Self {
        Self {
            p_ctrl: PController::new( p_gain ),
            i_ctrl: IController::new( i_gain ),
            bias,
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        let p = self.p_ctrl.step( error );
        let i = self.i_ctrl.step( error, dt );
        p + i + self.bias
    }
}