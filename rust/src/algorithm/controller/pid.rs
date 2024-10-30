// Copyright 2024 Bewusstsein Labs

use std::ops::{ Add, Sub, Mul, Div };

use crate::algorithm::controller::{
    proportional::ProportionalController as PController,
    integral::IntegralController as IController,
    derivative::DerivativeController as DController
};

pub struct PIDController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    p_ctrl: PController<T>,
    i_ctrl: IController<T>,
    d_ctrl: DController<T>,
}

impl<T> PIDController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    pub fn new( p_gain: T, i_gain: T, d_gain: T ) -> Self {
        Self {
            p_ctrl: PController::new( p_gain ),
            i_ctrl: IController::new( i_gain ),
            d_ctrl: DController::new( d_gain ),
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        let p = self.p_ctrl.step( error );
        let i = self.i_ctrl.step( error, dt );
        let d = self.d_ctrl.step( error, dt );
        p + i + d
    }
}