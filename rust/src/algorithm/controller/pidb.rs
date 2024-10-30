// Copyright 2024 Bewusstsein Labs

use std::ops::{ Add, Sub, Mul, Div };

use crate::algorithm::controller::pid::PIDController;

pub struct PIDBController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    pid_ctrl: PIDController<T>,
    bias: T,
}

impl<T> PIDBController<T>
where
    T: Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Copy,
{
    pub fn new( p_gain: T, i_gain: T, d_gain: T, bias: T ) -> Self {
        Self {
            pid_ctrl: PIDController::new( p_gain, i_gain, d_gain ),
            bias,
        }
    }

    pub fn step( &mut self, error: T, dt: T ) -> T {
        let pid = self.pid_ctrl.step( error, dt );
        pid + self.bias
    }
}