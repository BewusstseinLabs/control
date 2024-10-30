// Copyright 2024 Bewusstsein Labs

pub struct KalmanFilter {
    pub state: f64,
    pub covariance: f64,
    pub process_noise: f64,
    pub measurement_noise: f64,
    pub kalman_gain: f64,
}