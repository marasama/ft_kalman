use super::*;
use matrix::matrix::Matrix;
use std::fmt::Display;

#[derive(Debug, Clone)]
pub struct Kalman {
    pub accel_error: f64,
    pub gyro_error: f64,
    pub gps_error: f64,
    noise_matrix: Matrix<f64>,
    pub state: Matrix<f64>,
    pub previous_state: Matrix<f64>,
}

impl Kalman {
    pub fn new(
        accel_error: f64,
        gyro_error: f64,
        gps_error: f64,
        noise_matrix: Matrix<f64>,
        state: Matrix<f64>,
        previous_state: Matrix<f64>,
    ) -> Self {
        Kalman {
            accel_error,
            gyro_error,
            gps_error,
            noise_matrix,
            state,
            previous_state,
        }
    }
}

impl Display for VehicleData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "TRUE_POSITION: ")?;
        writeln!(f, "{:?}", self.true_position)?;
        writeln!(f, "INITIAL_SPEED: ")?;
        writeln!(f, "{}", self.initial_speed)?;
        writeln!(f, "ACCELERATION: ")?;
        writeln!(f, "{:?}", self.acceleration)?;
        writeln!(f, "DIRECTION: ")?;
        writeln!(f, "{:?}", self.direction)?;
        writeln!(f, "TIME: ")?;
        writeln!(f, "{}", self.time)?;
        writeln!(f, "DELTA TIME: ")?;
        writeln!(f, "{:?}", self.delta_time)?;
        Ok(())
    }
}
