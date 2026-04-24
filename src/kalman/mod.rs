use std::fmt::{write, Display};

use matrix::matrix::Matrix;

#[derive(Debug, Clone, Copy)]
pub struct VehicleData {
    pub true_position: (f64, f64, f64),
    pub initial_speed: f64,
    pub acceleration: (f64, f64, f64),
    pub direction: (f64, f64, f64),
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
        Ok(())
    }
}
