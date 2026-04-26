pub mod kalman;
pub mod udp_parser;

use std::fmt::Display;

#[derive(Debug, Clone, Copy)]
pub struct VehicleData {
    pub true_position: (f64, f64, f64),
    pub initial_speed: f64,
    pub acceleration: (f64, f64, f64),
    pub direction: (f64, f64, f64),
    pub time: Time,
    pub delta_time: f64,
}

#[derive(Debug)]
pub enum ParsedData {
    MsgStart,
    MsgEnd,
    TruePosition { x: f64, y: f64, z: f64 },
    Speed { s: f64 },
    Acceleration { x: f64, y: f64, z: f64 },
    Direction { x: f64, y: f64, z: f64 },
    Undefined,
}

#[derive(Debug, Clone, Copy)]
pub struct Time {
    pub hours: u32,
    pub minutes: u32,
    pub seconds: f64,
}

impl Time {
    pub fn as_sec(&self) -> f64 {
        self.hours as f64 * 3600.0 + self.minutes as f64 * 60.0 + self.seconds
    }
    /// Returns the time in seconds
    pub fn delta(first: Self, second: Self) -> f64 {
        first.as_sec() - second.as_sec()
    }
}

impl Display for Time {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{}:{}:{}", self.hours, self.minutes, self.seconds)?;
        Ok(())
    }
}

#[derive(Debug)]
pub struct Frame {
    pub time: Option<Time>,
    pub data: ParsedData,
}
