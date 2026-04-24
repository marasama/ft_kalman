//DATA FORMAT
//9 ---> "MSG_START"
//69 ---> "[00:00:00.000]TRUE POSITION\n8.087830169543953\n-3.684344480772115\n0.5\n"
//38 ---> "[00:00:00.000]SPEED\n54.33707206642073\n"
//91 ---> "[00:00:00.000]ACCELERATION\n-0.006974409692370343\n0.002745037193434422\n0.015353371960037995\n"
//87 ---> "[00:00:00.000]DIRECTION\n0.011132070329239845\n0.010590600280816103\n0.014958991641497313\n"
//7 ---> "MSG_END"

pub enum ParsingRes {
    MSG_START,
    MSG_END,
    SPEED,
    DIRECTION,
    ACCELERATION,
    UNDEFINED,
}

use std::char;

use crate::kalman::VehicleData;

pub fn parse(data: &[u8]) -> (ParsingRes, (f64, f64, f64)) {
    let text = data.iter().map(|a| *a as char).collect::<String>();
    if text.eq("MSG_START") {
        return (ParsingRes::MSG_START, (0., 0., 0.));
    }
    (ParsingRes::UNDEFINED, (-1., -1., -1.))
}

pub fn process_parsing(vehicle: VehicleData, res: (ParsingRes, (f64, f64, f64))) {}
