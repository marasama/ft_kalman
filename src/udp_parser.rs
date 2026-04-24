//DATA FORMAT
//9 ---> "MSG_START"
//69 ---> "[00:00:00.000]TRUE POSITION\n8.087830169543953\n-3.684344480772115\n0.5\n"
//38 ---> "[00:00:00.000]SPEED\n54.33707206642073\n"
//91 ---> "[00:00:00.000]ACCELERATION\n-0.006974409692370343\n0.002745037193434422\n0.015353371960037995\n"
//87 ---> "[00:00:00.000]DIRECTION\n0.011132070329239845\n0.010590600280816103\n0.014958991641497313\n"
//7 ---> "MSG_END"

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

use core::f64;

use crate::kalman::VehicleData;

pub fn parse(data: &[u8]) -> ParsedData {
    let Ok(text) = std::str::from_utf8(data) else {
        return ParsedData::Undefined;
    };
    match text {
        "MSG_START" => return ParsedData::MsgStart,
        "MSG_END" => return ParsedData::MsgEnd,
        _ => {}
    }

    let Some((header, body)) = text.split_once('\n') else {
        return ParsedData::Undefined;
    };

    let header = header.find(']').map(|i| &header[i + 1..]).unwrap_or(header);
    let mut nums = body.lines().map(|n| n.parse::<f64>().unwrap_or(f64::NAN));

    match header {
        "TRUE POSITION" => {
            let (Some(x), Some(y), Some(z)) = (nums.next(), nums.next(), nums.next()) else {
                return ParsedData::Undefined;
            };
            ParsedData::TruePosition { x, y, z }
        }
        "SPEED" => {
            let Some(s) = nums.next() else {
                return ParsedData::Undefined;
            };
            ParsedData::Speed { s }
        }
        "DIRECTION" => {
            let (Some(x), Some(y), Some(z)) = (nums.next(), nums.next(), nums.next()) else {
                return ParsedData::Undefined;
            };
            ParsedData::Acceleration { x, y, z }
        }
        "ACCELERATION" => {
            let (Some(x), Some(y), Some(z)) = (nums.next(), nums.next(), nums.next()) else {
                return ParsedData::Undefined;
            };
            ParsedData::Direction { x, y, z }
        }

        _ => ParsedData::Undefined,
    }
}

pub fn process_parsing(vehicle: &mut VehicleData, res: ParsedData) {
    println!("{:?}", res);
}
