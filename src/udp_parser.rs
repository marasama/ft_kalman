//DATA FORMAT
//9 ---> "MSG_START"
//69 ---> "[00:00:00.000]TRUE POSITION\n8.087830169543953\n-3.684344480772115\n0.5\n"
//38 ---> "[00:00:00.000]SPEED\n54.33707206642073\n"
//91 ---> "[00:00:00.000]ACCELERATION\n-0.006974409692370343\n0.002745037193434422\n0.015353371960037995\n"
//87 ---> "[00:00:00.000]DIRECTION\n0.011132070329239845\n0.010590600280816103\n0.014958991641497313\n"
//7 ---> "MSG_END"

use super::*;

const UNDEF_FRAME: Frame = Frame {
    time: None,
    data: ParsedData::Undefined,
};

pub fn parse_time(time_text: &str) -> Option<Time> {
    let (hour, rest) = time_text.split_once(':')?;
    let (minute, second) = rest.split_once(':')?;
    Some(Time {
        hours: hour.parse::<u32>().ok()?,
        minutes: minute.parse::<u32>().ok()?,
        seconds: second.parse::<f64>().ok()?,
    })
}

pub fn parse(data: &[u8]) -> Frame {
    let Ok(text) = std::str::from_utf8(data) else {
        return UNDEF_FRAME;
    };
    match text {
        "MSG_START" => {
            return Frame {
                time: None,
                data: ParsedData::MsgStart,
            }
        }
        "MSG_END" => {
            return Frame {
                time: None,
                data: ParsedData::MsgEnd,
            }
        }
        _ => {}
    }
    let Some((header, body)) = text.split_once('\n') else {
        return UNDEF_FRAME;
    };

    let Some((current_time, header)) = header
        .split_once(']')
        .map(|(time, rest)| (parse_time(&time[1..]), rest))
    else {
        return UNDEF_FRAME;
    };

    let mut lines = body.lines();
    let mut get_f64 = || lines.next()?.parse::<f64>().ok();
    let data = match header {
        "TRUE POSITION" => match (get_f64(), get_f64(), get_f64()) {
            (Some(x), Some(y), Some(z)) => ParsedData::TruePosition { x, y, z },
            _ => ParsedData::Undefined,
        },
        "DIRECTION" => match (get_f64(), get_f64(), get_f64()) {
            (Some(x), Some(y), Some(z)) => ParsedData::Direction { x, y, z },
            _ => ParsedData::Undefined,
        },
        "ACCELERATION" => match (get_f64(), get_f64(), get_f64()) {
            (Some(x), Some(y), Some(z)) => ParsedData::Acceleration { x, y, z },
            _ => ParsedData::Undefined,
        },
        "SPEED" => match get_f64() {
            Some(s) => ParsedData::Speed { s },
            _ => ParsedData::Undefined,
        },
        _ => ParsedData::Undefined,
    };
    Frame {
        time: current_time,
        data,
    }
}

pub fn process_parsing(vehicle: &mut VehicleData, res: Frame) {
    if let Some(t) = res.time {
        vehicle.delta_time = Time::delta(t, vehicle.time);
        vehicle.time = t;
    }
    match res.data {
        ParsedData::TruePosition { x, y, z } => vehicle.true_position = (x, y, z),
        ParsedData::Acceleration { x, y, z } => vehicle.acceleration = (x, y, z),
        ParsedData::Direction { x, y, z } => vehicle.direction = (x, y, z),
        ParsedData::Speed { s } => vehicle.initial_speed = s,
        ParsedData::MsgStart => println!("Message Started!"),
        ParsedData::MsgEnd => println!("{}", vehicle),
        ParsedData::Undefined => println!("Undefined entry!"),
    }
}
