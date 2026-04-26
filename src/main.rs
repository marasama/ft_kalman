use ft_kalman::udp_parser::*;
use ft_kalman::*;
use std::net::UdpSocket;

fn main() -> std::io::Result<()> {
    let mut vehicle: VehicleData = VehicleData {
        true_position: (-1., -1., -1.),
        initial_speed: -1.,
        acceleration: (-1., -1., -1.),
        direction: (-1., -1., -1.),
        time: Time {
            hours: 0,
            minutes: 0,
            seconds: 0.,
        },
        delta_time: 0.,
    };

    let socket = UdpSocket::bind("127.0.0.1:8484")?;
    let _ = socket.connect("127.0.0.1:4242");
    let rdy = String::from("READY");
    loop {
        let _ = socket.send(rdy.as_bytes());
        let mut buffer: [u8; 300] = [0; 300];
        let (amt, _src) = socket.recv_from(&mut buffer)?;
        let buf = &buffer[..amt];
        process_parsing(&mut vehicle, parse(buf));
        //println!("{:?}", parse(buf));
        let ans = format!(
            "{} {} {}",
            vehicle.true_position.0 + 1.,
            vehicle.true_position.1 + 1.,
            vehicle.true_position.2 + 1.
        );
        socket.send(ans.as_bytes())?;
    }
}
