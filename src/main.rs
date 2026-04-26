use ft_kalman::kalman::*;
use ft_kalman::udp_parser::*;
use std::net::UdpSocket;

fn main() -> std::io::Result<()> {
    let mut vehicle: VehicleData = VehicleData {
        true_position: (-1., -1., -1.),
        initial_speed: -1.,
        acceleration: (-1., -1., -1.),
        direction: (-1., -1., -1.),
        delta_time: (0, 0, 0.),
    };

    println!("{}", vehicle);
    let socket = UdpSocket::bind("127.0.0.1:8484")?;
    let _ = socket.connect("127.0.0.1:4242");
    let rdy = String::from("READY");
    loop {
        let _ = socket.send(rdy.as_bytes());
        let mut buffer: [u8; 300] = [0; 300];
        let (amt, _src) = socket.recv_from(&mut buffer)?;
        let buf = &buffer[..amt];

        println!(
            " {:?} ---> {:?}",
            amt,
            buf.iter().map(|a| *a as char).collect::<String>()
        );
        println!("{:?}", parse(buf));
    }
}
