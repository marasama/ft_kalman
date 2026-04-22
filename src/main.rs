use std::net::UdpSocket;

fn main() -> std::io::Result<()> {
    let socket = UdpSocket::bind("127.0.0.1:8484")?;

    let _ = socket.connect("127.0.0.1:4242");
    let rdy = String::from("READY");
    let binding = rdy.into_bytes();
    let send_data: &[u8] = binding.as_array::<5>().unwrap();
    let _ = socket.send(send_data);
    let mut a = [0; 30];
    let (amt, src) = socket.recv_from(&mut a)?;
    let buf = &mut a[..amt];
    println!(" {:?} ---> {:?}", amt, buf);

    Ok(())
}
