#![allow(non_snake_case)]
use ft_kalman::filters::KalmanCore;
use ft_kalman::udp_parser::*;
use ft_kalman::*;
use matrix::matrix::Matrix;
use matrix::vector::Vector;
use std::net::UdpSocket;

const DELTA_T: f64 = 0.01;
// state vector =
// |x_angle |
// |y_angle |
// |z_angle |
// |x_pos   |
// |x_vel   |
// |x_accel |
// |y_pos   |
// |y_vel   |
// |y_accel |
// |z_pos   |
// |z_vel   |
// |z_accel |

//DATA FORMAT
//9 ---> "MSG_START"
//69 ---> "[00:00:00.000]TRUE POSITION\n8.087830169543953\n-3.684344480772115\n0.5\n"
//69 ---> "[00:00:00.000]POSITION\n8.087830169543953\n-3.684344480772115\n0.5\n"
//38 ---> "[00:00:00.000]SPEED\n54.33707206642073\n"
//91 ---> "[00:00:00.000]ACCELERATION\n-0.006974409692370343\n0.002745037193434422\n0.015353371960037995\n"
//87 ---> "[00:00:00.000]DIRECTION\n0.011132070329239845\n0.010590600280816103\n0.014958991641497313\n"
//7 ---> "MSG_END"

fn main() -> std::io::Result<()> {
    let mut vehicle: VehicleData = VehicleData {
        true_position: (-1., -1., -1.),
        initial_speed: -1.,
        acceleration: (-1., -1., -1.),
        direction: (-1., -1., -1.),
        gps_position: (0., 0., 0.),
        gps_fresh: false,
        time: Time {
            hours: 0,
            minutes: 0,
            seconds: 0.,
        },
        delta_time: 0.,
    };

    let delta_t_pow_4 = num_traits::pow(DELTA_T, 4);
    let delta_t_pow_3 = num_traits::pow(DELTA_T, 3);
    let delta_t_pow_2 = num_traits::pow(DELTA_T, 2);
    let dt_2 = delta_t_pow_2 * 0.5;
    let gyro_noise = num_traits::pow(1e-2, 2);
    // σ²_gyro · Δt²
    let gn_delta: f64 = gyro_noise * delta_t_pow_2;
    let accel_noise: f64 = num_traits::pow(1e-3, 2);
    let gps_noise: f64 = num_traits::pow(1e-1, 2);
    let q_pp = (delta_t_pow_4 / 4.) * accel_noise;
    let q_pv = (delta_t_pow_3 / 2.) * accel_noise;
    let q_pa = (delta_t_pow_2 / 2.) * accel_noise;
    let q_vv = delta_t_pow_2 * accel_noise;
    let q_va = DELTA_T * accel_noise;
    let q_aa = accel_noise;
    let Q: Matrix<f64> = Matrix::from([
        [gn_delta, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., gn_delta, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., gn_delta, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., q_pp, q_pv, q_pa, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., q_pv, q_vv, q_va, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., q_pa, q_va, q_aa, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., q_pp, q_pv, q_pa, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., q_pv, q_vv, q_va, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., q_pa, q_va, q_aa, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., q_pp, q_pv, q_pa],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., q_pv, q_vv, q_va],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., q_pa, q_va, q_aa],
    ]);

    let R: Matrix<f64> = Matrix::from([
        [gyro_noise, 0., 0., 0., 0., 0.],
        [0., gyro_noise, 0., 0., 0., 0.],
        [0., 0., gyro_noise, 0., 0., 0.],
        [0., 0., 0., accel_noise, 0., 0.],
        [0., 0., 0., 0., accel_noise, 0.],
        [0., 0., 0., 0., 0., accel_noise],
    ]);

    let R_gps: Matrix<f64> = Matrix::from([
        [gps_noise, 0., 0.],
        [0., gps_noise, 0.],
        [0., 0., gps_noise],
    ]);

    let F: Matrix<f64> = Matrix::from([
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 1., DELTA_T, dt_2, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 1., DELTA_T, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 1., DELTA_T, dt_2, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 1., DELTA_T, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., DELTA_T, dt_2],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., DELTA_T],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
    ]);

    let H: Matrix<f64> = Matrix::from([
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
    ]);

    let H_gps: Matrix<f64> = Matrix::from([
        [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
    ]);
    let model = filters::LKF::new(12, 6, 0, F, H, Option::None);

    let x: Vector<f64> = Vector::from([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]);
    let P: Matrix<f64> = Matrix::from([
        [1e-4, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 1e-4, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1e-4, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 1e-2, 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 1e-6, 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 1e-2, 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 1e-6, 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 1e-2, 0., 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.1, 0.],
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1e-6],
    ]);

    let mut z: Vector<f64> = Vector::from([0., 0., 0., 0., 0., 0.]);
    let u = Option::None;

    let mut filter = KalmanCore::new(model, x, P, Q, R);
    let socket = UdpSocket::bind("127.0.0.1:8484")?;
    let _ = socket.connect("127.0.0.1:4242");
    let rdy = String::from("READY");
    let _ = socket.send(rdy.as_bytes());

    loop {
        let mut buffer: [u8; 400] = [0; 400];
        let (amt, _src) = socket.recv_from(&mut buffer)?;
        let buf = &buffer[..amt];
        if process_parsing(&mut vehicle, parse(buf)) {
            break;
        }
    }

    let speed_ms = vehicle.initial_speed / 3.6;
    let (roll, pitch, yaw) = vehicle.direction;

    filter.x = Vector::from([
        roll,
        pitch,
        yaw,
        vehicle.true_position.0,
        speed_ms * pitch.cos() * yaw.cos(),
        vehicle.acceleration.0,
        vehicle.true_position.1,
        speed_ms * pitch.cos() * yaw.sin(),
        vehicle.acceleration.1,
        vehicle.true_position.2,
        speed_ms * pitch.sin(),
        vehicle.acceleration.2,
    ]);

    loop {
        //println!("{}", filter.state());
        let ans = format!(
            "{} {} {}",
            filter.x.data[3], filter.x.data[6], filter.x.data[9],
        );
        socket.send(ans.as_bytes())?;
        loop {
            let mut buffer: [u8; 400] = [0; 400];
            let (amt, _src) = socket.recv_from(&mut buffer)?;
            //println!(r"{}", String::from_utf8_lossy(&buffer[..amt]));
            if process_parsing(&mut vehicle, parse(&buffer[..amt])) {
                break;
            }
        }

        let (roll, pitch, yaw) = vehicle.direction;

        let ax_body = vehicle.acceleration.0;
        let ay_body = vehicle.acceleration.1;
        let az_body = vehicle.acceleration.2;

        let ax_world = ax_body * pitch.cos() * yaw.cos() - ay_body * yaw.sin()
            + az_body * pitch.sin() * yaw.cos();
        let ay_world = ax_body * pitch.cos() * yaw.sin()
            + ay_body * yaw.cos()
            + az_body * pitch.sin() * yaw.sin();
        let az_world = -ax_body * pitch.sin() + az_body * pitch.cos();
        z.data[0] = vehicle.direction.0;
        z.data[1] = vehicle.direction.1;
        z.data[2] = vehicle.direction.2;
        z.data[3] = ax_body;
        z.data[4] = ay_body;
        z.data[5] = az_body;
        filter.step(&z, &u);

        if vehicle.gps_fresh {
            let z_gps: Vector<f64> = Vector::from([
                vehicle.gps_position.0,
                vehicle.gps_position.1,
                vehicle.gps_position.2,
            ]);

            let x_post = filter.x.clone();
            let P_post = filter.P.clone();
            (filter.x, filter.P, filter.K) = filter
                .model
                .update_with(&x_post, &z_gps, &u, &P_post, &R_gps, &H_gps);
            vehicle.gps_fresh = false;
        }
    }
}
