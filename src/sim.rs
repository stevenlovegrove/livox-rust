//! A fake lidar on loopback, for running the driver and viewers without
//! hardware. It acknowledges parameter writes and, while sampling, streams a
//! Mid-360-like scan of a box-shaped room plus 200 Hz IMU packets.

use std::{
    io::Cursor,
    net::{Ipv4Addr, SocketAddrV4, UdpSocket},
    sync::{
        atomic::{AtomicBool, Ordering::Relaxed},
        Arc,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use binrw::{BinRead, BinWrite};

use crate::{
    control_packet::*,
    crc::{CRC_DATA, CRC_HEADER},
    data_packet::{encode_imu, encode_points, EncodeHeader, ImuSample, Point},
    datatype::{RetCode, WorkState},
    key::{KeyValue, KeyValueList},
    lidar::Config,
};

const POINTS_PER_PACKET: usize = 96;
const POINT_RATE_HZ: f64 = 200_000.0;
const IMU_RATE_HZ: f64 = 200.0;
/// Room half-extents (m) around the sensor, and the sensor's height above the floor.
const ROOM: [f32; 3] = [5.0, 4.0, 3.0];
const SENSOR_HEIGHT: f32 = 1.0;

pub struct FakeLidar {
    running: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl FakeLidar {
    /// Serves the lidar side of `cfg`'s ports on 127.0.0.1. Point `Device` at
    /// it with `Config::new(Ipv4Addr::LOCALHOST)`.
    pub fn spawn(cfg: &Config) -> anyhow::Result<Self> {
        let cmd = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.lidar_cmd_port))?;
        cmd.set_read_timeout(Some(Duration::from_millis(50)))?;
        let data = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0))?;
        let point_dst = SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.host_point_port);
        let imu_dst = SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.host_imu_port);

        let running = Arc::new(AtomicBool::new(true));
        let sampling = Arc::new(AtomicBool::new(false));

        let control = std::thread::spawn({
            let running = running.clone();
            let sampling = sampling.clone();
            move || serve_control(cmd, &running, &sampling)
        });

        let stream = std::thread::spawn({
            let running = running.clone();
            move || {
                let start = Instant::now();
                let packet_period = POINTS_PER_PACKET as f64 / POINT_RATE_HZ;
                let (mut n_points, mut n_imu) = (0u64, 0u64);
                let mut udp_cnt = 0u16;
                let mut imu_cnt = 0u16;
                while running.load(Relaxed) {
                    std::thread::sleep(Duration::from_millis(2));
                    let now = start.elapsed().as_secs_f64();
                    if !sampling.load(Relaxed) {
                        n_points = (now / packet_period) as u64;
                        n_imu = (now * IMU_RATE_HZ) as u64;
                        continue;
                    }
                    while (n_points as f64) * packet_period < now {
                        let t_ns = (n_points as f64 * packet_period * 1e9) as u64;
                        let _ = data.send_to(&point_packet(n_points, t_ns, udp_cnt), point_dst);
                        n_points += 1;
                        udp_cnt = udp_cnt.wrapping_add(1);
                    }
                    while (n_imu as f64) / IMU_RATE_HZ < now {
                        let t_ns = (n_imu as f64 / IMU_RATE_HZ * 1e9) as u64;
                        let _ = data.send_to(&imu_packet(t_ns, imu_cnt), imu_dst);
                        n_imu += 1;
                        imu_cnt = imu_cnt.wrapping_add(1);
                    }
                }
            }
        });

        Ok(Self {
            running,
            threads: vec![control, stream],
        })
    }
}

impl Drop for FakeLidar {
    fn drop(&mut self) {
        self.running.store(false, Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// Acknowledges every parameter write on `cmd` (a socket with a read
/// timeout), tracking whether the host has asked for sampling, until `running`
/// is cleared.
pub(crate) fn serve_control(cmd: UdpSocket, running: &AtomicBool, sampling: &AtomicBool) {
    let mut buf = [0u8; 1500];
    while running.load(Relaxed) {
        let Ok((len, from)) = cmd.recv_from(&mut buf) else { continue };
        let Ok(req) = ControlCommandPacket::read(&mut Cursor::new(&buf[..len])) else {
            continue;
        };
        if let Ok(list) = KeyValueList::read(&mut Cursor::new(&req.data)) {
            for item in list.items {
                if let KeyValue::WorkTgtMode(state) = item.value {
                    sampling.store(matches!(state, WorkState::Sampling), Relaxed);
                }
            }
        }
        let _ = cmd.send_to(&ack(&req), from);
    }
}

fn ack(req: &ControlCommandPacket) -> Vec<u8> {
    let mut data = Cursor::new(Vec::new());
    ControlCommandParamConfigAck {
        ret_code: RetCode::LvxRetSuccess,
        error_key: 0,
    }
    .write(&mut data)
    .unwrap();
    let data = data.into_inner();
    let header = ControlCommandPacketHeader {
        length: 24 + data.len() as u16,
        seq_num: req.header.seq_num,
        cmd_id: CommandID::LidarParamInfoConfig,
        cmd_type: CommandType::Ack,
        sender_type: SenderType::Lidar,
    };
    let mut out = Cursor::new(Vec::new());
    header.write(&mut out).unwrap();
    let crc16 = CRC_HEADER.checksum(&out.get_ref()[..18]);
    out.set_position(0);
    ControlCommandPacket {
        header,
        crc16,
        crc32: CRC_DATA.checksum(&data),
        data,
    }
    .write(&mut out)
    .unwrap();
    out.into_inner()
}

fn point_packet(index: u64, t_ns: u64, udp_cnt: u16) -> Vec<u8> {
    let points: Vec<Point> = (0..POINTS_PER_PACKET as u64)
        .map(|i| {
            let k = index * POINTS_PER_PACKET as u64 + i;
            // Golden-angle azimuth with a slowly sweeping elevation, over the
            // Mid-360's -7..52 degree vertical field of view.
            let azimuth = (k as f64 * 2.399_963_229_728_653) as f32;
            let sweep = ((k as f64 * 1e-4).sin() * 0.5 + 0.5) as f32;
            let elevation = (-7.0f32 + 59.0 * sweep).to_radians();
            let dir = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            let range = ray_to_room(dir);
            Point {
                xyz: dir.map(|c| c * range),
                reflectivity: (range * 20.0) as u8,
                ..Default::default()
            }
        })
        .collect();
    let time_interval = (POINTS_PER_PACKET as f64 / POINT_RATE_HZ * 1e7) as u16;
    let h = EncodeHeader { timestamp_ns: t_ns, udp_cnt, ..Default::default() };
    encode_points(h, time_interval, &points)
}

fn imu_packet(t_ns: u64, udp_cnt: u16) -> Vec<u8> {
    let h = EncodeHeader { timestamp_ns: t_ns, udp_cnt, ..Default::default() };
    encode_imu(h, &ImuSample { timestamp_ns: t_ns, gyro: [0.0; 3], accel: [0.0, 0.0, 1.0] })
}

/// Distance along unit `dir` from the sensor to the room's walls, floor or ceiling.
fn ray_to_room(dir: [f32; 3]) -> f32 {
    let lo = [-ROOM[0], -ROOM[1], -SENSOR_HEIGHT];
    let hi = [ROOM[0], ROOM[1], ROOM[2] - SENSOR_HEIGHT];
    (0..3)
        .filter(|&a| dir[a].abs() > 1e-6)
        .map(|a| if dir[a] > 0.0 { hi[a] / dir[a] } else { lo[a] / dir[a] })
        .fold(f32::INFINITY, f32::min)
}
