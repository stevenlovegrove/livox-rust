use std::{
    net::Ipv4Addr,
    sync::atomic::Ordering::Relaxed,
    time::{Duration, Instant},
};

use livox_rust::{sim::FakeLidar, Config, Device, Packet};

#[test]
fn streams_points_and_imu_from_fake_lidar() {
    // Off the default ports, so a real lidar or a running viewer does not interfere.
    let mut cfg = Config::new(Ipv4Addr::LOCALHOST);
    cfg.host_ip = Ipv4Addr::LOCALHOST;
    cfg.lidar_cmd_port += 1000;
    cfg.host_cmd_port += 1000;
    cfg.host_point_port += 1000;
    cfg.host_imu_port += 1000;

    let _lidar = FakeLidar::spawn(&cfg).unwrap();
    let (mut device, rx) = Device::open(cfg).unwrap();
    device.start().unwrap();

    let (mut points, mut imu, mut last_t) = (0usize, 0usize, 0u64);
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(Packet::Points(p)) => {
                assert!(p.header.timestamp_ns >= last_t, "point packets out of order");
                last_t = p.header.timestamp_ns;
                assert_eq!(p.points.len(), 96);
                assert!(p.points.iter().all(|pt| pt.is_valid()));
                assert!(p.points[95].offset_ns > 0);
                points += p.points.len();
            }
            Ok(Packet::Imu { sample, .. }) => {
                assert_eq!(sample.accel, [0.0, 0.0, 1.0]);
                imu += 1;
            }
            Err(_) => {}
        }
    }

    // 200k points/s and 200 Hz IMU over 0.5 s, allowing for start-up.
    assert!(points > 50_000, "only {points} points");
    assert!(imu > 50, "only {imu} imu samples");
    let s = device.stats();
    assert_eq!(s.points.crc_errors.load(Relaxed), 0);
    assert_eq!(s.points.parse_errors.load(Relaxed), 0);
    assert_eq!(s.points.seq_gaps.load(Relaxed), 0);
    assert_eq!(s.imu.seq_gaps.load(Relaxed), 0);
    device.stop().unwrap();
}
