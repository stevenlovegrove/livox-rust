use std::{
    net::Ipv4Addr,
    sync::atomic::Ordering::Relaxed,
    time::{Duration, Instant},
};

use livox_rust::{sim::FakeLidar, Config, Device, ImuRate, Packet};

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
    assert_eq!(s.points.parse_errors.load(Relaxed), 0);
    assert_eq!(s.points.seq_gaps.load(Relaxed), 0);
    assert_eq!(s.imu.seq_gaps.load(Relaxed), 0);
    device.stop().unwrap();
}

#[test]
fn sets_the_imu_rate_keeping_its_ranges() {
    let mut cfg = Config::new(Ipv4Addr::LOCALHOST);
    cfg.host_ip = Ipv4Addr::LOCALHOST;
    for port in [&mut cfg.lidar_cmd_port, &mut cfg.host_cmd_port, &mut cfg.host_point_port, &mut cfg.host_imu_port] {
        *port += 2000;
    }
    cfg.imu_rate = Some(ImuRate::Hz500);

    let _lidar = FakeLidar::spawn(&cfg).unwrap();
    let (mut device, rx) = Device::open(cfg).unwrap();
    let before = device.imu_sensor_cfg().unwrap().unwrap();
    assert_eq!(before.rate(), Some(ImuRate::Hz200));
    device.start().unwrap();
    let after = device.imu_sensor_cfg().unwrap().unwrap();
    assert_eq!(after.rate(), Some(ImuRate::Hz500));
    assert_eq!((after.accel_range, after.gyro_range), (before.accel_range, before.gyro_range), "ranges kept");

    // Count IMU samples over a second, after a moment to settle.
    std::thread::sleep(Duration::from_millis(100));
    while rx.try_recv().is_ok() {}
    let (mut imu, end) = (0usize, Instant::now() + Duration::from_secs(1));
    while Instant::now() < end {
        if let Ok(Packet::Imu { .. }) = rx.recv_timeout(Duration::from_millis(50)) {
            imu += 1;
        }
    }
    assert!((450..=550).contains(&imu), "{imu} IMU samples in 1 s at 500 Hz");
    device.stop().unwrap();
}
