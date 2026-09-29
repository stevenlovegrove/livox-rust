use std::{
    net::Ipv4Addr,
    path::Path,
    sync::atomic::Ordering::Relaxed,
    time::Duration,
};

use livox_rust::{
    pcap,
    replay::{ReplayLidar, ReplayOptions},
    sim::FakeLidar,
    Config, Device, Packet,
};

fn config() -> Config {
    // Off the default ports (and the other tests'), so nothing else interferes.
    let mut cfg = Config::new(Ipv4Addr::LOCALHOST);
    cfg.host_ip = Ipv4Addr::LOCALHOST;
    cfg.lidar_cmd_port += 2000;
    cfg.host_cmd_port += 2000;
    cfg.host_point_port += 2000;
    cfg.host_imu_port += 2000;
    cfg
}

/// (header timestamp, first point / imu sample) per packet, for comparison.
fn key(p: &Packet) -> (u64, u16, [f32; 3]) {
    match p {
        Packet::Points(p) => (p.header.timestamp_ns, p.header.dot_num, p.points[0].xyz),
        Packet::Imu { header, sample } => (header.timestamp_ns, 0, sample.accel),
    }
}

/// Receives while `more()`, then until the channel has been quiet for 200 ms.
fn collect_while(rx: &std::sync::mpsc::Receiver<Packet>, mut more: impl FnMut() -> bool) -> Vec<Packet> {
    let mut out = Vec::new();
    while more() {
        out.extend(rx.recv_timeout(Duration::from_millis(10)));
    }
    while let Ok(p) = rx.recv_timeout(Duration::from_millis(200)) {
        out.push(p);
    }
    out
}

#[test]
fn replayed_recording_matches_live_stream() {
    let dir = std::env::temp_dir().join(format!("livox-record-replay-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("sim.pcap");

    // Record 300 ms of the simulator through a live Device.
    let live = {
        let mut cfg = config();
        cfg.record = Some(path.clone());
        let _lidar = FakeLidar::spawn(&cfg).unwrap();
        let (mut device, rx) = Device::open(cfg).unwrap();
        device.start().unwrap();
        std::thread::sleep(Duration::from_millis(300));
        device.stop().unwrap();
        let packets = collect_while(&rx, || false);
        drop(device); // flushes the recording
        packets
    };
    assert!(live.len() > 300, "only {} packets live", live.len());

    // The file holds exactly the datagrams the device received.
    let recorded = pcap::Reader::open(&path).unwrap().count();
    assert_eq!(recorded, live.len());

    // Replay it (fast) through a fresh Device and compare packet by packet.
    let cfg = config();
    let replay = ReplayLidar::spawn(&cfg, &path, ReplayOptions { speed: 4.0, ..Default::default() }).unwrap();
    let (mut device, rx) = Device::open(cfg).unwrap();
    device.start().unwrap();
    let replayed = collect_while(&rx, || !replay.finished());
    assert_eq!(device.stats().points.dropped.load(Relaxed), 0);

    // Points and IMU come from separate sockets, so compare each stream in order.
    for imu in [false, true] {
        let pick = |v: &[Packet]| -> Vec<_> {
            v.iter().filter(|p| matches!(p, Packet::Imu { .. }) == imu).map(key).collect()
        };
        assert_eq!(pick(&replayed), pick(&live), "imu stream: {imu}");
    }
    let _ = std::fs::remove_dir_all(Path::new(&dir));
}
