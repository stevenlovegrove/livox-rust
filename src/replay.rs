//! Replays a pcap recording as a lidar on loopback, so a `Device` opened with
//! `Config::new(Ipv4Addr::LOCALHOST)` receives it exactly as it would a live
//! sensor: same sockets, same decoding, same channel.
//!
//! Datagrams are re-sent byte for byte at their recorded pace (scaled by
//! `speed`), starting when the host asks the lidar to sample and pausing when it
//! asks it to stop. Only lidar data packets are replayed, each to the host port
//! of its stream (IMU or points); control traffic in the capture is ignored and
//! parameter writes are acknowledged instead.
//!
//! For batch processing without real-time pacing, read the file directly with
//! `pcap::Reader` and `data_packet::parse`.

use std::{
    net::{Ipv4Addr, SocketAddrV4, UdpSocket},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering::Relaxed},
        Arc,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use anyhow::Context;

use crate::{
    data_packet::PacketHeader,
    datatype::DataType,
    lidar::Config,
    pcap::{self, Datagram},
    sim::serve_control,
};

#[derive(Debug, Clone)]
pub struct ReplayOptions {
    /// Playback rate relative to the recording (2.0 is twice as fast).
    pub speed: f64,
    /// Start again from the beginning at the end of the file. The lidar's
    /// timestamps then jump back.
    pub looped: bool,
    /// Replay only datagrams from this address (for captures of several lidars).
    pub lidar_ip: Option<Ipv4Addr>,
}

impl Default for ReplayOptions {
    fn default() -> Self {
        Self {
            speed: 1.0,
            looped: false,
            lidar_ip: None,
        }
    }
}

pub struct ReplayLidar {
    running: Arc<AtomicBool>,
    finished: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl ReplayLidar {
    /// Serves the lidar side of `cfg`'s ports on 127.0.0.1 from `path`.
    pub fn spawn(cfg: &Config, path: impl AsRef<Path>, opts: ReplayOptions) -> anyhow::Result<Self> {
        let path = path.as_ref().to_path_buf();
        anyhow::ensure!(opts.speed > 0.0 && opts.speed.is_finite(), "speed must be positive");
        // Fail now, not in the thread, if the file is unreadable.
        pcap::Reader::open(&path).with_context(|| format!("opening {}", path.display()))?;

        let cmd = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.lidar_cmd_port))
            .context("binding replay control socket")?;
        cmd.set_read_timeout(Some(Duration::from_millis(50)))?;
        let data = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0))?;

        let running = Arc::new(AtomicBool::new(true));
        let sampling = Arc::new(AtomicBool::new(false));
        let finished = Arc::new(AtomicBool::new(false));

        let control = std::thread::spawn({
            let running = running.clone();
            let sampling = sampling.clone();
            move || serve_control(cmd, &running, &sampling)
        });

        let stream = std::thread::spawn({
            let player = Player {
                path,
                opts,
                socket: data,
                point_dst: SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.host_point_port),
                imu_dst: SocketAddrV4::new(Ipv4Addr::LOCALHOST, cfg.host_imu_port),
                running: running.clone(),
                sampling,
            };
            let finished = finished.clone();
            move || {
                if let Err(e) = player.run() {
                    eprintln!("livox replay: {e:#}");
                }
                finished.store(true, Relaxed);
            }
        });

        Ok(Self {
            running,
            finished,
            threads: vec![control, stream],
        })
    }

    /// True once every datagram has been sent (never, when looping).
    pub fn finished(&self) -> bool {
        self.finished.load(Relaxed)
    }
}

impl Drop for ReplayLidar {
    fn drop(&mut self) {
        self.running.store(false, Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

struct Player {
    path: PathBuf,
    opts: ReplayOptions,
    socket: UdpSocket,
    point_dst: SocketAddrV4,
    imu_dst: SocketAddrV4,
    running: Arc<AtomicBool>,
    sampling: Arc<AtomicBool>,
}

impl Player {
    fn run(&self) -> anyhow::Result<()> {
        loop {
            if !self.play_once()? || !self.opts.looped {
                return Ok(());
            }
        }
    }

    /// Plays the file through once. Returns false if stopped early.
    fn play_once(&self) -> anyhow::Result<bool> {
        let reader = pcap::Reader::open(&self.path)?;
        // Maps a recording time to the wall-clock instant it is due.
        let mut clock: Option<(u64, Instant)> = None;
        for datagram in reader {
            let Datagram { capture_ns, src, payload, .. } = datagram?;
            if self.opts.lidar_ip.is_some_and(|ip| ip != *src.ip()) {
                continue;
            }
            let Ok(header) = PacketHeader::parse(&payload) else { continue };
            let dst = match header.data_type {
                DataType::IMUData => self.imu_dst,
                _ => self.point_dst,
            };

            // Hold while the host has the lidar idle, then carry on from here.
            let mut paused = false;
            while !self.sampling.load(Relaxed) {
                if !self.running.load(Relaxed) {
                    return Ok(false);
                }
                paused = true;
                std::thread::sleep(Duration::from_millis(1));
            }
            if !self.running.load(Relaxed) {
                return Ok(false);
            }
            let (t0, wall0) = match clock {
                Some(c) if !paused => c,
                _ => {
                    let c = (capture_ns, Instant::now());
                    clock = Some(c);
                    c
                }
            };
            let due = wall0 + Duration::from_secs_f64(capture_ns.saturating_sub(t0) as f64 * 1e-9 / self.opts.speed);
            let now = Instant::now();
            if due > now {
                std::thread::sleep(due - now);
            }
            self.socket.send_to(&payload, dst)?;
        }
        Ok(true)
    }
}
