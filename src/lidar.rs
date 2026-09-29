//! A Livox SDK2 device (e.g. Mid-360): control over UDP, plus one receive
//! thread per data stream (point cloud, IMU) that decodes packets and hands
//! them to the caller over a bounded channel.

use std::{
    io::{Cursor, ErrorKind},
    fs::File,
    io::BufWriter,
    net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering::Relaxed},
        mpsc::{Receiver, SyncSender, TrySendError},
        Arc, Mutex,
    },
    thread::JoinHandle,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use anyhow::{anyhow, bail, Context};
use binrw::BinRead;
use socket2::{Domain, Protocol, Socket, Type};

use crate::{
    control_packet::{CommandID, CommandType, ControlCommandPacket, ControlCommandParamConfigAck},
    data_packet::{self, Packet},
    datatype::{RetCode, WorkState},
    key::KeyValueList,
    params::{packet_set_params, params_imu_enable, params_work_mode},
    pcap,
};

type Recorder = Arc<Mutex<pcap::Writer<BufWriter<File>>>>;

#[derive(Debug, Clone)]
pub struct Config {
    pub lidar_ip: Ipv4Addr,
    /// Local interface to bind; `UNSPECIFIED` binds all of them.
    pub host_ip: Ipv4Addr,
    pub lidar_cmd_port: u16,
    pub host_cmd_port: u16,
    pub host_point_port: u16,
    pub host_imu_port: u16,
    /// Requested SO_RCVBUF per data socket. The OS default (tens of KB on
    /// macOS) holds only a few ms of point data.
    pub recv_buffer_bytes: usize,
    /// Packets buffered between the receive threads and the consumer before
    /// new ones are dropped (and counted in `Stats::dropped`).
    pub queue_packets: usize,
    pub ack_timeout: Duration,
    /// Record every datagram from the lidar, as received, to this pcap file
    /// (replay it with `replay::ReplayLidar`).
    pub record: Option<PathBuf>,
}

impl Config {
    /// Mid-360 default ports.
    pub fn new(lidar_ip: Ipv4Addr) -> Self {
        Self {
            lidar_ip,
            host_ip: Ipv4Addr::UNSPECIFIED,
            lidar_cmd_port: 56100,
            host_cmd_port: 56101,
            host_point_port: 56301,
            host_imu_port: 56401,
            recv_buffer_bytes: 8 << 20,
            queue_packets: 16384,
            ack_timeout: Duration::from_secs(1),
            record: None,
        }
    }
}

/// Counters for one data stream.
#[derive(Debug, Default)]
pub struct StreamStats {
    pub packets: AtomicU64,
    pub points: AtomicU64,
    pub parse_errors: AtomicU64,
    pub crc_errors: AtomicU64,
    /// Packets whose `udp_cnt` did not follow the previous one: packets were
    /// lost on the wire or in the socket buffer.
    pub seq_gaps: AtomicU64,
    /// Packets decoded but discarded because the consumer fell behind.
    pub dropped: AtomicU64,
}

#[derive(Debug, Default)]
pub struct Stats {
    pub points: StreamStats,
    pub imu: StreamStats,
}

pub struct Device {
    cfg: Config,
    cmd: UdpSocket,
    seq_num: u32,
    sampling: bool,
    running: Arc<AtomicBool>,
    stats: Arc<Stats>,
    threads: Vec<JoinHandle<()>>,
    recorder: Option<Recorder>,
}

impl Device {
    /// Binds the sockets and starts the receive threads. Packets arrive on the
    /// returned channel once the lidar is sampling (see `start`).
    pub fn open(cfg: Config) -> anyhow::Result<(Self, Receiver<Packet>)> {
        let cmd = UdpSocket::bind(SocketAddrV4::new(cfg.host_ip, cfg.host_cmd_port))
            .context("binding control socket")?;
        cmd.connect(SocketAddrV4::new(cfg.lidar_ip, cfg.lidar_cmd_port))?;
        cmd.set_read_timeout(Some(cfg.ack_timeout))?;

        let (tx, rx) = std::sync::mpsc::sync_channel(cfg.queue_packets);
        let running = Arc::new(AtomicBool::new(true));
        let stats = Arc::new(Stats::default());
        let recorder = match &cfg.record {
            Some(path) => Some(Arc::new(Mutex::new(
                pcap::Writer::create(path).with_context(|| format!("creating {}", path.display()))?,
            ))),
            None => None,
        };

        let mut threads = Vec::new();
        for (port, name) in [(cfg.host_point_port, "livox-points"), (cfg.host_imu_port, "livox-imu")] {
            let socket = bind_data_socket(SocketAddrV4::new(cfg.host_ip, port), cfg.recv_buffer_bytes)
                .with_context(|| format!("binding {name} socket on port {port}"))?;
            let ctx = RecvContext {
                socket,
                lidar_ip: cfg.lidar_ip,
                tx: tx.clone(),
                running: running.clone(),
                stats: stats.clone(),
                recorder: recorder.clone(),
            };
            threads.push(std::thread::Builder::new().name(name.into()).spawn(move || ctx.run())?);
        }

        let device = Self {
            cfg,
            cmd,
            seq_num: 0,
            sampling: false,
            running,
            stats,
            threads,
            recorder,
        };
        Ok((device, rx))
    }

    /// Enables the IMU stream and starts sampling.
    pub fn start(&mut self) -> anyhow::Result<()> {
        self.set_params(params_imu_enable(true))?;
        self.set_params(params_work_mode(WorkState::Sampling))?;
        self.sampling = true;
        Ok(())
    }

    pub fn stop(&mut self) -> anyhow::Result<()> {
        self.set_params(params_work_mode(WorkState::IDLE))?;
        self.sampling = false;
        Ok(())
    }

    /// Live counters, shareable with other threads.
    pub fn stats(&self) -> Arc<Stats> {
        self.stats.clone()
    }

    /// Writes `params` and waits for the lidar to acknowledge them.
    pub fn set_params(&mut self, params: KeyValueList) -> anyhow::Result<()> {
        self.seq_num = self.seq_num.wrapping_add(1);
        let seq_num = self.seq_num;
        self.cmd.send(&packet_set_params(seq_num, params)?)?;

        let deadline = Instant::now() + self.cfg.ack_timeout;
        let mut buf = [0u8; 1500];
        while Instant::now() < deadline {
            let len = match self.cmd.recv(&mut buf) {
                Ok(len) => len,
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => break,
                Err(e) => return Err(e).context("waiting for ack"),
            };
            let Ok(packet) = ControlCommandPacket::read(&mut Cursor::new(&buf[..len])) else {
                continue;
            };
            let h = &packet.header;
            if h.seq_num != seq_num
                || !matches!(h.cmd_type, CommandType::Ack)
                || !matches!(h.cmd_id, CommandID::LidarParamInfoConfig)
            {
                continue;
            }
            let ack = ControlCommandParamConfigAck::read(&mut Cursor::new(&packet.data))
                .map_err(|e| anyhow!("malformed ack: {e}"))?;
            if ack.ret_code != RetCode::LvxRetSuccess {
                bail!("lidar rejected key 0x{:04X}: {:?}", ack.error_key, ack.ret_code);
            }
            return Ok(());
        }
        // Which local address the OS would use to reach the lidar: if it is on
        // another subnet, the lidar's link is down or has no address.
        let route = UdpSocket::bind("0.0.0.0:0")
            .and_then(|s| s.connect((self.cfg.lidar_ip, 9)).and(s.local_addr()))
            .map(|a| a.ip().to_string())
            .unwrap_or_else(|_| "none".into());
        bail!(
            "no ack from lidar at {}:{} within {:?} (packets to it leave from {route}). \
             Check it is powered and that a wired interface has an address on its subnet \
             (the lidar sends data to its configured host IP, 192.168.1.50 by default).",
            self.cfg.lidar_ip,
            self.cfg.lidar_cmd_port,
            self.cfg.ack_timeout
        )
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if self.sampling {
            if let Err(e) = self.stop() {
                eprintln!("livox: failed to stop lidar: {e:#}");
            }
        }
        self.running.store(false, Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        if let Some(recorder) = &self.recorder {
            if let Err(e) = recorder.lock().unwrap().flush() {
                eprintln!("livox: failed to finish recording: {e}");
            }
        }
    }
}

fn bind_data_socket(addr: SocketAddrV4, recv_buffer_bytes: usize) -> anyhow::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    // The OS caps the size (kern.ipc.maxsockbuf on macOS, net.core.rmem_max on
    // Linux) and rejects or clamps larger requests, so back off until accepted.
    let mut size = recv_buffer_bytes;
    while size >= 64 << 10 && socket.set_recv_buffer_size(size).is_err() {
        size /= 2;
    }
    let actual = socket.recv_buffer_size()?;
    if actual < recv_buffer_bytes / 2 {
        eprintln!(
            "livox: receive buffer on port {} is {} KiB (asked for {} KiB); raise the OS limit to avoid drops",
            addr.port(),
            actual >> 10,
            recv_buffer_bytes >> 10
        );
    }
    socket.bind(&SocketAddr::V4(addr).into())?;
    let socket: UdpSocket = socket.into();
    // Bounds how long shutdown waits for a thread to notice `running`.
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;
    Ok(socket)
}

struct RecvContext {
    socket: UdpSocket,
    lidar_ip: Ipv4Addr,
    tx: SyncSender<Packet>,
    running: Arc<AtomicBool>,
    stats: Arc<Stats>,
    recorder: Option<Recorder>,
}

impl RecvContext {
    fn run(self) {
        let local = match self.socket.local_addr() {
            Ok(SocketAddr::V4(a)) => a,
            _ => SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0),
        };
        let mut recorder = self.recorder.clone();
        let mut buf = vec![0u8; 65536];
        let mut last_cnt: Option<(u8, u16)> = None;
        while self.running.load(Relaxed) {
            let (len, from) = match self.socket.recv_from(&mut buf) {
                Ok(r) => r,
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => continue,
                Err(e) => {
                    eprintln!("livox: recv failed: {e}");
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
            };
            let SocketAddr::V4(from) = from else { continue };
            if *from.ip() != self.lidar_ip {
                continue;
            }
            let buf = &buf[..len];
            if let Some(r) = &recorder {
                let now_ns = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_nanos() as u64;
                let result = r.lock().unwrap().write(now_ns, from, local, buf);
                if let Err(e) = result {
                    eprintln!("livox: recording stopped: {e}");
                    recorder = None;
                }
            }
            let packet = match data_packet::parse(buf) {
                Ok(p) => p,
                Err(_) => {
                    // Unknown until parsed, so charge the stream this socket carries.
                    self.stats.points.parse_errors.fetch_add(1, Relaxed);
                    continue;
                }
            };
            let header = packet.header();
            let stats = match packet {
                Packet::Points(_) => &self.stats.points,
                Packet::Imu { .. } => &self.stats.imu,
            };
            stats.packets.fetch_add(1, Relaxed);
            stats.points.fetch_add(header.dot_num as u64, Relaxed);
            if !data_packet::crc_ok(buf, header) {
                stats.crc_errors.fetch_add(1, Relaxed);
            }
            // udp_cnt counts packets; it may also restart at 0 when a new frame starts.
            let cnt = (header.frame_cnt, header.udp_cnt);
            if let Some((frame, udp)) = last_cnt {
                let follows = cnt.1 == udp.wrapping_add(1) || (cnt.0 != frame && cnt.1 == 0);
                if !follows {
                    stats.seq_gaps.fetch_add(1, Relaxed);
                }
            }
            last_cnt = Some(cnt);

            match self.tx.try_send(packet) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) => {
                    stats.dropped.fetch_add(1, Relaxed);
                }
                // Nobody is listening; keep draining the socket until shutdown.
                Err(TrySendError::Disconnected(_)) => {}
            }
        }
    }
}
