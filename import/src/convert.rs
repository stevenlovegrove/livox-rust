//! Turns decoded dataset messages back into the lidar's UDP packets and
//! writes them, in time order, to a pcap recording.
//!
//! A driver-assembled frame (`CustomMsg`) gives each point its own time. The
//! lidar's packets instead carry one timestamp and space their points evenly
//! over `time_interval`. The driver builds a frame by appending packets in
//! the order they arrived, which is not always time order, so a frame is cut,
//! in its own order, into runs of evenly spaced points: each run is one of the
//! original packets (a new one starts wherever the spacing breaks). That
//! reproduces every point's time to within the packet format's 0.1 us
//! resolution; `Report` states the error actually reached. The reorder
//! buffer then writes whole packets in time order.

use std::{
    cmp::Reverse,
    collections::BinaryHeap,
    io::Write,
    net::{Ipv4Addr, SocketAddrV4},
};

use livox_rust::{
    data_packet::{encode_imu, encode_points, EncodeHeader},
    pcap, ImuSample, Point,
};

use crate::ros::{CustomMsg, Imu};

/// Points per packet, as a Mid-360 sends them.
const PACKET_POINTS: usize = 96;
/// A point further than this from the packet's even spacing starts a new packet.
const SPACING_TOLERANCE_NS: i64 = 1000;
/// Largest point spacing a full packet can express (`time_interval` is a u16 of 0.1 us).
const MAX_SPACING_NS: u64 = u16::MAX as u64 * 100 / PACKET_POINTS as u64;
/// Packets are held this long (in data time) to put the two streams in order.
const REORDER_WINDOW_NS: u64 = 2_000_000_000;
const STANDARD_GRAVITY: f64 = 9.80665;

/// Addresses written into the recording (Mid-360 defaults); replay ignores them.
const LIDAR_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 100);
const HOST_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 50);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccelUnits {
    /// Decide from the first sample's magnitude.
    Auto,
    /// Already in g, as livox_ros_driver2 publishes it.
    G,
    MetresPerSecond2,
}

#[derive(Debug, Default)]
pub struct Report {
    pub frames: u64,
    pub points: u64,
    pub point_packets: u64,
    pub imu_samples: u64,
    /// Points whose time the packets reproduce, and the worst error.
    pub max_time_error_ns: u64,
    pub sum_time_error_ns: u64,
    /// Frames whose packets the driver had appended out of time order.
    pub unsorted_frames: u64,
    pub accel_scale: Option<f64>,
    pub first_ns: Option<u64>,
    pub last_ns: u64,
}

enum Pending {
    Points { t: u64, time_interval: u16, frame_cnt: u8, points: Vec<Point> },
    Imu(ImuSample),
}

impl Pending {
    fn time(&self) -> u64 {
        match self {
            Pending::Points { t, .. } => *t,
            Pending::Imu(s) => s.timestamp_ns,
        }
    }
}

pub struct Converter<W: Write> {
    out: pcap::Writer<W>,
    accel_units: AccelUnits,
    /// Min-heap on (time, arrival order).
    queue: BinaryHeap<Reverse<(u64, u64, usize)>>,
    slots: Vec<Option<Pending>>,
    free: Vec<usize>,
    arrivals: u64,
    newest: u64,
    frame_cnt: u8,
    udp_cnt_points: u16,
    udp_cnt_imu: u16,
    pub report: Report,
}

impl<W: Write> Converter<W> {
    pub fn new(out: pcap::Writer<W>, accel_units: AccelUnits) -> Self {
        Self {
            out,
            accel_units,
            queue: BinaryHeap::new(),
            slots: Vec::new(),
            free: Vec::new(),
            arrivals: 0,
            newest: 0,
            frame_cnt: 0,
            udp_cnt_points: 0,
            udp_cnt_imu: 0,
            report: Report::default(),
        }
    }

    pub fn push_lidar(&mut self, msg: &CustomMsg) -> std::io::Result<()> {
        self.report.frames += 1;
        self.report.points += msg.points.len() as u64;
        let frame_cnt = self.frame_cnt;
        self.frame_cnt = self.frame_cnt.wrapping_add(1);

        let pts: Vec<(u64, Point)> = msg
            .points
            .iter()
            .map(|p| {
                let point = Point {
                    xyz: p.xyz,
                    reflectivity: p.reflectivity,
                    tag: p.tag,
                    offset_ns: 0,
                };
                (msg.timebase + p.offset_time as u64, point)
            })
            .collect();
        if !pts.is_sorted_by_key(|p| p.0) {
            self.report.unsorted_frames += 1;
        }

        let mut start = 0;
        while start < pts.len() {
            let len = even_run(&pts[start..]);
            let chunk = &pts[start..start + len];
            let t0 = chunk[0].0;
            let spacing = if len > 1 { (chunk[len - 1].0 - t0) as f64 / (len - 1) as f64 } else { 0.0 };
            let time_interval = (spacing * len as f64 / 100.0).round() as u16;
            // What `data_packet::parse` will make of it.
            let dt = time_interval as u64 * 100 / len as u64;
            for (k, (t, _)) in chunk.iter().enumerate() {
                let err = (t0 + k as u64 * dt).abs_diff(*t);
                self.report.max_time_error_ns = self.report.max_time_error_ns.max(err);
                self.report.sum_time_error_ns += err;
            }
            let points = chunk.iter().map(|p| p.1).collect();
            self.enqueue(Pending::Points { t: t0, time_interval, frame_cnt, points })?;
            start += len;
        }
        Ok(())
    }

    pub fn push_imu(&mut self, msg: &Imu) -> std::io::Result<()> {
        let scale = *self.report.accel_scale.get_or_insert_with(|| {
            let a = msg.linear_acceleration;
            let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            match self.accel_units {
                AccelUnits::G => 1.0,
                AccelUnits::MetresPerSecond2 => 1.0 / STANDARD_GRAVITY,
                // At rest the norm is ~1 g; it is only near 9.8 in m/s^2.
                AccelUnits::Auto if norm > 4.0 => 1.0 / STANDARD_GRAVITY,
                AccelUnits::Auto => 1.0,
            }
        });
        self.report.imu_samples += 1;
        let sample = ImuSample {
            timestamp_ns: msg.stamp_ns,
            gyro: msg.angular_velocity.map(|v| v as f32),
            accel: msg.linear_acceleration.map(|v| (v * scale) as f32),
        };
        self.enqueue(Pending::Imu(sample))
    }

    fn enqueue(&mut self, p: Pending) -> std::io::Result<()> {
        let t = p.time();
        let slot = match self.free.pop() {
            Some(i) => {
                self.slots[i] = Some(p);
                i
            }
            None => {
                self.slots.push(Some(p));
                self.slots.len() - 1
            }
        };
        self.queue.push(Reverse((t, self.arrivals, slot)));
        self.arrivals += 1;
        self.newest = self.newest.max(t);
        while self.queue.peek().is_some_and(|Reverse((t, ..))| t + REORDER_WINDOW_NS < self.newest) {
            self.write_next()?;
        }
        Ok(())
    }

    fn write_next(&mut self) -> std::io::Result<()> {
        let Some(Reverse((t, _, slot))) = self.queue.pop() else { return Ok(()) };
        let pending = self.slots[slot].take().unwrap();
        self.free.push(slot);
        if t < self.report.last_ns {
            // Arrived later than the reorder window allows; keep the file monotonic.
            eprintln!("warning: packet at {t} ns is {} ms out of order", (self.report.last_ns - t) / 1_000_000);
        }
        self.report.first_ns.get_or_insert(t);
        self.report.last_ns = self.report.last_ns.max(t);

        let (payload, src_port, dst_port) = match pending {
            Pending::Points { t, time_interval, frame_cnt, points } => {
                let h = EncodeHeader { timestamp_ns: t, udp_cnt: self.udp_cnt_points, frame_cnt, time_type: 0 };
                self.udp_cnt_points = self.udp_cnt_points.wrapping_add(1);
                self.report.point_packets += 1;
                (encode_points(h, time_interval, &points), 56300, 56301)
            }
            Pending::Imu(sample) => {
                let h = EncodeHeader {
                    timestamp_ns: sample.timestamp_ns,
                    udp_cnt: self.udp_cnt_imu,
                    ..Default::default()
                };
                self.udp_cnt_imu = self.udp_cnt_imu.wrapping_add(1);
                (encode_imu(h, &sample), 56400, 56401)
            }
        };
        // The data time doubles as the capture time, which paces replay.
        self.out.write(
            t,
            SocketAddrV4::new(LIDAR_IP, src_port),
            SocketAddrV4::new(HOST_IP, dst_port),
            &payload,
        )
    }

    /// Writes everything still held and flushes the file.
    pub fn finish(mut self) -> std::io::Result<(Report, W)> {
        while !self.queue.is_empty() {
            self.write_next()?;
        }
        self.out.flush()?;
        Ok((self.report, self.out.into_inner()))
    }
}

/// Length of the leading run of `pts` that one packet can carry: at most
/// `PACKET_POINTS`, evenly spaced (forwards in time) to within tolerance.
fn even_run(pts: &[(u64, Point)]) -> usize {
    let n = pts.len().min(PACKET_POINTS);
    if n < 2 {
        return n;
    }
    let t0 = pts[0].0;
    let dt = match pts[1].0.checked_sub(t0) {
        Some(dt) if dt <= MAX_SPACING_NS => dt,
        _ => return 1,
    };
    (2..n)
        .find(|&k| (pts[k].0 as i64 - (t0 + k as u64 * dt) as i64).abs() > SPACING_TOLERANCE_NS)
        .unwrap_or(n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::CustomPoint;
    use livox_rust::{data_packet, Packet};

    fn msg(offsets: &[u32]) -> CustomMsg {
        CustomMsg {
            timebase: 1_000_000_000,
            points: offsets
                .iter()
                .enumerate()
                .map(|(i, &offset_time)| CustomPoint {
                    offset_time,
                    xyz: [i as f32 * 0.001, 1.0, -2.5],
                    reflectivity: i as u8,
                    tag: 0,
                })
                .collect(),
        }
    }

    fn convert(m: &CustomMsg) -> (Report, Vec<Packet>) {
        let mut c = Converter::new(pcap::Writer::new(Vec::new()).unwrap(), AccelUnits::Auto);
        c.push_lidar(m).unwrap();
        let (report, data) = c.finish().unwrap();
        let packets = pcap::Reader::new(&data[..])
            .unwrap()
            .map(|d| data_packet::parse(&d.unwrap().payload).unwrap())
            .collect();
        (report, packets)
    }

    #[test]
    fn evenly_spaced_frame_round_trips_exactly() {
        // 200 points at 5 us: two full packets and a short one.
        let offsets: Vec<u32> = (0..200).map(|i| i * 5000).collect();
        let (report, packets) = convert(&msg(&offsets));
        assert_eq!(packets.len(), 3);
        assert_eq!(report.max_time_error_ns, 0);
        let mut k = 0;
        for p in packets {
            let Packet::Points(p) = p else { panic!() };
            for pt in &p.points {
                assert_eq!(p.header.timestamp_ns + pt.offset_ns as u64, 1_000_000_000 + offsets[k] as u64);
                assert_eq!(pt.reflectivity, k as u8);
                assert!((pt.xyz[0] - k as f32 * 0.001).abs() < 1e-6);
                k += 1;
            }
        }
        assert_eq!(k, 200);
    }

    #[test]
    fn packets_appended_out_of_order_are_kept_whole() {
        // Three 96-point packets, appended to the frame as 0, 2, 1.
        let packet = |k: u32| (0..96).map(move |i| (k * 96 + i) * 4947);
        let offsets: Vec<u32> = packet(0).chain(packet(2)).chain(packet(1)).collect();
        let (report, packets) = convert(&msg(&offsets));
        assert_eq!(report.unsorted_frames, 1);
        assert!(report.max_time_error_ns < 100, "{}", report.max_time_error_ns);
        // Written in time order, whole.
        let starts: Vec<u64> = packets
            .iter()
            .map(|p| match p {
                Packet::Points(p) => {
                    assert_eq!(p.points.len(), 96);
                    p.header.timestamp_ns
                }
                _ => panic!(),
            })
            .collect();
        assert_eq!(starts, [0, 1, 2].map(|k| 1_000_000_000 + k * 96 * 4947));
    }

    #[test]
    fn gap_starts_a_new_packet() {
        // 10 points, then 3 missing, then 10 more.
        let offsets: Vec<u32> = (0..10).chain(13..23).map(|i| i * 5000).collect();
        let (report, packets) = convert(&msg(&offsets));
        assert_eq!(packets.len(), 2);
        assert_eq!(report.max_time_error_ns, 0);
    }
}
