//! Point cloud / IMU UDP packets (Livox SDK2 protocol, e.g. Mid-360).
//!
//! These arrive at ~2 kHz, so they are decoded by hand from the byte slice
//! rather than through binrw: one pass, no per-field `Read` calls, and the
//! output is already in SI units.

use crate::crc::CRC_DATA;
use crate::datatype::{DataType, TimeType};

/// Size of the fixed header that precedes every data packet.
pub const HEADER_LEN: usize = 36;

/// Byte range covered by the header's `crc32` field (timestamp + payload).
const CRC_START: usize = 28;

#[derive(Debug, Clone)]
pub struct PacketHeader {
    pub version: u8,
    /// Total UDP payload length in bytes, header included.
    pub length: u16,
    /// Time spanned by the packet's points, in units of 0.1 us.
    pub time_interval: u16,
    pub dot_num: u16,
    /// Increments by one per packet on each stream; a gap means packets were lost.
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: DataType,
    pub time_type: TimeType,
    pub crc32: u32,
    /// Time of the first point, in ns. The epoch depends on `time_type`
    /// (time since power-on when unsynchronised).
    pub timestamp_ns: u64,
}

/// One lidar return, converted to metres in the sensor frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point {
    pub xyz: [f32; 3],
    pub reflectivity: u8,
    pub tag: u8,
    /// Time of this point relative to the packet's `timestamp_ns`.
    pub offset_ns: u32,
}

impl Point {
    /// The sensor reports "no return" as a point at the origin.
    pub fn is_valid(&self) -> bool {
        self.xyz != [0.0; 3]
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuSample {
    pub timestamp_ns: u64,
    /// Angular velocity, rad/s.
    pub gyro: [f32; 3],
    /// Specific force, in units of g (multiply by 9.81 for m/s^2).
    pub accel: [f32; 3],
}

#[derive(Debug, Clone)]
pub struct PointPacket {
    pub header: PacketHeader,
    pub points: Vec<Point>,
}

#[derive(Debug, Clone)]
pub enum Packet {
    Points(PointPacket),
    Imu { header: PacketHeader, sample: ImuSample },
}

impl Packet {
    pub fn header(&self) -> &PacketHeader {
        match self {
            Packet::Points(p) => &p.header,
            Packet::Imu { header, .. } => header,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum ParseError {
    TooShort(usize),
    LengthMismatch { header: usize, actual: usize },
    UnknownDataType(u8),
    UnknownTimeType(u8),
    PayloadSize { data_type: DataType, dot_num: u16, bytes: usize },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ParseError {}

fn u16_at(b: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([b[i], b[i + 1]])
}
fn u32_at(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}
fn i32_at(b: &[u8], i: usize) -> i32 {
    i32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}
fn i16_at(b: &[u8], i: usize) -> i16 {
    i16::from_le_bytes([b[i], b[i + 1]])
}
fn f32_at(b: &[u8], i: usize) -> f32 {
    f32::from_le_bytes(b[i..i + 4].try_into().unwrap())
}

impl DataType {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => DataType::IMUData,
            1 => DataType::PointCloudData1,
            2 => DataType::PointCloudData2,
            3 => DataType::PointCloudData3,
            _ => return None,
        })
    }

    /// Bytes per point (or per IMU sample) in the payload.
    pub fn point_size(self) -> usize {
        match self {
            DataType::IMUData => 24,
            DataType::PointCloudData1 => 14,
            DataType::PointCloudData2 => 8,
            DataType::PointCloudData3 => 10,
        }
    }
}

impl TimeType {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => TimeType::NoSync,
            1 => TimeType::GPTP,
            2 => TimeType::GPS,
            _ => return None,
        })
    }
}

impl PacketHeader {
    pub fn parse(buf: &[u8]) -> Result<Self, ParseError> {
        if buf.len() < HEADER_LEN {
            return Err(ParseError::TooShort(buf.len()));
        }
        let length = u16_at(buf, 1);
        if length as usize != buf.len() {
            return Err(ParseError::LengthMismatch {
                header: length as usize,
                actual: buf.len(),
            });
        }
        Ok(Self {
            version: buf[0],
            length,
            time_interval: u16_at(buf, 3),
            dot_num: u16_at(buf, 5),
            udp_cnt: u16_at(buf, 7),
            frame_cnt: buf[9],
            data_type: DataType::from_u8(buf[10]).ok_or(ParseError::UnknownDataType(buf[10]))?,
            time_type: TimeType::from_u8(buf[11]).ok_or(ParseError::UnknownTimeType(buf[11]))?,
            crc32: u32_at(buf, 24),
            timestamp_ns: u64::from_le_bytes(buf[28..36].try_into().unwrap()),
        })
    }

    /// Time spanned by the packet's points, in ns.
    pub fn time_interval_ns(&self) -> u32 {
        self.time_interval as u32 * 100
    }
}

/// Checks the header's crc32, which covers the timestamp and payload.
pub fn crc_ok(buf: &[u8], header: &PacketHeader) -> bool {
    CRC_DATA.checksum(&buf[CRC_START..]) == header.crc32
}

/// Decodes one UDP payload from the point cloud or IMU port.
pub fn parse(buf: &[u8]) -> Result<Packet, ParseError> {
    let header = PacketHeader::parse(buf)?;
    let payload = &buf[HEADER_LEN..];
    let n = header.dot_num as usize;
    let size = header.data_type.point_size();
    if payload.len() != n * size {
        return Err(ParseError::PayloadSize {
            data_type: header.data_type,
            dot_num: header.dot_num,
            bytes: payload.len(),
        });
    }

    if header.data_type == DataType::IMUData {
        if n != 1 {
            return Err(ParseError::PayloadSize {
                data_type: header.data_type,
                dot_num: header.dot_num,
                bytes: payload.len(),
            });
        }
        let p = payload;
        let sample = ImuSample {
            timestamp_ns: header.timestamp_ns,
            gyro: [f32_at(p, 0), f32_at(p, 4), f32_at(p, 8)],
            accel: [f32_at(p, 12), f32_at(p, 16), f32_at(p, 20)],
        };
        return Ok(Packet::Imu { header, sample });
    }

    // Points are evenly spaced over the packet's time interval.
    let dt_ns = if n > 0 { header.time_interval_ns() / n as u32 } else { 0 };
    let chunks = payload.chunks_exact(size).zip((0u32..).map(|i| i * dt_ns));
    let points: Vec<Point> = match header.data_type {
        DataType::PointCloudData1 => chunks
            .map(|(c, offset_ns)| Point {
                xyz: [
                    i32_at(c, 0) as f32 / 1000.0,
                    i32_at(c, 4) as f32 / 1000.0,
                    i32_at(c, 8) as f32 / 1000.0,
                ],
                reflectivity: c[12],
                tag: c[13],
                offset_ns,
            })
            .collect(),
        DataType::PointCloudData2 => chunks
            .map(|(c, offset_ns)| Point {
                xyz: [
                    i16_at(c, 0) as f32 / 100.0,
                    i16_at(c, 2) as f32 / 100.0,
                    i16_at(c, 4) as f32 / 100.0,
                ],
                reflectivity: c[6],
                tag: c[7],
                offset_ns,
            })
            .collect(),
        DataType::PointCloudData3 => chunks
            .map(|(c, offset_ns)| {
                // depth in mm; theta (from +z) and phi (about +z) in 0.01 deg.
                let depth = u32_at(c, 0) as f32 / 1000.0;
                let theta = (u16_at(c, 4) as f32 * 0.01).to_radians();
                let phi = (u16_at(c, 6) as f32 * 0.01).to_radians();
                let (st, ct) = theta.sin_cos();
                let (sp, cp) = phi.sin_cos();
                Point {
                    xyz: [depth * st * cp, depth * st * sp, depth * ct],
                    reflectivity: c[8],
                    tag: c[9],
                    offset_ns,
                }
            })
            .collect(),
        DataType::IMUData => unreachable!(),
    };
    Ok(Packet::Points(PointPacket { header, points }))
}

/// Fields of an outgoing packet header that `encode_*` does not derive.
#[derive(Debug, Clone, Copy, Default)]
pub struct EncodeHeader {
    pub timestamp_ns: u64,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub time_type: u8,
}

fn encode(h: EncodeHeader, data_type: DataType, dot_num: u16, time_interval: u16, payload: &[u8]) -> Vec<u8> {
    let mut b = vec![0u8; HEADER_LEN];
    b[1..3].copy_from_slice(&((HEADER_LEN + payload.len()) as u16).to_le_bytes());
    b[3..5].copy_from_slice(&time_interval.to_le_bytes());
    b[5..7].copy_from_slice(&dot_num.to_le_bytes());
    b[7..9].copy_from_slice(&h.udp_cnt.to_le_bytes());
    b[9] = h.frame_cnt;
    b[10] = data_type as u8;
    b[11] = h.time_type;
    b[28..36].copy_from_slice(&h.timestamp_ns.to_le_bytes());
    b.extend_from_slice(payload);
    let crc = CRC_DATA.checksum(&b[CRC_START..]);
    b[24..28].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Encodes a point packet (Cartesian, mm precision) as the lidar would send
/// it: the inverse of `parse`. `time_interval` is in units of 0.1 us; `parse`
/// spaces the points evenly over it (each point's `offset_ns` is ignored here).
pub fn encode_points(h: EncodeHeader, time_interval: u16, points: &[Point]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(points.len() * 14);
    for p in points {
        for c in p.xyz {
            payload.extend_from_slice(&((c * 1000.0).round() as i32).to_le_bytes());
        }
        payload.extend_from_slice(&[p.reflectivity, p.tag]);
    }
    encode(h, DataType::PointCloudData1, points.len() as u16, time_interval, &payload)
}

/// Encodes an IMU packet: the inverse of `parse`.
pub fn encode_imu(h: EncodeHeader, sample: &ImuSample) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for v in sample.gyro.iter().chain(&sample.accel) {
        payload.extend_from_slice(&v.to_le_bytes());
    }
    encode(h, DataType::IMUData, 1, 0, &payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(data_type: u8, time_interval: u16, payload: &[u8], dot_num: u16) -> Vec<u8> {
        let mut b = vec![0u8; HEADER_LEN];
        b[0] = 0;
        b[1..3].copy_from_slice(&((HEADER_LEN + payload.len()) as u16).to_le_bytes());
        b[3..5].copy_from_slice(&time_interval.to_le_bytes());
        b[5..7].copy_from_slice(&dot_num.to_le_bytes());
        b[7..9].copy_from_slice(&7u16.to_le_bytes());
        b[10] = data_type;
        b[11] = 0;
        b[28..36].copy_from_slice(&123_456_789u64.to_le_bytes());
        b.extend_from_slice(payload);
        let crc = CRC_DATA.checksum(&b[CRC_START..]);
        b[24..28].copy_from_slice(&crc.to_le_bytes());
        b
    }

    #[test]
    fn cartesian_high() {
        let mut payload = Vec::new();
        for (x, y, z, r) in [(1000i32, -2000i32, 3500i32, 10u8), (0, 0, 0, 0)] {
            payload.extend_from_slice(&x.to_le_bytes());
            payload.extend_from_slice(&y.to_le_bytes());
            payload.extend_from_slice(&z.to_le_bytes());
            payload.extend_from_slice(&[r, 0x10]);
        }
        let buf = packet(1, 4800, &payload, 2);
        let Packet::Points(p) = parse(&buf).unwrap() else { panic!() };
        assert!(crc_ok(&buf, &p.header));
        assert_eq!(p.header.udp_cnt, 7);
        assert_eq!(p.header.timestamp_ns, 123_456_789);
        assert_eq!(p.points.len(), 2);
        assert_eq!(p.points[0].xyz, [1.0, -2.0, 3.5]);
        assert_eq!(p.points[0].reflectivity, 10);
        assert_eq!(p.points[0].tag, 0x10);
        assert_eq!(p.points[0].offset_ns, 0);
        assert_eq!(p.points[1].offset_ns, 240_000);
        assert!(!p.points[1].is_valid());
    }

    #[test]
    fn spherical() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&2000u32.to_le_bytes());
        payload.extend_from_slice(&9000u16.to_le_bytes()); // theta 90 deg
        payload.extend_from_slice(&9000u16.to_le_bytes()); // phi 90 deg
        payload.extend_from_slice(&[5, 0]);
        let Packet::Points(p) = parse(&packet(3, 0, &payload, 1)).unwrap() else { panic!() };
        let [x, y, z] = p.points[0].xyz;
        assert!(x.abs() < 1e-5 && (y - 2.0).abs() < 1e-5 && z.abs() < 1e-5);
    }

    #[test]
    fn imu() {
        let mut payload = Vec::new();
        for v in [0.1f32, 0.2, 0.3, 0.0, 0.0, 1.0] {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        let Packet::Imu { sample, .. } = parse(&packet(0, 0, &payload, 1)).unwrap() else {
            panic!()
        };
        assert_eq!(sample.gyro, [0.1, 0.2, 0.3]);
        assert_eq!(sample.accel, [0.0, 0.0, 1.0]);
        assert_eq!(sample.timestamp_ns, 123_456_789);
    }

    #[test]
    fn rejects_malformed() {
        assert_eq!(parse(&[0u8; 10]).unwrap_err(), ParseError::TooShort(10));
        let mut buf = packet(1, 0, &[0u8; 14], 1);
        buf.pop();
        assert!(matches!(parse(&buf), Err(ParseError::LengthMismatch { .. })));
        let buf = packet(1, 0, &[0u8; 14], 2);
        assert!(matches!(parse(&buf), Err(ParseError::PayloadSize { .. })));
    }
}
