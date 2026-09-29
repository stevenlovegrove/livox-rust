//! Decoding of the ROS 2 messages Livox datasets carry, from CDR (the ROS 2
//! wire and bag serialisation). Written out by hand: two message types do not
//! justify a ROS or IDL dependency.

use anyhow::{bail, ensure, Context};

/// Little-endian CDR reader. Primitives are aligned to their size, counted
/// from the end of the 4-byte encapsulation header.
struct Cdr<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Cdr<'a> {
    fn new(data: &'a [u8]) -> anyhow::Result<Self> {
        ensure!(data.len() >= 4, "message shorter than its CDR header");
        // Encapsulation kind 0x0001 is CDR little-endian (0x0000 big-endian).
        ensure!(data[0] == 0 && data[1] == 1, "not little-endian CDR (kind {:02x}{:02x})", data[0], data[1]);
        Ok(Self { b: &data[4..], pos: 0 })
    }

    fn take(&mut self, align: usize, n: usize) -> anyhow::Result<&'a [u8]> {
        self.pos = self.pos.next_multiple_of(align);
        let s = self.b.get(self.pos..self.pos + n).context("message truncated")?;
        self.pos += n;
        Ok(s)
    }
    fn u8(&mut self) -> anyhow::Result<u8> {
        Ok(self.take(1, 1)?[0])
    }
    fn u32(&mut self) -> anyhow::Result<u32> {
        Ok(u32::from_le_bytes(self.take(4, 4)?.try_into()?))
    }
    fn i32(&mut self) -> anyhow::Result<i32> {
        Ok(i32::from_le_bytes(self.take(4, 4)?.try_into()?))
    }
    fn u64(&mut self) -> anyhow::Result<u64> {
        Ok(u64::from_le_bytes(self.take(8, 8)?.try_into()?))
    }
    fn f32(&mut self) -> anyhow::Result<f32> {
        Ok(f32::from_le_bytes(self.take(4, 4)?.try_into()?))
    }
    fn f64(&mut self) -> anyhow::Result<f64> {
        Ok(f64::from_le_bytes(self.take(8, 8)?.try_into()?))
    }
    fn string(&mut self) -> anyhow::Result<String> {
        let n = self.u32()? as usize; // includes the trailing NUL
        let s = self.take(1, n)?;
        Ok(String::from_utf8_lossy(s.strip_suffix(&[0]).unwrap_or(s)).into_owned())
    }
    fn vec3(&mut self) -> anyhow::Result<[f64; 3]> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }
    fn skip_f64s(&mut self, n: usize) -> anyhow::Result<()> {
        self.take(8, 8 * n).map(|_| ())
    }
    /// Checks the whole message was consumed (up to final padding).
    fn finish(&self, what: &str) -> anyhow::Result<()> {
        let left = self.b.len() - self.pos;
        ensure!(left < 4, "{what}: {left} bytes left over; message layout differs from expected");
        Ok(())
    }
}

/// std_msgs/Header stamp, in ns.
fn header(c: &mut Cdr) -> anyhow::Result<u64> {
    let sec = c.i32()?;
    let nanosec = c.u32()?;
    c.string()?; // frame_id
    ensure!(sec >= 0, "negative header stamp");
    Ok(sec as u64 * 1_000_000_000 + nanosec as u64)
}

/// livox_ros_driver2/msg/CustomPoint.
#[derive(Debug, Clone, Copy)]
pub struct CustomPoint {
    /// ns after the message's `timebase`.
    pub offset_time: u32,
    pub xyz: [f32; 3],
    pub reflectivity: u8,
    pub tag: u8,
}

/// livox_ros_driver2/msg/CustomMsg (livox_ros_driver's is the same shape).
#[derive(Debug, Clone)]
pub struct CustomMsg {
    /// Time of the first point, ns.
    pub timebase: u64,
    pub points: Vec<CustomPoint>,
}

pub fn custom_msg(data: &[u8]) -> anyhow::Result<CustomMsg> {
    let mut c = Cdr::new(data)?;
    header(&mut c)?;
    let timebase = c.u64()?;
    let point_num = c.u32()?;
    c.take(1, 4)?; // lidar_id, rsvd
    let n = c.u32()? as usize;
    if n != point_num as usize {
        bail!("point_num {point_num} but {n} points");
    }
    let mut points = Vec::with_capacity(n);
    for _ in 0..n {
        points.push(CustomPoint {
            offset_time: c.u32()?,
            xyz: [c.f32()?, c.f32()?, c.f32()?],
            reflectivity: c.u8()?,
            tag: c.u8()?,
        });
        c.u8()?; // line: the laser that fired, not in the lidar's packets
    }
    c.finish("CustomMsg")?;
    Ok(CustomMsg { timebase, points })
}

/// sensor_msgs/msg/Imu, without orientation and covariances.
#[derive(Debug, Clone, Copy)]
pub struct Imu {
    pub stamp_ns: u64,
    pub angular_velocity: [f64; 3],
    pub linear_acceleration: [f64; 3],
}

pub fn imu(data: &[u8]) -> anyhow::Result<Imu> {
    let mut c = Cdr::new(data)?;
    let stamp_ns = header(&mut c)?;
    c.skip_f64s(4 + 9)?; // orientation, covariance
    let angular_velocity = c.vec3()?;
    c.skip_f64s(9)?;
    let linear_acceleration = c.vec3()?;
    c.skip_f64s(9)?;
    c.finish("Imu")?;
    Ok(Imu {
        stamp_ns,
        angular_velocity,
        linear_acceleration,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A CDR writer mirroring `Cdr`, for building test messages.
    #[derive(Default)]
    struct W(Vec<u8>);
    impl W {
        fn pad(&mut self, a: usize) {
            while !(self.0.len() - 4).is_multiple_of(a) {
                self.0.push(0);
            }
        }
        fn put(&mut self, a: usize, b: &[u8]) {
            self.pad(a);
            self.0.extend_from_slice(b);
        }
        fn header(&mut self, sec: i32, nsec: u32, frame: &str) {
            self.put(4, &sec.to_le_bytes());
            self.put(4, &nsec.to_le_bytes());
            self.put(4, &(frame.len() as u32 + 1).to_le_bytes());
            self.put(1, frame.as_bytes());
            self.put(1, &[0]);
        }
    }

    #[test]
    fn decodes_custom_msg() {
        let mut w = W(vec![0, 1, 0, 0]);
        w.header(12, 345, "livox_frame");
        w.put(8, &1_000_000_123u64.to_le_bytes());
        w.put(4, &2u32.to_le_bytes());
        w.put(1, &[7, 0, 0, 0]); // lidar_id, rsvd
        w.put(4, &2u32.to_le_bytes());
        for (t, x) in [(0u32, 1.5f32), (5000, -2.0)] {
            w.put(4, &t.to_le_bytes());
            for v in [x, 0.25, 3.0] {
                w.put(4, &v.to_le_bytes());
            }
            w.put(1, &[40, 0x10, 3]);
        }
        let m = custom_msg(&w.0).unwrap();
        assert_eq!(m.timebase, 1_000_000_123);
        assert_eq!(m.points.len(), 2);
        assert_eq!(m.points[1].offset_time, 5000);
        assert_eq!(m.points[1].xyz, [-2.0, 0.25, 3.0]);
        assert_eq!((m.points[1].reflectivity, m.points[1].tag), (40, 0x10));
    }

    #[test]
    fn decodes_imu() {
        let mut w = W(vec![0, 1, 0, 0]);
        w.header(1, 2, "imu");
        let f = |w: &mut W, vs: &[f64]| vs.iter().for_each(|v| w.put(8, &v.to_le_bytes()));
        f(&mut w, &[0.0, 0.0, 0.0, 1.0]);
        f(&mut w, &[0.0; 9]);
        f(&mut w, &[0.1, 0.2, 0.3]);
        f(&mut w, &[0.0; 9]);
        f(&mut w, &[0.0, 0.0, 9.8]);
        f(&mut w, &[0.0; 9]);
        let m = imu(&w.0).unwrap();
        assert_eq!(m.stamp_ns, 1_000_000_002);
        assert_eq!(m.angular_velocity, [0.1, 0.2, 0.3]);
        assert_eq!(m.linear_acceleration, [0.0, 0.0, 9.8]);
    }
}
