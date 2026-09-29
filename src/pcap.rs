//! Classic libpcap files of UDP datagrams: the recording format.
//!
//! A recording is the lidar's UDP traffic exactly as it arrived, so replaying
//! it exercises the same decode path as a live device. The format is the one
//! `tcpdump -w` writes and Wireshark reads, so a capture taken with
//! `tcpdump -i <iface> -w out.pcap udp` can be replayed too.
//!
//! Writing produces Ethernet/IPv4/UDP frames with nanosecond timestamps.
//! Reading accepts either timestamp resolution and byte order, and the link
//! types tcpdump produces on macOS and Linux (Ethernet, BSD loopback, raw IP,
//! Linux cooked v1/v2).

use std::{
    fs::File,
    io::{self, BufReader, BufWriter, Read, Write},
    net::{Ipv4Addr, SocketAddrV4},
    path::Path,
};

const MAGIC_US: u32 = 0xA1B2_C3D4;
const MAGIC_NS: u32 = 0xA1B2_3C4D;

const LINKTYPE_NULL: u32 = 0;
const LINKTYPE_ETHERNET: u32 = 1;
const LINKTYPE_RAW: u32 = 101;
const LINKTYPE_LINUX_SLL: u32 = 113;
const LINKTYPE_IPV4: u32 = 228;
const LINKTYPE_LINUX_SLL2: u32 = 276;

const ETH_LEN: usize = 14;
const IP_LEN: usize = 20;
const UDP_LEN: usize = 8;

/// One UDP datagram and when it was captured.
#[derive(Debug, Clone, PartialEq)]
pub struct Datagram {
    /// Capture time, ns since the Unix epoch (host clock, not the lidar's).
    pub capture_ns: u64,
    pub src: SocketAddrV4,
    pub dst: SocketAddrV4,
    pub payload: Vec<u8>,
}

pub struct Writer<W: Write> {
    out: W,
    ip_id: u16,
}

impl Writer<BufWriter<File>> {
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::new(BufWriter::with_capacity(1 << 20, File::create(path)?))
    }
}

impl<W: Write> Writer<W> {
    pub fn new(mut out: W) -> io::Result<Self> {
        out.write_all(&MAGIC_NS.to_le_bytes())?;
        out.write_all(&2u16.to_le_bytes())?; // version 2.4
        out.write_all(&4u16.to_le_bytes())?;
        out.write_all(&0i32.to_le_bytes())?; // thiszone
        out.write_all(&0u32.to_le_bytes())?; // sigfigs
        out.write_all(&65535u32.to_le_bytes())?; // snaplen
        out.write_all(&LINKTYPE_ETHERNET.to_le_bytes())?;
        Ok(Self { out, ip_id: 0 })
    }

    pub fn write(&mut self, capture_ns: u64, src: SocketAddrV4, dst: SocketAddrV4, payload: &[u8]) -> io::Result<()> {
        let udp_len = UDP_LEN + payload.len();
        let ip_len = IP_LEN + udp_len;
        let frame_len = ETH_LEN + ip_len;
        if ip_len > u16::MAX as usize {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "datagram too large"));
        }

        let mut h = [0u8; ETH_LEN + IP_LEN + UDP_LEN];
        // Ethernet: locally administered MACs, IPv4 ethertype.
        h[0..6].copy_from_slice(&[0x02, 0, 0, 0, 0, 0x01]);
        h[6..12].copy_from_slice(&[0x02, 0, 0, 0, 0, 0x02]);
        h[12..14].copy_from_slice(&0x0800u16.to_be_bytes());
        let ip = &mut h[ETH_LEN..ETH_LEN + IP_LEN];
        ip[0] = 0x45;
        ip[2..4].copy_from_slice(&(ip_len as u16).to_be_bytes());
        ip[4..6].copy_from_slice(&self.ip_id.to_be_bytes());
        ip[8] = 64; // ttl
        ip[9] = 17; // udp
        ip[12..16].copy_from_slice(&src.ip().octets());
        ip[16..20].copy_from_slice(&dst.ip().octets());
        let checksum = ipv4_checksum(ip);
        ip[10..12].copy_from_slice(&checksum.to_be_bytes());
        let udp = &mut h[ETH_LEN + IP_LEN..];
        udp[0..2].copy_from_slice(&src.port().to_be_bytes());
        udp[2..4].copy_from_slice(&dst.port().to_be_bytes());
        udp[4..6].copy_from_slice(&(udp_len as u16).to_be_bytes());
        // UDP checksum 0: not computed (allowed for IPv4).
        self.ip_id = self.ip_id.wrapping_add(1);

        let o = &mut self.out;
        o.write_all(&((capture_ns / 1_000_000_000) as u32).to_le_bytes())?;
        o.write_all(&((capture_ns % 1_000_000_000) as u32).to_le_bytes())?;
        o.write_all(&(frame_len as u32).to_le_bytes())?;
        o.write_all(&(frame_len as u32).to_le_bytes())?;
        o.write_all(&h)?;
        o.write_all(payload)
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }

    pub fn into_inner(self) -> W {
        self.out
    }
}

fn ipv4_checksum(header: &[u8]) -> u16 {
    let mut sum: u32 = header
        .as_chunks::<2>().0.iter()
        .map(|c| u16::from_be_bytes([c[0], c[1]]) as u32)
        .sum();
    while sum > 0xFFFF {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

pub struct Reader<R: Read> {
    input: R,
    swapped: bool,
    nanos: bool,
    linktype: u32,
    frame: Vec<u8>,
    /// Frames skipped because they were not complete, unfragmented IPv4/UDP.
    pub skipped: u64,
}

impl Reader<BufReader<File>> {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::new(BufReader::with_capacity(1 << 20, File::open(path)?))
    }
}

fn invalid(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

impl<R: Read> Reader<R> {
    pub fn new(mut input: R) -> io::Result<Self> {
        let mut h = [0u8; 24];
        input.read_exact(&mut h)?;
        let magic = u32::from_le_bytes(h[0..4].try_into().unwrap());
        let (swapped, nanos) = match magic {
            MAGIC_US => (false, false),
            MAGIC_NS => (false, true),
            m if m.swap_bytes() == MAGIC_US => (true, false),
            m if m.swap_bytes() == MAGIC_NS => (true, true),
            _ => return Err(invalid("not a pcap file (pcapng is not supported; convert with `editcap -F pcap`)")),
        };
        let mut reader = Self {
            input,
            swapped,
            nanos,
            linktype: 0,
            frame: Vec::new(),
            skipped: 0,
        };
        reader.linktype = reader.u32(&h[20..24]) & 0x0FFF_FFFF;
        match reader.linktype {
            LINKTYPE_NULL | LINKTYPE_ETHERNET | LINKTYPE_RAW | LINKTYPE_IPV4 | LINKTYPE_LINUX_SLL
            | LINKTYPE_LINUX_SLL2 => Ok(reader),
            other => Err(invalid(format!("unsupported pcap link type {other}"))),
        }
    }

    fn u32(&self, b: &[u8]) -> u32 {
        let v = u32::from_le_bytes(b.try_into().unwrap());
        if self.swapped { v.swap_bytes() } else { v }
    }

    /// The next UDP datagram, or `None` at the end of the file.
    pub fn next_datagram(&mut self) -> io::Result<Option<Datagram>> {
        loop {
            let mut rec = [0u8; 16];
            match self.input.read_exact(&mut rec) {
                Ok(()) => {}
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
                Err(e) => return Err(e),
            }
            let sec = self.u32(&rec[0..4]) as u64;
            let frac = self.u32(&rec[4..8]) as u64;
            let incl_len = self.u32(&rec[8..12]) as usize;
            let orig_len = self.u32(&rec[12..16]) as usize;
            if incl_len > 1 << 24 {
                return Err(invalid(format!("implausible record length {incl_len}")));
            }
            self.frame.resize(incl_len, 0);
            match self.input.read_exact(&mut self.frame) {
                Ok(()) => {}
                // A capture cut off mid-record (e.g. tcpdump killed) just ends.
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
                Err(e) => return Err(e),
            }
            let capture_ns = sec * 1_000_000_000 + if self.nanos { frac } else { frac * 1000 };
            if incl_len < orig_len {
                self.skipped += 1;
                continue;
            }
            match self.decode(capture_ns) {
                Some(d) => return Ok(Some(d)),
                None => self.skipped += 1,
            }
        }
    }

    fn decode(&self, capture_ns: u64) -> Option<Datagram> {
        let f = &self.frame[..];
        let ip = match self.linktype {
            LINKTYPE_ETHERNET => {
                let mut off = 12;
                let mut ethertype = u16::from_be_bytes([*f.get(off)?, *f.get(off + 1)?]);
                // 802.1Q / 802.1ad tags.
                while ethertype == 0x8100 || ethertype == 0x88A8 {
                    off += 4;
                    ethertype = u16::from_be_bytes([*f.get(off)?, *f.get(off + 1)?]);
                }
                (ethertype == 0x0800).then_some(f.get(off + 2..)?)?
            }
            LINKTYPE_NULL => {
                // Address family in host byte order of the capturing machine; AF_INET is 2.
                let family = f.get(0..4)?;
                (family == [2, 0, 0, 0] || family == [0, 0, 0, 2]).then_some(&f[4..])?
            }
            LINKTYPE_RAW | LINKTYPE_IPV4 => f,
            LINKTYPE_LINUX_SLL => (u16::from_be_bytes([*f.get(14)?, *f.get(15)?]) == 0x0800).then_some(f.get(16..)?)?,
            LINKTYPE_LINUX_SLL2 => (u16::from_be_bytes([*f.first()?, *f.get(1)?]) == 0x0800).then_some(f.get(20..)?)?,
            _ => return None,
        };

        if ip.first()? >> 4 != 4 {
            return None;
        }
        let ihl = ((ip[0] & 0x0F) as usize) * 4;
        let total_len = u16::from_be_bytes([*ip.get(2)?, *ip.get(3)?]) as usize;
        let frag = u16::from_be_bytes([*ip.get(6)?, *ip.get(7)?]);
        // More-fragments set, or a non-zero offset: not a whole datagram.
        if frag & 0x3FFF != 0 || *ip.get(9)? != 17 || ihl < IP_LEN {
            return None;
        }
        let ip = ip.get(..total_len)?;
        let src_ip = Ipv4Addr::new(ip[12], ip[13], ip[14], ip[15]);
        let dst_ip = Ipv4Addr::new(ip[16], ip[17], ip[18], ip[19]);
        let udp = ip.get(ihl..)?;
        let udp_len = u16::from_be_bytes([*udp.get(4)?, *udp.get(5)?]) as usize;
        let payload = udp.get(UDP_LEN..udp_len)?;
        Some(Datagram {
            capture_ns,
            src: SocketAddrV4::new(src_ip, u16::from_be_bytes([udp[0], udp[1]])),
            dst: SocketAddrV4::new(dst_ip, u16::from_be_bytes([udp[2], udp[3]])),
            payload: payload.to_vec(),
        })
    }
}

impl<R: Read> Iterator for Reader<R> {
    type Item = io::Result<Datagram>;

    fn next(&mut self) -> Option<Self::Item> {
        self.next_datagram().transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let a = SocketAddrV4::new(Ipv4Addr::new(192, 168, 1, 135), 56300);
        let b = SocketAddrV4::new(Ipv4Addr::new(192, 168, 1, 50), 56301);
        let mut w = Writer::new(Vec::new()).unwrap();
        w.write(1_700_000_000_123_456_789, a, b, b"hello").unwrap();
        w.write(1_700_000_000_223_456_789, b, a, &[7u8; 1380]).unwrap();
        let bytes = w.into_inner();

        let got: Vec<Datagram> = Reader::new(&bytes[..]).unwrap().map(Result::unwrap).collect();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].capture_ns, 1_700_000_000_123_456_789);
        assert_eq!((got[0].src, got[0].dst), (a, b));
        assert_eq!(got[0].payload, b"hello");
        assert_eq!(got[1].payload, vec![7u8; 1380]);
        // Header checksum verifies to zero.
        assert_eq!(ipv4_checksum(&bytes[24 + 16 + ETH_LEN..24 + 16 + ETH_LEN + IP_LEN]), 0);
    }

    #[test]
    fn reads_big_endian_microsecond_loopback() {
        // As tcpdump -i lo0 writes on a big-endian host: BSD loopback link type.
        let mut f = Vec::new();
        f.extend_from_slice(&MAGIC_US.to_be_bytes());
        f.extend_from_slice(&[0, 2, 0, 4]);
        f.extend_from_slice(&[0; 8]);
        f.extend_from_slice(&65535u32.to_be_bytes());
        f.extend_from_slice(&LINKTYPE_NULL.to_be_bytes());
        let payload = [1u8, 2, 3];
        let mut frame = vec![0, 0, 0, 2];
        let mut ip = [0u8; IP_LEN];
        ip[0] = 0x45;
        ip[2..4].copy_from_slice(&((IP_LEN + UDP_LEN + 3) as u16).to_be_bytes());
        ip[9] = 17;
        ip[12..16].copy_from_slice(&[127, 0, 0, 1]);
        ip[16..20].copy_from_slice(&[127, 0, 0, 1]);
        frame.extend_from_slice(&ip);
        frame.extend_from_slice(&[0xDB, 0xEC, 0xDB, 0xED, 0, 11, 0, 0]);
        frame.extend_from_slice(&payload);
        f.extend_from_slice(&5u32.to_be_bytes());
        f.extend_from_slice(&250u32.to_be_bytes());
        f.extend_from_slice(&(frame.len() as u32).to_be_bytes());
        f.extend_from_slice(&(frame.len() as u32).to_be_bytes());
        f.extend_from_slice(&frame);

        let d = Reader::new(&f[..]).unwrap().next_datagram().unwrap().unwrap();
        assert_eq!(d.capture_ns, 5_000_250_000);
        assert_eq!(d.src.port(), 56300);
        assert_eq!(d.dst.port(), 56301);
        assert_eq!(d.payload, payload);
    }
}
