//! Receiving a datagram with the time it arrived.
//!
//! On Linux the kernel stamps each datagram as the network stack receives
//! it (`SO_TIMESTAMPNS`, CLOCK_REALTIME), before the receive thread is even
//! scheduled. Elsewhere the time is taken just after `recv` returns.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{SystemTime, UNIX_EPOCH};

/// Where a receive time came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxClock {
    /// The kernel's receive timestamp (Linux).
    Kernel,
    /// Read in user space after `recv` returned.
    User,
}

pub(crate) fn now_ns() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_nanos() as u64
}

/// Asks the kernel to timestamp received datagrams; returns which clock
/// [`recv`] will report.
pub(crate) fn enable(socket: &UdpSocket) -> RxClock {
    #[cfg(target_os = "linux")]
    {
        use std::os::fd::AsRawFd;
        let on: libc::c_int = 1;
        let rc = unsafe {
            libc::setsockopt(
                socket.as_raw_fd(),
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMPNS,
                &on as *const _ as *const libc::c_void,
                std::mem::size_of_val(&on) as libc::socklen_t,
            )
        };
        if rc == 0 {
            return RxClock::Kernel;
        }
    }
    let _ = socket;
    RxClock::User
}

/// Receives one datagram: its length, sender, and arrival time (ns since
/// the Unix epoch), from the kernel when it provided one.
pub(crate) fn recv(socket: &UdpSocket, buf: &mut [u8]) -> io::Result<(usize, SocketAddr, u64)> {
    #[cfg(target_os = "linux")]
    {
        recv_linux(socket, buf)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let (len, from) = socket.recv_from(buf)?;
        Ok((len, from, now_ns()))
    }
}

#[cfg(target_os = "linux")]
fn recv_linux(socket: &UdpSocket, buf: &mut [u8]) -> io::Result<(usize, SocketAddr, u64)> {
    use std::net::{Ipv4Addr, SocketAddrV4};
    use std::os::fd::AsRawFd;

    let mut addr: libc::sockaddr_in = unsafe { std::mem::zeroed() };
    let mut iov = libc::iovec { iov_base: buf.as_mut_ptr() as *mut libc::c_void, iov_len: buf.len() };
    // Room for one timespec control message, with alignment to spare.
    let mut control = [0u64; 8];
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_name = &mut addr as *mut _ as *mut libc::c_void;
    msg.msg_namelen = std::mem::size_of_val(&addr) as libc::socklen_t;
    msg.msg_iov = &mut iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.as_mut_ptr() as *mut libc::c_void;
    msg.msg_controllen = std::mem::size_of_val(&control) as _;

    let n = unsafe { libc::recvmsg(socket.as_raw_fd(), &mut msg, 0) };
    if n < 0 {
        // EAGAIN after the read timeout surfaces as WouldBlock, as with recv_from.
        return Err(io::Error::last_os_error());
    }
    let mut rx_ns = None;
    unsafe {
        let mut cmsg = libc::CMSG_FIRSTHDR(&msg);
        while !cmsg.is_null() {
            if (*cmsg).cmsg_level == libc::SOL_SOCKET && (*cmsg).cmsg_type == libc::SCM_TIMESTAMPNS {
                let ts = std::ptr::read_unaligned(libc::CMSG_DATA(cmsg) as *const libc::timespec);
                rx_ns = Some(ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64);
            }
            cmsg = libc::CMSG_NXTHDR(&msg, cmsg);
        }
    }
    let from = SocketAddrV4::new(Ipv4Addr::from(u32::from_be(addr.sin_addr.s_addr)), u16::from_be(addr.sin_port));
    Ok((n as usize, SocketAddr::V4(from), rx_ns.unwrap_or_else(now_ns)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn receive_time_is_close_to_now() {
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let clock = enable(&rx);
        if cfg!(target_os = "linux") {
            assert_eq!(clock, RxClock::Kernel);
        }
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let before = now_ns();
        tx.send_to(b"hello", rx.local_addr().unwrap()).unwrap();
        let mut buf = [0u8; 16];
        let (n, from, t) = recv(&rx, &mut buf).unwrap();
        let after = now_ns();
        assert_eq!(&buf[..n], b"hello");
        assert_eq!(from, tx.local_addr().unwrap());
        assert!(before <= t && t <= after, "{before} <= {t} <= {after}");
    }
}
