//! Records a lidar's UDP traffic to a pcap file, without a viewer.
//!
//!   cargo run --release --example record -- OUT.pcap [--duration SECS] [SOURCE OPTIONS]
//!
//! Stops on Ctrl-C, after `--duration`, or at the end of a `--replay`
//! (re-recording a replay turns a raw tcpdump capture into a clean one).
//! Play it back with any tool's `--replay OUT.pcap`.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering::Relaxed},
        Arc,
    },
    time::{Duration, Instant},
};

use anyhow::Context;
use livox_rust::{source::Source, Device, Packet};

fn main() -> anyhow::Result<()> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let out = match args.first() {
        Some(a) if !a.starts_with('-') && a.parse::<std::net::Ipv4Addr>().is_err() => args.remove(0),
        _ => {
            eprintln!("record OUT.pcap [--duration SECS] [SOURCE OPTIONS]\n{}", livox_rust::source::USAGE);
            std::process::exit(2);
        }
    };
    let duration = match args.iter().position(|a| a == "--duration") {
        Some(i) => Some(Duration::from_secs_f64(
            args.get(i + 1).context("--duration needs a value")?.parse()?,
        )),
        None => None,
    };
    args.extend(["--record".to_string(), out.clone()]);
    let (cfg, source) = Source::from_args(&args)?;

    let quit = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let quit = quit.clone();
        move || quit.store(true, Relaxed)
    })?;

    let (mut device, rx) = Device::open(cfg)?;
    device.start()?;
    println!("recording to {out} (Ctrl-C to stop)");

    let start = Instant::now();
    let mut last_report = Instant::now();
    let (mut points, mut imu) = (0u64, 0u64);
    while !quit.load(Relaxed) && !duration.is_some_and(|d| start.elapsed() >= d) {
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(Packet::Points(p)) => points += p.points.len() as u64,
            Ok(Packet::Imu { .. }) => imu += 1,
            // A replay is done once it has sent everything and the socket is quiet.
            Err(_) if source.finished() => break,
            Err(_) => {}
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            last_report = Instant::now();
            let s = device.stats();
            println!(
                "{:6.1} s  {points} points  {imu} imu  | lidar gaps {} crc {} | imu gaps {}",
                start.elapsed().as_secs_f64(),
                s.points.seq_gaps.load(Relaxed),
                s.points.crc_errors.load(Relaxed),
                s.imu.seq_gaps.load(Relaxed),
            );
        }
    }

    drop(device); // stops the lidar and flushes the file
    drop(source);
    println!("wrote {out}: {points} points, {imu} imu samples in {:.1} s", start.elapsed().as_secs_f64());
    Ok(())
}
