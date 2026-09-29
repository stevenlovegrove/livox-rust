//! Picks where a tool's data comes from — a live lidar, the simulator, or a
//! recording — from its command line. Whichever it is, the tool then opens a
//! `Device` with the returned `Config`, so the code path is the same.

use std::{net::Ipv4Addr, path::PathBuf};

use anyhow::{bail, Context};

use crate::{
    lidar::Config,
    replay::{ReplayLidar, ReplayOptions},
    sim::FakeLidar,
};

pub const USAGE: &str = "\
  [LIDAR_IP [HOST_IP]]             live lidar (default 192.168.1.135, all interfaces)
  --sim                            simulated lidar on loopback
  --replay FILE [--speed X] [--loop]
                                   replay a pcap recording (or a tcpdump capture)
  --record FILE                    also record everything received to a pcap file";

/// Keeps the simulated or replayed lidar running; drop it after the `Device`.
pub enum Source {
    Live,
    Sim(FakeLidar),
    Replay(ReplayLidar),
}

impl Source {
    /// Parses the options in `USAGE` from `args`, ignoring anything else (so
    /// tools can take their own flags too).
    pub fn from_args(args: &[String]) -> anyhow::Result<(Config, Source)> {
        let value = |flag: &str| -> anyhow::Result<Option<&String>> {
            match args.iter().position(|a| a == flag) {
                Some(i) => args.get(i + 1).map(Some).with_context(|| format!("{flag} needs a value")),
                None => Ok(None),
            }
        };
        let ips: Vec<Ipv4Addr> = args.iter().filter_map(|a| a.parse().ok()).collect();
        let sim = args.iter().any(|a| a == "--sim");
        let replay = value("--replay")?.map(PathBuf::from);
        if sim && replay.is_some() {
            bail!("--sim and --replay are exclusive");
        }

        let mut cfg = Config::new(ips.first().copied().unwrap_or(Ipv4Addr::new(192, 168, 1, 135)));
        if let Some(&host_ip) = ips.get(1) {
            cfg.host_ip = host_ip;
        }
        cfg.record = value("--record")?.map(PathBuf::from);
        if sim || replay.is_some() {
            cfg.lidar_ip = Ipv4Addr::LOCALHOST;
            cfg.host_ip = Ipv4Addr::LOCALHOST;
        }

        let source = if sim {
            Source::Sim(FakeLidar::spawn(&cfg)?)
        } else if let Some(path) = replay {
            let opts = ReplayOptions {
                speed: match value("--speed")? {
                    Some(s) => s.parse().with_context(|| format!("bad --speed {s}"))?,
                    None => 1.0,
                },
                looped: args.iter().any(|a| a == "--loop"),
                lidar_ip: None,
            };
            Source::Replay(ReplayLidar::spawn(&cfg, path, opts)?)
        } else {
            Source::Live
        };
        Ok((cfg, source))
    }

    /// True when a replay has sent everything (never for other sources).
    pub fn finished(&self) -> bool {
        matches!(self, Source::Replay(r) if r.finished())
    }
}
