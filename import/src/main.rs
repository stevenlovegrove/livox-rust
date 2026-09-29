//! Converts a ROS 2 bag (sqlite3 `.db3`) of a Livox lidar to a pcap recording
//! of the lidar's UDP stream, replayable with any tool's `--replay`.
//!
//!   cargo run --release -p livox-import -- BAG_DIR_OR_DB3... -o OUT.pcap
//!       [--lidar-topic T] [--imu-topic T] [--imu-accel auto|g|mps2]
//!
//! By default the topics are found by type: livox_ros_driver(2) `CustomMsg`
//! for points (the only Livox message with per-point times) and
//! `sensor_msgs/msg/Imu`. IMU acceleration is written in g, as the lidar
//! sends it; `auto` detects m/s^2 from the first sample.

mod convert;
mod ros;

use std::path::{Path, PathBuf};

use anyhow::{bail, ensure, Context};
use convert::{AccelUnits, Converter};
use livox_rust::pcap;
use rusqlite::{Connection, OpenFlags};

struct Args {
    inputs: Vec<PathBuf>,
    out: PathBuf,
    lidar_topic: Option<String>,
    imu_topic: Option<String>,
    accel: AccelUnits,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut inputs = Vec::new();
    let (mut out, mut lidar_topic, mut imu_topic, mut accel) = (None, None, None, AccelUnits::Auto);
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut value = || it.next().with_context(|| format!("{a} needs a value"));
        match a.as_str() {
            "-o" | "--out" => out = Some(PathBuf::from(value()?)),
            "--lidar-topic" => lidar_topic = Some(value()?),
            "--imu-topic" => imu_topic = Some(value()?),
            "--imu-accel" => {
                accel = match value()?.as_str() {
                    "auto" => AccelUnits::Auto,
                    "g" => AccelUnits::G,
                    "mps2" => AccelUnits::MetresPerSecond2,
                    other => bail!("--imu-accel {other}: expected auto, g or mps2"),
                }
            }
            "-h" | "--help" => {
                println!("{}", include_str!("main.rs").lines().take_while(|l| l.starts_with("//!")).map(|l| l.trim_start_matches("//!").trim_start_matches(' ')).collect::<Vec<_>>().join("\n"));
                std::process::exit(0);
            }
            _ if a.starts_with('-') => bail!("unknown option {a}"),
            _ => inputs.push(PathBuf::from(a)),
        }
    }
    ensure!(!inputs.is_empty(), "no input bag given (see --help)");
    let out = out.context("no output given (-o OUT.pcap)")?;
    Ok(Args { inputs, out, lidar_topic, imu_topic, accel })
}

/// The `.db3` files of the inputs, in order (a bag directory may be split).
fn db3_files(inputs: &[PathBuf]) -> anyhow::Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for input in inputs {
        if input.is_dir() {
            let meta = input.join("metadata.yaml");
            if let Ok(yaml) = std::fs::read_to_string(&meta) {
                let compressed = yaml
                    .lines()
                    .any(|l| l.trim().starts_with("compression_format:") && !l.contains("\"\"") && !l.trim().ends_with(':'));
                ensure!(!compressed, "{} is a compressed bag; decompress it first", input.display());
            }
            let mut found: Vec<PathBuf> = std::fs::read_dir(input)?
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().is_some_and(|e| e == "db3"))
                .collect();
            ensure!(!found.is_empty(), "no .db3 files in {}", input.display());
            // rosbag2 names splits NAME_0.db3, NAME_1.db3, ...: sort by that index.
            found.sort_by_key(|p| split_index(p));
            files.extend(found);
        } else if input.extension().is_some_and(|e| e == "mcap" || e == "bag") {
            bail!("{}: only ROS 2 sqlite3 (.db3) bags are supported so far", input.display());
        } else {
            files.push(input.clone());
        }
    }
    Ok(files)
}

fn split_index(p: &Path) -> (u64, PathBuf) {
    let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("");
    let index = stem.rsplit('_').next().and_then(|s| s.parse().ok()).unwrap_or(0);
    (index, p.to_path_buf())
}

struct Topic {
    id: i64,
    name: String,
    ty: String,
    format: String,
}

fn topics(db: &Connection) -> anyhow::Result<Vec<Topic>> {
    let mut stmt = db.prepare("SELECT id, name, type, serialization_format FROM topics")?;
    let rows = stmt.query_map([], |r| {
        Ok(Topic { id: r.get(0)?, name: r.get(1)?, ty: r.get(2)?, format: r.get(3)? })
    })?;
    Ok(rows.collect::<Result<_, _>>()?)
}

fn pick<'a>(topics: &'a [Topic], wanted: &Option<String>, is_type: impl Fn(&str) -> bool, what: &str) -> anyhow::Result<&'a Topic> {
    let candidates: Vec<&Topic> = match wanted {
        Some(name) => topics.iter().filter(|t| &t.name == name).collect(),
        None => topics.iter().filter(|t| is_type(&t.ty)).collect(),
    };
    match candidates.as_slice() {
        [t] => {
            ensure!(t.format == "cdr", "{} is serialised as {}, not cdr", t.name, t.format);
            ensure!(is_type(&t.ty), "{} has type {}, not a {what}", t.name, t.ty);
            Ok(t)
        }
        [] => bail!(
            "no {what} topic{}; topics are:\n{}",
            wanted.as_ref().map(|w| format!(" named {w}")).unwrap_or_default(),
            topics.iter().map(|t| format!("  {} ({})", t.name, t.ty)).collect::<Vec<_>>().join("\n")
        ),
        many => bail!(
            "several {what} topics ({}); choose one with --{}-topic",
            many.iter().map(|t| t.name.as_str()).collect::<Vec<_>>().join(", "),
            if what == "IMU" { "imu" } else { "lidar" }
        ),
    }
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    let files = db3_files(&args.inputs)?;
    let out = pcap::Writer::create(&args.out).with_context(|| format!("creating {}", args.out.display()))?;
    let mut conv = Converter::new(out, args.accel);
    let (mut lidar_errors, mut imu_errors) = (0u64, 0u64);

    for file in &files {
        println!("reading {}", file.display());
        let db = Connection::open_with_flags(file, OpenFlags::SQLITE_OPEN_READ_ONLY)
            .with_context(|| format!("opening {}", file.display()))?;
        let topics = topics(&db)?;
        let lidar = pick(&topics, &args.lidar_topic, |t| t.starts_with("livox_ros_driver") && t.ends_with("/CustomMsg"), "Livox CustomMsg")?;
        let imu = pick(&topics, &args.imu_topic, |t| t == "sensor_msgs/msg/Imu", "IMU")?;
        println!("  points: {} ({})\n  imu:    {} ({})", lidar.name, lidar.ty, imu.name, imu.ty);

        let mut stmt = db.prepare(
            "SELECT topic_id, data FROM messages WHERE topic_id IN (?1, ?2) ORDER BY timestamp",
        )?;
        let mut rows = stmt.query([lidar.id, imu.id])?;
        while let Some(row) = rows.next()? {
            let topic: i64 = row.get(0)?;
            let data = row.get_ref(1)?.as_blob()?;
            if topic == lidar.id {
                match ros::custom_msg(data) {
                    Ok(m) => conv.push_lidar(&m)?,
                    Err(e) => {
                        lidar_errors += 1;
                        if lidar_errors == 1 {
                            eprintln!("  bad {} message: {e:#}", lidar.name);
                        }
                    }
                }
            } else {
                match ros::imu(data) {
                    Ok(m) => conv.push_imu(&m)?,
                    Err(e) => {
                        imu_errors += 1;
                        if imu_errors == 1 {
                            eprintln!("  bad {} message: {e:#}", imu.name);
                        }
                    }
                }
            }
        }
    }

    let (r, _) = conv.finish()?;
    let secs = (r.last_ns - r.first_ns.unwrap_or(r.last_ns)) as f64 * 1e-9;
    println!("wrote {} ({:.1} s)", args.out.display(), secs);
    println!(
        "  {} frames, {} points in {} packets ({:.1} points/packet)",
        r.frames,
        r.points,
        r.point_packets,
        r.points as f64 / r.point_packets.max(1) as f64
    );
    println!(
        "  point time error: max {} ns, mean {:.1} ns{}",
        r.max_time_error_ns,
        r.sum_time_error_ns as f64 / r.points.max(1) as f64,
        if r.unsorted_frames > 0 { format!(" ({} frames had packets out of order)", r.unsorted_frames) } else { String::new() }
    );
    println!(
        "  {} imu samples ({:.0} Hz), acceleration {}",
        r.imu_samples,
        r.imu_samples as f64 / secs.max(1e-9),
        match r.accel_scale {
            Some(1.0) => "already in g".to_string(),
            Some(_) => "converted from m/s^2 to g".to_string(),
            None => "-".to_string(),
        }
    );
    if lidar_errors + imu_errors > 0 {
        println!("  skipped {lidar_errors} lidar and {imu_errors} imu messages that failed to decode");
    }
    Ok(())
}
