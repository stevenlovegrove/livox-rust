//! Live point cloud viewer.
//!
//!   cargo run --release -p livox-viz -- [OPTIONS]
//!
//! Options are those of `livox_rust::source::USAGE`, e.g. `192.168.1.135`,
//! `--sim`, `--replay run.pcap --speed 2`, `--record run.pcap`.
//!
//! Left drag orbits, right drag pans, scroll zooms. Ctrl-C or closing the
//! window puts the lidar back to idle (and finishes any recording).

use std::{
    collections::VecDeque,
    sync::{
        atomic::{AtomicBool, Ordering::Relaxed},
        mpsc::{Receiver, RecvTimeoutError},
        Arc,
    },
    time::{Duration, Instant},
};

use anyhow::anyhow;
use livox_rust::{source::Source, Device, Packet, Stats};
use pango::{GraphPane, PanelApp, Var, WidgetPane, WidgetView};
use pango_core::colormap::{Colormap, Jet};
use pango_render::{
    BufferPolicy, CoordAxisList, PointCloud, PointVertex, RenderGraph, RenderToken, SceneView, Transform,
};

/// How often the displayed cloud is rebuilt and uploaded.
const UPLOAD_PERIOD: Duration = Duration::from_millis(50);

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--help" || a == "-h") {
        println!("livox-viz [OPTIONS]\n{}", livox_rust::source::USAGE);
        return Ok(());
    }
    let (cfg, source) = Source::from_args(&args)?;

    let (mut device, rx) = Device::open(cfg)?;
    device.start()?;

    let graph = Arc::new(RenderGraph::new());
    let view = SceneView::new()
        .with_name("scene")
        .with_look_at_up([-3.0, -2.0, 1.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0])
        // The default 0.1..100 m clips a Mid-360's ~70 m returns once zoomed
        // out; 0.05..1000 m still leaves the f32 depth buffer ample precision.
        .with_perspective(std::f32::consts::FRAC_PI_4, 0.05, 1000.0)
        .build(&graph);
    let cloud = graph.add_to(
        "lidar",
        PointCloud::new(1 << 20).with_policy(BufferPolicy::Grow),
        view.world,
        Transform::Identity,
    );
    let mut axis = CoordAxisList::new(1).with_axis_length(0.5);
    axis.push_matrix(&nalgebra::Matrix4::identity());
    graph.add_to("sensor", axis, view.world, Transform::Identity);

    let settings = Settings {
        window_s: Var::new(1.0f32).with_label("Window (s)").with_range(0.05, 5.0),
        by_height: Var::new(true).with_label("Colour by height"),
    };
    let controls = WidgetView::new()
        .with_name("controls")
        .add(&settings.window_s)
        .add(&settings.by_height)
        .build(&graph);

    // The device stays on the main thread: it must outlive the window so that
    // dropping it afterwards stops the lidar.
    let stats = device.stats();
    std::thread::spawn({
        let graph = graph.clone();
        move || accumulate(rx, graph, cloud, settings, stats)
    });

    let quit = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let quit = quit.clone();
        let graph = graph.clone();
        move || {
            quit.store(true, Relaxed);
            graph.wake();
        }
    })?;

    PanelApp::builder(graph)
        .title("Livox")
        .app_name("livox-viz")
        .pane("3D View", GraphPane::from(view))
        .pane("Controls", WidgetPane::from(controls))
        .on_frame(move |ctx| {
            if quit.load(Relaxed) {
                ctx.exit();
            }
        })
        .run()
        .map_err(|e| anyhow!("{e}"))?;

    // Dropping the device puts the lidar back to idle and closes the recording.
    drop(device);
    drop(source);
    Ok(())
}

struct Settings {
    window_s: Var<f32>,
    by_height: Var<bool>,
}

struct Sample {
    t_ns: u64,
    xyz: [f32; 3],
    reflectivity: u8,
}

fn accumulate(
    rx: Receiver<Packet>,
    graph: Arc<RenderGraph>,
    cloud: RenderToken,
    settings: Settings,
    stats: Arc<Stats>,
) {
    let mut samples: VecDeque<Sample> = VecDeque::new();
    let mut last_upload = Instant::now();
    let mut last_report = Instant::now();
    let mut imu_packets = 0u64;
    let mut last_packet_ns = 0u64;

    loop {
        match rx.recv_timeout(UPLOAD_PERIOD) {
            Ok(Packet::Points(p)) => {
                // Time went backwards (a looped replay, or the lidar restarted).
                if p.header.timestamp_ns < last_packet_ns {
                    samples.clear();
                }
                last_packet_ns = p.header.timestamp_ns;
                samples.extend(p.points.iter().filter(|pt| pt.is_valid()).map(|pt| Sample {
                    t_ns: p.header.timestamp_ns + pt.offset_ns as u64,
                    xyz: pt.xyz,
                    reflectivity: pt.reflectivity,
                }))
            }
            Ok(Packet::Imu { .. }) => imu_packets += 1,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
        if last_upload.elapsed() < UPLOAD_PERIOD {
            continue;
        }
        last_upload = Instant::now();

        let window_ns = (*settings.window_s.get() as f64 * 1e9) as u64;
        if let Some(newest) = samples.back().map(|s| s.t_ns) {
            while samples.front().is_some_and(|s| s.t_ns + window_ns < newest) {
                samples.pop_front();
            }
        }

        let by_height = *settings.by_height.get();
        let vertices: Vec<PointVertex> = samples
            .iter()
            .map(|s| {
            let x = if by_height {
                (s.xyz[2] + 1.0) / 4.0
            } else {
                // 0..150 is diffuse reflectance in %, 151..255 retro-reflectors.
                s.reflectivity as f32 / 150.0
            };
            PointVertex::new(s.xyz, Jet.sample(x.clamp(0.0, 1.0)).to_array())
            })
            .collect();
        graph.with_renderable_mut(cloud, |r| {
            if let Some(pc) = r.as_point_cloud_mut() {
                pc.set_points(vertices);
            }
        });

        if last_report.elapsed() >= Duration::from_secs(1) {
            let secs = last_report.elapsed().as_secs_f64();
            last_report = Instant::now();
            let s = &stats.points;
            println!(
                "{:>7} pts shown | imu {:>4.0} Hz | lidar packets {} gaps {} crc {} parse {} dropped {} | imu gaps {}",
                samples.len(),
                imu_packets as f64 / secs,
                s.packets.load(Relaxed),
                s.seq_gaps.load(Relaxed),
                s.crc_errors.load(Relaxed),
                s.parse_errors.load(Relaxed),
                s.dropped.load(Relaxed),
                stats.imu.seq_gaps.load(Relaxed),
            );
            imu_packets = 0;
        }
    }
}
