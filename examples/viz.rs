use std::sync::{atomic::AtomicBool, Arc, Mutex};

use livox_rust::data_packet::Data;
use tokio::sync::broadcast::error::TryRecvError;

#[tokio::main]
async fn main() {
    match run().await {
        Ok(_) => {}
        Err(e) => eprintln!("Error: {:?}", e),
    }
}
async fn run() -> anyhow::Result<()> {
    let lidar_ip = std::net::Ipv4Addr::new(192, 168, 1, 135);
    let host_interface_ip = std::net::Ipv4Addr::UNSPECIFIED;
    let device = Arc::new(Mutex::new(
        livox_rust::lidar::Device::open(lidar_ip, host_interface_ip).await?,
    ));
    device.lock().unwrap().start().await?;

    let mut recv = device.lock().unwrap().lidar_receiver();

    use three_d::*;

    let window = Window::new(WindowSettings {
        title: "LIVOX".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();

    let should_end = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let should_end = should_end.clone();
        move || {
            let device = device.clone();
            async_std::task::block_on(device.lock().unwrap().stop()).unwrap();
            should_end.store(true, std::sync::atomic::Ordering::Relaxed);
        }
    })
    .expect("Error setting Ctrl-C handler");

    let context = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.125, -0.25, -0.5),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.01,
        100.0,
    );
    let mut control = OrbitControl::new(*camera.target(), 2.0, 20.0);

    let points = Arc::new(Mutex::new(Vec::with_capacity(100000)));

    let mut point_mesh = CpuMesh::sphere(4);
    point_mesh.transform(&Mat4::from_scale(0.01)).unwrap();
    let point_cloud: Arc<Mutex<Gm<InstancedMesh, ColorMaterial>>> = Arc::new(Mutex::new(Gm {
        geometry: InstancedMesh::new(&context, &PointCloud::default().into(), &point_mesh),
        material: ColorMaterial {
            color: Srgba::RED,
            ..Default::default()
        },
    }));

    // main loop
    window.render_loop({
        let should_end = should_end.clone();
        move |mut frame_input| {
            let mut points = points.lock().unwrap();
            let mut point_cloud = point_cloud.lock().unwrap();

            loop {
                match recv.try_recv() {
                    Ok(packet) => {
                        if let Data::PointCloud1(laser_points) = packet.data {
                            for point in laser_points {
                                points.push(
                                    Vec3::new(point.x as f32, point.y as f32, point.z as f32)
                                        / 1000.0,
                                );
                            }
                        }
                    }
                    Err(TryRecvError::Lagged(_)) => {
                        // continue to catch up
                    }
                    Err(_) => {
                        break;
                    }
                }
            }

            if points.len() > 100000 {
                let pc = PointCloud {
                    positions: Positions::F32(points.to_vec()),
                    ..Default::default()
                };
                point_cloud.geometry =
                    InstancedMesh::new(&context, &pc.into(), &point_mesh);
                points.clear();
            }

            // let pc = PointCloud {
            //     positions: Positions::F32(points.to_vec()),
            //     ..Default::default()
            // };

            // let point_cloud = Gm {
            //     geometry: InstancedMesh::new(&context, &pc.into(), &point_mesh),
            //     material: ColorMaterial {
            //         color: Srgba::RED,
            //         ..Default::default()
            //     },
            // };

            let mut redraw = true; //frame_input.first_frame;
            redraw |= camera.set_viewport(frame_input.viewport);
            redraw |= control.handle_events(&mut camera, &mut frame_input.events);

            if redraw {
                frame_input
                    .screen()
                    .clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0))
                    .render(
                        &camera,
                        point_cloud
                            .into_iter()
                            .chain(&Axes::new(&context, 0.01, 1.0)),
                        &[],
                    );
            }

            FrameOutput {
                swap_buffers: redraw,
                exit: should_end.load(std::sync::atomic::Ordering::Relaxed),
                ..Default::default()
            }
        }
    });

    Ok(())
}

// fn main() {
//     let runtime = tokio::runtime::Builder::new_multi_thread()
//         .worker_threads(4)
//         .enable_all()
//         .build()
//         .unwrap();

//     let cancel_token = tokio_util::sync::CancellationToken::new();
//     ctrlc::set_handler({
//         let ctrl_c_cancel_token = cancel_token.clone();
//         move || {
//             ctrl_c_cancel_token.cancel();
//         }
//     })
//     .expect("Error setting Ctrl-C handler");

//     let run_jh = runtime.spawn({
//         let cancel_token = cancel_token.clone();
//         async move {
//             match run(cancel_token).await {
//                 Ok(_) => {}
//                 Err(e) => eprintln!("Error: {:?}", e),
//             }
//             println!("4");
//         }
//     });

//     gui();
//     cancel_token.cancel();
//     runtime.block_on(async {run_jh.await.unwrap()});
// }
