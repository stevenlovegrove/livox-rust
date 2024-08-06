pub mod control_packet;
pub mod data_packet;
pub mod datatype;
pub mod key;
mod key_size;

use binrw::{io::Cursor, BinRead, BinWrite};
use control_packet::*;
use crc::*;
use data_packet::LidarPacket;
use datatype::WorkState;
use get_if_addrs::get_if_addrs;
use key::*;
use pnet::datalink::{self, NetworkInterface};
use std::net::SocketAddr;
use tokio::net::UdpSocket;

const CRC_HEADER: Crc<u16> = crc::Crc::<u16>::new(&crc::Algorithm {
    width: 16,
    poly: 0x1021,
    init: 0xFFFF,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0x29B1,   // optional: a known good value for a specific input
    residue: 0x0000, // optional: the residue of a valid input
});

const CRC_DATA: Crc<u32> = crc::Crc::<u32>::new(&crc::Algorithm {
    width: 32,
    poly: 0x04C11DB7,
    init: 0xFFFFFFFF,
    refin: true,
    refout: true,
    xorout: 0xFFFFFFFF,
    check: 0xCBF43926,
    residue: 0xDEBB20E3,
});

async fn handle_push_cmd(
    socket_push_cmd: tokio::net::UdpSocket,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut buf = vec![0; 1024];

    loop {
        let (len, addr) = socket_push_cmd.recv_from(&mut buf).await?;
        println!("handle_push_cmd: Received {} bytes from {}", len, addr);

        let mut cursor = Cursor::new(&buf[..len]);
        match ControlCommandPacket::read(&mut cursor) {
            Ok(frame) => match frame.header.cmd_id {
                CommandID::LidarPushInfo => {
                    // println!("{:?}", frame);
                    // let mut cursor = Cursor::new(&frame.data);
                    // let t = KeyValueList::read(&mut cursor).map(|kv_list| {
                    //     println!("{:?}", kv_list);
                    // });
                    // println!("{:?}", t);

                    let data_crc = CRC_DATA.checksum(&frame.data);
                    let header_crc = CRC_HEADER.checksum(&buf[..18]);
                    println!("Data CRC: 0x{:08X} vs 0x{:08X}", data_crc, frame.crc32);
                    println!("Header CRC: 0x{:04X} vs 0x{:04X}", header_crc, frame.crc16);
                }
                _ => {
                    println!("Received {:?}", frame);
                }
            },
            Err(e) => eprintln!("Failed to parse frame: {:?}", e),
        }
    }
}

async fn handle_pointcloud_data(
    socket_pointcloud_data: tokio::net::UdpSocket,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut buf = vec![0; 10240];

    loop {
        let (len, addr) = socket_pointcloud_data.recv_from(&mut buf).await?;
        println!(
            "handle_pointcloud_data: Received {} bytes from {}",
            len, addr
        );

        let mut cursor = Cursor::new(&buf[..len]);
        match LidarPacket::read(&mut cursor) {
            Ok(frame) => {
                println!("{:?}", frame);
            }
            Err(e) => eprintln!("Failed to parse frame: {:?}", e),
        }
    }
}

async fn handle_any_data(socket: tokio::net::UdpSocket) -> Result<(), Box<dyn std::error::Error>> {
    let mut buf = vec![0; 1024];

    loop {
        let (len, addr) = socket.recv_from(&mut buf).await?;
        println!("Received {} bytes from {}", len, addr);
        let mut cursor = Cursor::new(&buf[..len]);
        let packet = ControlCommandPacket::read(&mut cursor).unwrap();
        println!("{:?}", packet);
        match packet.header.cmd_id {
            CommandID::LidarParamInfoConfig => {
                let mut cursor = Cursor::new(packet.data);
                let ack = ControlCommandParamConfigAck::read(&mut cursor).unwrap();
                println!("{:?}", ack);
            }
            _ => {}
        }
    }
}

async fn make_config_packet() -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let data = {
        let mut cursor = Cursor::new(Vec::new());
        let list = KeyValueList {
            items: vec![KeyValueItem {
                key: Key::WorkTgtMode,
                value: KeyValue::WorkTgtMode(WorkState::IDLE),
                // value: KeyValue::WorkTgtMode(WorkState::Sampling),
            }],
        };
        list.write(&mut cursor)?;
        cursor.into_inner()
    };

    let mut cursor = Cursor::new(Vec::new());
    let header = ControlCommandPacketHeader {
        length: 24 + data.len() as u16,
        seq_num: 0,
        cmd_id: CommandID::LidarParamInfoConfig,
        cmd_type: CommandType::Req,
        sender_type: SenderType::HostComputer,
    };
    header.write(&mut cursor)?;
    let header_crc = CRC_HEADER.checksum(cursor.get_ref()[..18].as_ref());
    let data_crc = CRC_DATA.checksum(&data);

    cursor.set_position(0);

    let packet = ControlCommandPacket {
        header,
        crc16: header_crc,
        crc32: data_crc,
        data: data,
    };
    packet.write(&mut cursor)?;
    Ok(cursor.into_inner())
}

#[tokio::main]
async fn main2() {
    use three_d::*;

    let window = Window::new(WindowSettings {
        title: "LIVOX".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();
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
    let mut control = OrbitControl::new(*camera.target(), 0.1, 3.0);

    // Load point cloud
    let mut loaded =
        three_d_asset::io::load_async(&["/Users/stevenlovegrove/code/livox-rust/hand.pcd"])
            .await
            .unwrap();
    let cpu_point_cloud: PointCloud = loaded.deserialize("hand.pcd").unwrap();

    let mut point_mesh = CpuMesh::circle(10);
    point_mesh.transform(&Mat4::from_scale(0.001)).unwrap();

    let mut point_cloud = Gm {
        geometry: InstancedMesh::new(&context, &cpu_point_cloud.into(), &point_mesh),
        material: ColorMaterial::default(),
    };
    let c = -point_cloud.aabb().center();
    point_cloud.set_transformation(Mat4::from_translation(c));

    // main loop
    window.render_loop(move |mut frame_input| {
        let mut redraw = frame_input.first_frame;
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
                        .chain(&Axes::new(&context, 0.01, 0.1)),
                    &[],
                );
        }

        FrameOutput {
            swap_buffers: redraw,
            ..Default::default()
        }
    });
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let interfaces = get_if_addrs()?;

    println!("Available Non loopback IPv4 Interfaces:");
    for iface in interfaces
        .iter()
        .filter(|iface| !iface.is_loopback() && iface.ip().is_ipv4())
    {
        println!("Interface {} ({:?})", iface.name, iface.addr.ip());
    }

    let iface_name = "en11";
    println!("Using interface '{}'", iface_name);

    let interfaces = datalink::interfaces();
    let selected_iface = interfaces
        .into_iter()
        .find(|iface: &NetworkInterface| iface.name == iface_name)
        .expect("No matching interface found");
    let selected_ip = selected_iface
        .ips
        .into_iter()
        .find(|ip| ip.is_ipv4())
        .expect("No matching IP found");

    let addr_lidar_control_cmd: SocketAddr = "192.168.1.135:56100".parse()?;

    let addr_host_control_cmd: SocketAddr = "0.0.0.0:56101".parse()?;
    let addr_push_cmd: SocketAddr = "0.0.0.0:56201".parse()?;
    let addr_pointcloud_data: SocketAddr = "0.0.0.0:56301".parse()?;
    // let addr_imu_data: SocketAddr = "0.0.0.0:56401".parse()?;
    // let addr_log_data: SocketAddr = "0.0.0.0:56501".parse()?;

    let socket_push_cmd = UdpSocket::bind(addr_push_cmd).await?;
    let socket_pointcloud_data = UdpSocket::bind(addr_pointcloud_data).await?;
    let socket_host_control_cmd = UdpSocket::bind(addr_host_control_cmd).await?;

    // loop for push cmd
    tokio::spawn(async move {
        handle_push_cmd(socket_push_cmd).await.unwrap();
    });

    // loop for pointcloud data
    tokio::spawn(async move {
        handle_pointcloud_data(socket_pointcloud_data)
            .await
            .unwrap();
    });

    // loop for any data (we expect some acks here)
    tokio::spawn(async move {
        let start_sampling = make_config_packet().await.unwrap();
        let mut cursor = Cursor::new(&start_sampling);
        let packet = ControlCommandPacket::read(&mut cursor).unwrap();
        println!("{:?}", packet);
        let mut cursor = Cursor::new(packet.data);
        let kv_list = KeyValueList::read(&mut cursor).unwrap();
        println!("{:?}", kv_list);

        socket_host_control_cmd
            .send_to(&start_sampling, addr_lidar_control_cmd)
            .await
            .unwrap();
        handle_any_data(socket_host_control_cmd).await.unwrap();
    });

    loop {
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
