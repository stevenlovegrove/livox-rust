use std::{
    io::Cursor,
    net::{Ipv4Addr, SocketAddrV4},
};

use binrw::BinRead;
use tokio::net::UdpSocket;
use tokio_util::sync::CancellationToken;

use crate::{data_packet::LidarPacket, params::{packet_start, packet_stop}};

pub struct Device {
    cancel_token: CancellationToken,
    lidar_sender: tokio::sync::broadcast::Sender<LidarPacket>,
    socket_host_control_cmd: UdpSocket,
}

impl Device {
    // specify Ipv4Addr::UNSPECIFIED if you do not care which interface to use
    pub async fn open(lidar_ip: Ipv4Addr, host_interface_ip: Ipv4Addr) -> anyhow::Result<Self> {
        let cancel_token = CancellationToken::new();
        let (lidar_sender, _) = tokio::sync::broadcast::channel::<LidarPacket>(100);

        let addr_lidar_control_cmd = SocketAddrV4::new(lidar_ip, 56100);
        let addr_host_control_cmd = SocketAddrV4::new(host_interface_ip, 56101);
        let addr_push_cmd = SocketAddrV4::new(host_interface_ip, 56201);
        let addr_pointcloud_data = SocketAddrV4::new(host_interface_ip, 56301);
        // let addr_imu_data = SocketAddrV4::new(host_interface_ip, 56401);
        // let addr_log_data = SocketAddrV4::new(host_interface_ip, 56501);

        let socket_push_cmd = UdpSocket::bind(addr_push_cmd).await?;
        let socket_pointcloud_data = UdpSocket::bind(addr_pointcloud_data).await?;
        let socket_host_control_cmd = UdpSocket::bind(addr_host_control_cmd).await?;
        socket_host_control_cmd
            .connect(addr_lidar_control_cmd)
            .await?;

        tokio::spawn({
            let lidar_sender = lidar_sender.clone();
            let cancel_token = cancel_token.clone();
            async move {
                let mut buf = [0; 10240];

                loop {
                    tokio::select! {
                      _ = cancel_token.cancelled() => {
                        break
                      },
                      Ok((len, _addr)) = socket_pointcloud_data.recv_from(&mut buf) => {
                        let mut cursor = Cursor::new(&buf[..len]);
                        match LidarPacket::read(&mut cursor) {
                          Ok(frame) => {
                            if let Err(e) = lidar_sender.send(frame) {
                              eprintln!("Failed to send frame to lidar_sender: {:?}", e);
                            }
                          }
                          Err(e) => eprintln!("Failed to parse frame: {:?}", e),
                        }
                      }
                    }
                }
            }
        });

        Ok(Self {
            cancel_token,
            lidar_sender,
            socket_host_control_cmd,
        })
    }

    pub fn lidar_receiver(&self) -> tokio::sync::broadcast::Receiver<LidarPacket> {
        self.lidar_sender.subscribe()
    }

    pub async fn start(&self) -> anyhow::Result<()> {
        let packet = packet_start().await?;
        self.socket_host_control_cmd.send(&packet).await?;
        Ok(())
    }

    pub async fn stop(&self) -> anyhow::Result<()> {
        let packet = packet_stop().await?;
        self.socket_host_control_cmd.send(&packet).await?;
        Ok(())
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        self.cancel_token.cancel();
    }
}
