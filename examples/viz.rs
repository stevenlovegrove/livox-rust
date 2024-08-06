async fn run() -> anyhow::Result<()> {
    let lidar_ip = std::net::Ipv4Addr::new(192, 168, 1, 135);
    let host_interface_ip = std::net::Ipv4Addr::UNSPECIFIED;
    let device = livox_rust::lidar::Device::open(lidar_ip, host_interface_ip).await?;

    let ctrl_c_cancel_token = tokio_util::sync::CancellationToken::new();
    ctrlc::set_handler({
        let ctrl_c_cancel_token = ctrl_c_cancel_token.clone();
        move || {
            ctrl_c_cancel_token.cancel();
        }
    })
    .expect("Error setting Ctrl-C handler");

    device.start().await?;

    let mut recv = device.lidar_receiver();

    loop {
        tokio::select! {
          _ = ctrl_c_cancel_token.cancelled() => {
            println!("Ctrl-C received, stopping device...");
            device.stop().await?;
            break;
          },
          r = recv.recv() => {
            match r {
              Ok(packet) => {
                  // println!("{:?}", packet);
              }
              Err(e) => {
                  eprintln!("Error: {:?}", e);
                  break;
              }
            }
          }
        }
    }

    Ok(())
}

#[tokio::main]
async fn main() {
    match run().await {
        Ok(_) => {}
        Err(e) => eprintln!("Error: {:?}", e),
    }
}
