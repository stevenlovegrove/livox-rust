//! Reads keys from a live lidar and prints each reply raw: whether the
//! lidar knows a key, and what it holds. Read-only (opening a `Device`
//! sends nothing, and queries change nothing), and on its own host ports,
//! so it can run beside another client of the same lidar.
//!
//!   cargo run --release --example query_keys -- [LIDAR_IP] [0xKEY ...]
//!
//! With no keys: the firmware's versions and type, and the IMU's keys.

use livox_rust::{key::Key, Config, Device};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let lidar_ip = args.iter().find_map(|a| a.parse().ok()).unwrap_or(std::net::Ipv4Addr::new(192, 168, 1, 135));
    let keys: Vec<u16> = args.iter().filter_map(|a| u16::from_str_radix(a.strip_prefix("0x")?, 16).ok()).collect();
    let named = [
        Key::VersionApp,
        Key::VersionLoader,
        Key::VersionHardware,
        Key::FwType,
        Key::ProductInfo,
        Key::ImuDataEnable,
        Key::ImuSensorCfg,
        Key::CurWorkState,
    ];
    let mut cfg = Config::new(lidar_ip);
    // Not the defaults another client of the lidar holds.
    (cfg.host_cmd_port, cfg.host_point_port, cfg.host_imu_port) = (56191, 56391, 56491);
    let (mut device, _packets) = Device::open(cfg)?;
    let list: Vec<(String, Key)> = if keys.is_empty() {
        named.iter().map(|k| (format!("{k:?}"), *k)).collect()
    } else {
        keys.iter().map(|&k| (format!("0x{k:04X}"), key_of(k))).collect()
    };
    for (name, key) in list {
        match device.query(&[key]) {
            Ok(items) if items.is_empty() => println!("{name} (0x{:04X}): accepted, no value", key as u16),
            Ok(items) => {
                for (k, v) in items {
                    let text = String::from_utf8_lossy(&v).trim_end_matches('\0').to_string();
                    let printable = !v.is_empty() && text.chars().all(|c| !c.is_control());
                    println!("{name} (0x{k:04X}): {} bytes {:02X?}{}", v.len(), v, if printable { format!(" \"{text}\"") } else { String::new() });
                }
            }
            Err(e) => println!("{name} (0x{:04X}): {e:#}", key as u16),
        }
    }
    Ok(())
}

fn key_of(k: u16) -> Key {
    use Key::*;
    [
        PclDataType, PatternMode, LidarIpCfg, StateInfoHostIpCfg, PointCloudHostIpCfg, ImuHostIpCfg, InstallAttitude, FovCfg0, FovCfg1, FovCfgEn, DetectModeCfg,
        FuncIoCfg, WorkTgtMode, ImuDataEnable, ImuSensorCfg, Sn, ProductInfo, VersionApp, VersionLoader, VersionHardware, Mac, CurWorkState, CoreTemp,
        PowerUpCount, LocalTimeNow, LastSyncTime, TimeOffset, TimeSyncType, LidarDiagStatus, FwType, HmsCode,
    ]
    .into_iter()
    .find(|x| *x as u16 == k)
    .unwrap_or(Unknown)
}
