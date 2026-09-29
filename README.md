# livox-rust

A Rust driver for Livox lidars that use the SDK2 protocol (developed against
the Mid-360), with tools to view, record and replay their data, and to import
public datasets. It is the sensor front end for a lidar-inertial SLAM and 3D
reconstruction system.

- **Driver**: control the lidar and receive its point cloud and IMU streams,
  decoded to metres (points) and rad/s and g (IMU), with a time for every
  point.
- **Record / replay**: record the lidar's UDP traffic to a standard pcap file,
  and replay it as a lidar on loopback, so recorded data goes through exactly
  the same code as a live sensor.
- **Import**: convert ROS 2 bags of Livox data back into the lidar's packets.
- **Viewer**: a live point cloud viewer built on
  [pangolino](https://github.com/stevenlovegrove/pangolino).

## Workspace

| Crate | Path | What |
|---|---|---|
| `livox-rust` | `.` | Driver library; `examples/record.rs` |
| `livox-viz` | `viz/` | Point cloud viewer |
| `livox-import` | `import/` | `bag2pcap`, ROS 2 bag converter |

`livox-viz` depends on pangolino by path, so it must be checked out next to
this repository (`../pangolino`); the other crates need nothing else.

## Using the driver

```rust
use livox_rust::{Config, Device, Packet};

let (mut device, packets) = Device::open(Config::new("192.168.1.135".parse()?))?;
device.start()?; // enables the IMU and starts sampling; waits for the lidar's ack

for packet in packets {
    match packet {
        Packet::Points(p) => {
            for point in p.points.iter().filter(|pt| pt.is_valid()) {
                let t_ns = p.header.timestamp_ns + point.offset_ns as u64;
                // point.xyz (m), point.reflectivity, point.tag
            }
        }
        Packet::Imu { sample, .. } => {
            // sample.gyro (rad/s), sample.accel (g), sample.timestamp_ns
        }
    }
}
// Dropping the device puts the lidar back to idle.
```

`Device::open` binds the host ports and starts one receive thread per stream.
Packets arrive on a bounded channel; `device.stats()` counts packets, CRC and
parse errors, sequence gaps (packets lost before they reached us) and drops
(packets discarded because the consumer fell behind). `Config` holds the
ports (Mid-360 defaults), the socket buffer size and an optional recording
path.

### Network setup

The lidar sends its data to the host IP stored in its own configuration
(192.168.1.50 by default on a Mid-360), not to whoever sent the start command.
Give the wired interface that address, e.g. on macOS:

```sh
sudo networksetup -setmanual "<adapter name>" 192.168.1.50 255.255.255.0
```

The driver asks for 8 MB receive buffers and warns if the OS grants much
less (`kern.ipc.maxsockbuf` on macOS, `net.core.rmem_max` on Linux).

## Tools

Every tool takes the same source options:

```
  [LIDAR_IP [HOST_IP]]             live lidar (default 192.168.1.135, all interfaces)
  --sim                            simulated lidar on loopback
  --replay FILE [--speed X] [--loop]
                                   replay a pcap recording (or a tcpdump capture)
  --record FILE                    also record everything received to a pcap file
```

```sh
cargo run --release -p livox-viz -- 192.168.1.135             # view a live lidar
cargo run --release -p livox-viz -- --sim                      # no hardware needed
cargo run --release --example record -- run.pcap --duration 60 # record without a viewer
cargo run --release -p livox-viz -- --replay run.pcap          # play it back
```

In the viewer, left drag orbits, right drag pans and scroll zooms; the
controls set the time window of points shown and the colouring (height or
reflectivity).

### Recordings

Recordings are classic pcap files of the lidar's UDP datagrams, byte for byte,
with capture timestamps: the format `tcpdump -w` writes and Wireshark reads.
A capture taken with `tcpdump -i <iface> -w run.pcap udp` replays as well.

Replay (`replay::ReplayLidar`) acts as the lidar on 127.0.0.1: it
acknowledges the start and stop commands and re-sends the datagrams at their
recorded pace, so the program under test just opens a `Device`. For batch
processing without real-time pacing, read the file directly with
`pcap::Reader` and `data_packet::parse`.

### Importing datasets

`bag2pcap` converts a ROS 2 bag (sqlite3 `.db3`) holding
`livox_ros_driver2/msg/CustomMsg` points and `sensor_msgs/msg/Imu` into a
recording. It rebuilds the lidar's own packets from the driver's frames (the
driver appends packets in arrival order, which the converter undoes), writes
IMU acceleration in g as the lidar does, and reports how closely the point
times were reproduced.

```sh
cargo run --release -p livox-import -- BAG_DIR -o out.pcap
```

Topics are found by type (`--lidar-topic` / `--imu-topic` to choose).
ROS 1 `.bag` and `.mcap` are not supported yet.

#### Example: from download to viewer

The outdoor sequences of the Hard Point Cloud Localization Dataset
([Zenodo 10122133](https://zenodo.org/records/10122133), CC-BY 4.0) were
recorded with a Mid-360. The smallest is 651 MB:

```sh
curl -L -o outdoor_kidnap_a.zip "https://zenodo.org/records/10122133/files/outdoor_kidnap_a.zip?download=1"
unzip outdoor_kidnap_a.zip
cargo run --release -p livox-import -- outdoor_kidnap_a -o outdoor_kidnap_a.pcap
cargo run --release -p livox-viz -- --replay outdoor_kidnap_a.pcap
```

The conversion should report 96 points per packet (the original packets) and
point times within 0.1 us.

## Tests

```sh
cargo test -p livox-rust -p livox-import
```

The tests run the whole path over loopback against the simulator: live
streaming, and a recording that is replayed and checked packet for packet
against the live stream.

## Not yet confirmed on hardware

- That the packet `crc32` covers the timestamp and payload. Mismatches are
  counted in the stats, not rejected, so a non-zero count means otherwise.
- That points are evenly spaced over a packet's `time_interval`. The dataset
  above agrees, but its per-point times were produced by the ROS driver from
  the same assumption.
