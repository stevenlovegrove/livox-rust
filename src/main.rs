use binrw::{binrw, io::Cursor, meta::WriteEndian, BinRead, BinWrite, VecArgs};
use crc::*;
use get_if_addrs::get_if_addrs;
use pnet::datalink::{self, NetworkInterface};
use std::{
    io::{Read, Write},
    net::SocketAddr,
};
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

struct SizeCounter {
    pos: usize,
    max_size: usize,
}

impl std::io::Write for SizeCounter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let len = buf.len();
        self.pos += len;
        self.max_size = self.max_size.max(self.pos);
        Ok(len)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl std::io::Seek for SizeCounter {
    fn seek(&mut self, pos: std::io::SeekFrom) -> std::io::Result<u64> {
        match pos {
            std::io::SeekFrom::Start(pos) => {
                self.pos = pos as usize;
            }
            std::io::SeekFrom::End(pos) => {
                self.pos = self.max_size as usize + pos as usize;
            }
            std::io::SeekFrom::Current(pos) => {
                self.pos = self.pos + pos as usize;
            }
        }
        self.max_size = self.max_size.max(self.pos);
        Ok(self.pos as u64)
    }
}

impl SizeCounter {
    fn new() -> Self {
        Self {
            pos: 0,
            max_size: 0,
        }
    }

    fn size(&self) -> usize {
        self.max_size
    }
}

trait SerializedSize {
    fn serialized_size(&self) -> usize;
}

impl SerializedSize for KeyValue {
    fn serialized_size(&self) -> usize {
        let mut counter = SizeCounter::new();
        self.write(&mut counter).unwrap();
        counter.size()
    }
}

#[derive(BinRead, Debug)]
#[brw(little)]
struct LidarPacket {
    version: u8,
    length: u16,
    time_interval: u16,
    dot_num: u16,
    udp_cnt: u16,
    frame_cnt: u8,
    data_type: DataType,
    time_type: TimeType,
    reserved: [u8; 12],
    crc32: u32,
    timestamp: u64,
    #[br(args(data_type, dot_num))]
    data: Data,
}

#[derive(BinRead, BinWrite, Debug, Copy, Clone, PartialEq)]
#[brw(repr(u8))]
enum RetCode {
    LvxRetSuccess = 0x00,             // Execution succeed
    LvxRetFailure = 0x01,             // Execution failed
    LvxRetNotPermitNow = 0x02,        // Current state does not support
    LvxRetOutOfRange = 0x03,          // Setting value out of range
    LvxRetParamNotsupport = 0x20,     // The parameter is not supported
    LvxRetParamRebootEffect = 0x21,   // Parameters need to reboot to take effect
    LvxRetParamRdOnly = 0x22,         // The parameter is read-only and cannot be written
    LvxRetParamInvalidLen = 0x23, // The request parameter length is wrong, or the ack packet exceeds the maximum length
    LvxRetParamKeyNumErr = 0x24,  // Parameter key_ num and key_ list mismatch
    LvxRetUpgradePubKeyError = 0x30, // Public key signature verification error
    LvxRetUpgradeDigestError = 0x31, // Digest check error
    LvxRetUpgradeFwTypeError = 0x32, // Firmware type mismatch
    LvxRetUpgradeFwOutOfRange = 0x33, // Firmware length out of range
    LvxRetUpgradeFwErasing = 0x34, // Firmware erasing
}

#[derive(BinRead, BinWrite, Debug, Copy, Clone, PartialEq)]
#[brw(repr(u8))]
enum DataType {
    IMUData = 0,
    PointCloudData1 = 1,
    PointCloudData2 = 2,
    PointCloudData3 = 3,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
enum TimeType {
    NoSync = 0,
    GPTP = 1,
    GPS = 2,
}

#[derive(Debug)]
enum Data {
    IMU(Vec<IMUData>),
    PointCloud1(Vec<Point3D<i32>>),
    PointCloud2(Vec<Point3D<i16>>),
    PointCloud3(Vec<Bearing>),
}

impl BinRead for Data {
    type Args<'a> = (DataType, u16);

    fn read_options<R: Read + std::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let (data_type, num_points) = args;
        let args = VecArgs {
            count: num_points as usize,
            inner: (),
        };

        match data_type {
            DataType::IMUData => {
                let data = Vec::<IMUData>::read_options(reader, endian, args)?;
                Ok(Data::IMU(data))
            }
            DataType::PointCloudData1 => {
                let data = Vec::<Point3D<i32>>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud1(data))
            }
            DataType::PointCloudData2 => {
                let data = Vec::<Point3D<i16>>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud2(data))
            }
            DataType::PointCloudData3 => {
                let data = Vec::<Bearing>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud3(data))
            }
        }
    }
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
enum PatternMode {
    NonRepetitiveScan = 0,
    _RepetitiveScan = 1,
    _LowFrameRateRepetitiveScan = 2,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
struct LidarIpCfg {
    ip: [u8; 4],
    subnet: [u8; 4],
    gateway: [u8; 4],
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
struct HostIpCfg {
    ip: [u8; 4],
    dst_port: u16,
    src_port: u16,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
enum WorkState {
    Sampling = 0x1,
    IDLE = 0x2,
    Error = 0x4,
    SelfCheck = 0x5,
    MotorStartup = 0x6,
    Upgrade = 0x8,
    Ready = 0x9,
}

#[derive(BinRead, BinWrite, Debug, PartialEq, Copy, Clone)]
#[brw(repr(u16))]
enum Key {
    PclDataType = 0x0000,
    PatternMode = 0x0001,
    LidarIpCfg = 0x0004,
    StateInfoHostIpCfg = 0x0005,
    PointCloudHostIpCfg = 0x0006,
    ImuHostIpCfg = 0x0007,
    InstallAttitude = 0x0012,
    FovCfg0 = 0x0015,
    FovCfg1 = 0x0016,
    FovCfgEn = 0x0017,
    DetectModeCfg = 0x0018,
    FuncIoCfg = 0x0019,
    WorkTgtMode = 0x001A,
    ImuDataEnable = 0x001C,
    Sn = 0x8000,
    ProductInfo = 0x8001,
    VersionApp = 0x8002,
    VersionLoader = 0x8003,
    VersionHardware = 0x8004,
    Mac = 0x8005,
    CurWorkState = 0x8006,
    CoreTemp = 0x8007,
    PowerUpCount = 0x8008,
    LocalTimeNow = 0x8009,
    LastSyncTime = 0x800A,
    TimeOffset = 0x800B,
    TimeSyncType = 0x800C,
    LidarDiagStatus = 0x800E,
    FwType = 0x8010,
    HmsCode = 0x8011,
    Unknown = 0xFFFF,
}

impl Default for Key {
    fn default() -> Self {
        Key::Unknown
    }
}

#[derive(BinRead, BinWrite, Debug)]
#[br(import(key: Key))]
#[brw(little)]
enum KeyValue {
    #[br(pre_assert(key == Key::PclDataType))]
    PclDataType(DataType),
    #[br(pre_assert(key == Key::PatternMode))]
    PatternMode(PatternMode),
    #[br(pre_assert(key == Key::LidarIpCfg))]
    LidarIpCfg(LidarIpCfg),
    #[br(pre_assert(key == Key::StateInfoHostIpCfg))]
    StateInfoHostIpCfg(HostIpCfg),
    #[br(pre_assert(key == Key::PointCloudHostIpCfg))]
    PointCloudHostIpCfg(HostIpCfg),
    #[br(pre_assert(key == Key::ImuHostIpCfg))]
    ImuHostIpCfg(HostIpCfg),
    #[br(pre_assert(key == Key::InstallAttitude))]
    InstallAttitude([u8; 24]),
    #[br(pre_assert(key == Key::FovCfg0))]
    FovCfg0([u8; 20]),
    #[br(pre_assert(key == Key::FovCfg1))]
    FovCfg1([u8; 20]),
    #[br(pre_assert(key == Key::FovCfgEn))]
    FovCfgEn(u8),
    #[br(pre_assert(key == Key::DetectModeCfg))]
    DetectModeCfg(u8),
    #[br(pre_assert(key == Key::FuncIoCfg))]
    FuncIoCfg([u8; 4]),
    #[br(pre_assert(key == Key::WorkTgtMode))]
    WorkTgtMode(WorkState),
    #[br(pre_assert(key == Key::ImuDataEnable))]
    ImuDataEnable(u8),
    #[br(pre_assert(key == Key::Sn))]
    Sn([u8; 16]),
    #[br(pre_assert(key == Key::ProductInfo))]
    ProductInfo([u8; 64]),
    #[br(pre_assert(key == Key::VersionApp))]
    VersionApp([u8; 4]),
    #[br(pre_assert(key == Key::VersionLoader))]
    VersionLoader([u8; 4]),
    #[br(pre_assert(key == Key::VersionHardware))]
    VersionHardware([u8; 4]),
    #[br(pre_assert(key == Key::Mac))]
    Mac([u8; 6]),
    #[br(pre_assert(key == Key::CurWorkState))]
    CurWorkState(WorkState),
    #[br(pre_assert(key == Key::CoreTemp))]
    CoreTemp(u32),
    #[br(pre_assert(key == Key::PowerUpCount))]
    PowerUpCount(u32),
    #[br(pre_assert(key == Key::LocalTimeNow))]
    LocalTimeNow(u64),
    #[br(pre_assert(key == Key::LastSyncTime))]
    LastSyncTime(u64),
    #[br(pre_assert(key == Key::TimeOffset))]
    TimeOffset(i64),
    #[br(pre_assert(key == Key::TimeSyncType))]
    TimeSyncType(u8),
    #[br(pre_assert(key == Key::LidarDiagStatus))]
    LidarDiagStatus(u16),
    #[br(pre_assert(key == Key::FwType))]
    FwType(u8),
    #[br(pre_assert(key == Key::HmsCode))]
    HmsCode(u8),

    #[br(pre_assert(key == Key::Unknown))]
    Unknown,
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
struct KeyValueItem {
    key: Key,
    #[bw(calc(value.serialized_size() as u16))]
    length: u16,
    #[br(args(key))]
    value: KeyValue,
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
struct KeyValueList {
    #[bw(calc(items.len() as u16))]
    length: u16,
    #[bw(calc([0; 2]))]
    _reserved: [u8; 2],
    #[br(count = length)]
    items: Vec<KeyValueItem>,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
struct IMUData {
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
}

#[derive(Debug)]
struct Point3D<T: num_traits::Num> {
    x: T,
    y: T,
    z: T,
    reflectivity: u8,
    tag: u8,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
struct Bearing {
    depth: u32,
    theta: u16,
    phi: u16,
    reflectivity: u8,
    tag: u8,
}

impl<T: BinRead + num_traits::Num> BinRead for Point3D<T>
where
    for<'a> T: BinRead<Args<'a> = ()>,
{
    type Args<'a> = ();

    fn read_options<R: Read + std::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let x = T::read_options(reader, endian, ())?;
        let y = T::read_options(reader, endian, ())?;
        let z = T::read_options(reader, endian, ())?;
        let reflectivity = u8::read_options(reader, endian, ())?;
        let tag = u8::read_options(reader, endian, ())?;
        Ok(Point3D {
            x,
            y,
            z,
            reflectivity,
            tag,
        })
    }
}

#[binrw]
#[derive(Debug)]
#[brw(magic = 0xAAu8)]
#[brw(little)]
struct ControlCommandPacketHeader {
    #[bw(calc(0x0))]
    _version: u8,
    // #[bw(try_calc(u16::try_from(data.len() + 24)))]
    length: u16,
    seq_num: u32,
    cmd_id: CommandID,
    cmd_type: CommandType,
    sender_type: SenderType,
    #[bw(calc([0; 6]))]
    _reserved: [u8; 6],
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
struct ControlCommandParamConfigAck {
    ret_code: RetCode,
    error_key: u16,
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
struct ControlCommandPacket {
    header: ControlCommandPacketHeader,

    crc16: u16,
    crc32: u32,
    // #[br(count = header.length - 24)]
    #[br(count = header.length - 24)]
    data: Vec<u8>,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u8))]
enum CommandType {
    Req = 0,
    Ack = 1,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u8))]
enum SenderType {
    HostComputer = 0,
    Lidar = 1,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u16))]
enum CommandID {
    DeviceTypeQuery = 0x0000,
    LidarParamInfoConfig = 0x0100,
    LidarInquireInfo = 0x0101,
    LidarPushInfo = 0x0102,
    ControlCmdReboot = 0x0200,
    ControlCmdFactoryReset = 0x0201,
    ControlCmdSetGpsTimestamp = 0x0202,
    LogCmdLogFilePush = 0x0300,
    LogCmdLogCollectionConfig = 0x0301,
    LogCmdLogSystemTimeSync = 0x0302,
    LogCmdDebugRawDataCollectionConfig = 0x0303,
    GeneralUpgradeCmdRequestStart = 0x0400,
    GeneralUpgradeCmdFirmwareDataTransfer = 0x0401,
    GeneralUpgradeCmdFirmwareTransferComplete = 0x0402,
    GeneralUpgradeCmdGetFirmwareUpgradeStatus = 0x0403,
}

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
