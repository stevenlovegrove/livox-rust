use crate::datatype::*;
use crate::key_size::SerializedSize;
use binrw::binrw;

#[binrw]
#[derive(Debug, PartialEq, Copy, Clone, Default)]
#[brw(repr(u16))]
pub enum Key {
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
    #[default]
    Unknown = 0xFFFF,
}


#[binrw]
#[derive(Debug)]
#[br(import(key: Key))]
#[brw(little)]
pub enum KeyValue {
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
pub struct KeyValueItem {
    pub key: Key,
    #[bw(calc(value.serialized_size() as u16))]
    length: u16,
    #[br(args(key))]
    pub value: KeyValue,
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
pub struct KeyValueList {
    #[bw(calc(items.len() as u16))]
    length: u16,
    #[bw(calc([0; 2]))]
    _reserved: [u8; 2],
    #[br(count = length)]
    pub items: Vec<KeyValueItem>,
}
