use binrw::{BinRead, BinWrite};

#[derive(BinRead, BinWrite, Debug, Copy, Clone, PartialEq)]
#[brw(repr(u8))]
pub enum RetCode {
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
pub enum DataType {
    IMUData = 0,
    PointCloudData1 = 1,
    PointCloudData2 = 2,
    PointCloudData3 = 3,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
pub enum TimeType {
    NoSync = 0,
    GPTP = 1,
    GPS = 2,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
pub enum PatternMode {
    NonRepetitiveScan = 0,
    _RepetitiveScan = 1,
    _LowFrameRateRepetitiveScan = 2,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
pub struct LidarIpCfg {
    ip: [u8; 4],
    subnet: [u8; 4],
    gateway: [u8; 4],
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
pub struct HostIpCfg {
    ip: [u8; 4],
    dst_port: u16,
    src_port: u16,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(repr(u8))]
pub enum WorkState {
    Sampling = 0x1,
    IDLE = 0x2,
    Error = 0x4,
    SelfCheck = 0x5,
    MotorStartup = 0x6,
    Upgrade = 0x8,
    Ready = 0x9,
}
