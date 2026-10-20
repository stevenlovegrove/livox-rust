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

#[derive(BinRead, BinWrite, Debug, Copy, Clone, PartialEq)]
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

/// The IMU's output rate and ranges (key 0x002B, `imu_sensor_cfg`), as the
/// lidar's codes: see [`ImuRate`] for `output_rate`; `accel_range` 0..=3 is
/// ±4, 8, 16, 32 g; `gyro_range` 0..=7 is ±2000 dps halving down to ±15.625.
/// The decoded samples are in rad/s and g whatever the ranges.
#[derive(BinRead, BinWrite, Debug, Copy, Clone, PartialEq, Eq)]
#[brw(little)]
pub struct ImuSensorCfg {
    pub output_rate: u8,
    pub accel_range: u8,
    pub gyro_range: u8,
}

impl ImuSensorCfg {
    /// The output rate, if its code is one the protocol documents.
    pub fn rate(&self) -> Option<ImuRate> {
        ImuRate::from_code(self.output_rate)
    }

    /// The accelerometer's full scale, ±g.
    pub fn accel_range_g(&self) -> Option<f64> {
        (self.accel_range <= 3).then(|| 4.0 * f64::from(1u8 << self.accel_range))
    }

    /// The gyro's full scale, ±deg/s.
    pub fn gyro_range_dps(&self) -> Option<f64> {
        (self.gyro_range <= 7).then(|| 2000.0 / f64::from(1u8 << self.gyro_range))
    }
}

/// The IMU's output rates (`imu_sensor_cfg`'s `output_rate`). 200 Hz is
/// what the lidar runs at unless told otherwise.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum ImuRate {
    Hz50,
    Hz100,
    #[default]
    Hz200,
    Hz500,
}

impl ImuRate {
    pub const ALL: [ImuRate; 4] = [ImuRate::Hz50, ImuRate::Hz100, ImuRate::Hz200, ImuRate::Hz500];

    /// The protocol's code for the rate (not in order of rate).
    pub fn code(self) -> u8 {
        match self {
            ImuRate::Hz200 => 0,
            ImuRate::Hz500 => 1,
            ImuRate::Hz100 => 2,
            ImuRate::Hz50 => 3,
        }
    }

    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|r| r.code() == code)
    }

    pub fn hz(self) -> u32 {
        match self {
            ImuRate::Hz50 => 50,
            ImuRate::Hz100 => 100,
            ImuRate::Hz200 => 200,
            ImuRate::Hz500 => 500,
        }
    }

    pub fn from_hz(hz: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|r| r.hz() == hz)
    }
}
