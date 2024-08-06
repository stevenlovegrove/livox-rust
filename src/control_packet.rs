use crate::datatype::*;
use binrw::binrw;

#[binrw]
#[derive(Debug)]
#[brw(magic = 0xAAu8)]
#[brw(little)]
pub struct ControlCommandPacketHeader {
    #[bw(calc(0x0))]
    _version: u8,
    // #[bw(try_calc(u16::try_from(data.len() + 24)))]
    pub length: u16,
    pub seq_num: u32,
    pub cmd_id: CommandID,
    pub cmd_type: CommandType,
    pub sender_type: SenderType,
    #[bw(calc([0; 6]))]
    _reserved: [u8; 6],
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
pub struct ControlCommandPacket {
    pub header: ControlCommandPacketHeader,

    pub crc16: u16,
    pub crc32: u32,
    // #[br(count = header.length - 24)]
    #[br(count = header.length - 24)]
    pub data: Vec<u8>,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u8))]
pub enum CommandType {
    Req = 0,
    Ack = 1,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u8))]
pub enum SenderType {
    HostComputer = 0,
    Lidar = 1,
}

#[binrw]
#[derive(Debug)]
#[brw(little)]
pub struct ControlCommandParamConfigAck {
    ret_code: RetCode,
    error_key: u16,
}

#[binrw]
#[derive(Debug)]
#[brw(repr(u16))]
pub enum CommandID {
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
