use std::io::Cursor;

use binrw::BinWrite;

use crate::control_packet::*;
use crate::crc::*;
use crate::datatype::*;
use crate::key::*;

/// Encodes a `LidarParamInfoConfig` request that writes `params`.
pub fn packet_set_params(seq_num: u32, params: KeyValueList) -> anyhow::Result<Vec<u8>> {
    let data = {
        let mut cursor = Cursor::new(Vec::new());
        params.write(&mut cursor)?;
        cursor.into_inner()
    };
    packet_request(seq_num, CommandID::LidarParamInfoConfig, data)
}

/// Encodes a `LidarInquireInfo` request that reads `keys`.
pub fn packet_query(seq_num: u32, keys: &[Key]) -> anyhow::Result<Vec<u8>> {
    // key_num, reserved, then the keys.
    let mut data = u16::try_from(keys.len())?.to_le_bytes().to_vec();
    data.extend_from_slice(&[0, 0]);
    for &key in keys {
        data.extend_from_slice(&(key as u16).to_le_bytes());
    }
    packet_request(seq_num, CommandID::LidarInquireInfo, data)
}

/// Decodes a `LidarInquireInfo` ack body: (key, raw little-endian value).
pub fn parse_query_ack(data: &[u8]) -> anyhow::Result<Vec<(u16, Vec<u8>)>> {
    anyhow::ensure!(data.len() >= 3, "short query ack");
    anyhow::ensure!(data[0] == 0, "lidar refused the query (ret_code {})", data[0]);
    let count = u16::from_le_bytes([data[1], data[2]]) as usize;
    let mut items = Vec::with_capacity(count);
    let mut at = 3;
    for _ in 0..count {
        anyhow::ensure!(at + 4 <= data.len(), "truncated query ack");
        let key = u16::from_le_bytes([data[at], data[at + 1]]);
        let len = u16::from_le_bytes([data[at + 2], data[at + 3]]) as usize;
        at += 4;
        anyhow::ensure!(at + len <= data.len(), "truncated query ack");
        items.push((key, data[at..at + len].to_vec()));
        at += len;
    }
    Ok(items)
}

fn packet_request(seq_num: u32, cmd_id: CommandID, data: Vec<u8>) -> anyhow::Result<Vec<u8>> {
    let mut cursor = Cursor::new(Vec::new());
    let header = ControlCommandPacketHeader {
        length: u16::try_from(24 + data.len())?,
        seq_num,
        cmd_id,
        cmd_type: CommandType::Req,
        sender_type: SenderType::HostComputer,
    };
    header.write(&mut cursor)?;
    let header_crc = CRC_HEADER.checksum(&cursor.get_ref()[..18]);
    let data_crc = CRC_DATA.checksum(&data);

    cursor.set_position(0);

    let packet = ControlCommandPacket {
        header,
        crc16: header_crc,
        crc32: data_crc,
        data,
    };
    packet.write(&mut cursor)?;
    Ok(cursor.into_inner())
}

pub fn params_work_mode(state: WorkState) -> KeyValueList {
    KeyValueList {
        items: vec![KeyValueItem {
            key: Key::WorkTgtMode,
            value: KeyValue::WorkTgtMode(state),
        }],
    }
}

pub fn params_imu_enable(enable: bool) -> KeyValueList {
    KeyValueList {
        items: vec![KeyValueItem {
            key: Key::ImuDataEnable,
            value: KeyValue::ImuDataEnable(enable as u8),
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use binrw::BinRead;

    #[test]
    fn set_params_round_trip() {
        let bytes = packet_set_params(7, params_work_mode(WorkState::Sampling)).unwrap();
        // header(24) + key_num(2) + rsvd(2) + key(2) + len(2) + value(1)
        assert_eq!(bytes.len(), 33);
        assert_eq!(bytes[0], 0xAA);
        let packet = ControlCommandPacket::read(&mut Cursor::new(&bytes)).unwrap();
        assert_eq!(packet.header.length as usize, bytes.len());
        assert_eq!(packet.header.seq_num, 7);
        assert_eq!(packet.crc16, CRC_HEADER.checksum(&bytes[..18]));
        assert_eq!(packet.crc32, CRC_DATA.checksum(&bytes[24..]));
        let list = KeyValueList::read(&mut Cursor::new(&packet.data)).unwrap();
        assert_eq!(list.items.len(), 1);
        assert_eq!(list.items[0].key, Key::WorkTgtMode);
        assert_eq!(&bytes[28..], &[0x1A, 0x00, 0x01, 0x00, 0x01]);
    }

    #[test]
    fn query_request_and_ack() {
        let bytes = packet_query(3, &[Key::TimeSyncType, Key::TimeOffset]).unwrap();
        assert_eq!(bytes.len(), 24 + 8);
        assert_eq!(&bytes[24..], &[2, 0, 0, 0, 0x0C, 0x80, 0x0B, 0x80]);
        let ack = [0, 2, 0, 0x0C, 0x80, 1, 0, 1, 0x0B, 0x80, 8, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let items = parse_query_ack(&ack).unwrap();
        assert_eq!(items, vec![(0x800C, vec![1]), (0x800B, vec![0xFF; 8])]);
        assert!(parse_query_ack(&ack[..10]).is_err());
    }
}
