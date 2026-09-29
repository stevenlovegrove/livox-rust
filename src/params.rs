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

    let mut cursor = Cursor::new(Vec::new());
    let header = ControlCommandPacketHeader {
        length: u16::try_from(24 + data.len())?,
        seq_num,
        cmd_id: CommandID::LidarParamInfoConfig,
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
}
