use std::io::Cursor;

use binrw::BinWrite;

use crate::datatype::*;
use crate::key::*;
use crate::control_packet::*;
use crate::crc::*;

pub async fn packet_set_params(params : KeyValueList) -> anyhow::Result<Vec<u8>> {
  let data = {
      let mut cursor = Cursor::new(Vec::new());
      params.write(&mut cursor)?;
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

pub async fn packet_start() -> anyhow::Result<Vec<u8>> {
    let params = vec![KeyValueItem {
        key: Key::WorkTgtMode,
        value: KeyValue::WorkTgtMode(WorkState::Sampling),
    }];
    packet_set_params(KeyValueList { items: params }).await
}

pub async fn packet_stop() -> anyhow::Result<Vec<u8>> {
    let params = vec![KeyValueItem {
        key: Key::WorkTgtMode,
        value: KeyValue::WorkTgtMode(WorkState::IDLE),
    }];
    packet_set_params(KeyValueList { items: params }).await
}
