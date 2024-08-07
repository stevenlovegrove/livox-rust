use crc::Crc;

pub const CRC_HEADER: Crc<u16> = crc::Crc::<u16>::new(&crc::Algorithm {
  width: 16,
  poly: 0x1021,
  init: 0xFFFF,
  refin: false,
  refout: false,
  xorout: 0x0000,
  check: 0x29B1,   // optional: a known good value for a specific input
  residue: 0x0000, // optional: the residue of a valid input
});

pub const CRC_DATA: Crc<u32> = crc::Crc::<u32>::new(&crc::Algorithm {
  width: 32,
  poly: 0x04C11DB7,
  init: 0xFFFFFFFF,
  refin: true,
  refout: true,
  xorout: 0xFFFFFFFF,
  check: 0xCBF43926,
  residue: 0xDEBB20E3,
});
