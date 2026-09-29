use crc::{Crc, Table};

// `static`, not `const`: a const is a fresh copy of the lookup table at each use.

pub static CRC_HEADER: Crc<u16> = Crc::<u16>::new(&crc::Algorithm {
  width: 16,
  poly: 0x1021,
  init: 0xFFFF,
  refin: false,
  refout: false,
  xorout: 0x0000,
  check: 0x29B1,   // optional: a known good value for a specific input
  residue: 0x0000, // optional: the residue of a valid input
});

// Slice-by-16: this one runs over every data packet (~2 kHz).
pub static CRC_DATA: Crc<u32, Table<16>> = Crc::<u32, Table<16>>::new(&crc::Algorithm {
  width: 32,
  poly: 0x04C11DB7,
  init: 0xFFFFFFFF,
  refin: true,
  refout: true,
  xorout: 0xFFFFFFFF,
  check: 0xCBF43926,
  residue: 0xDEBB20E3,
});
