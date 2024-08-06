use binrw::BinWrite;

use crate::key::KeyValue;

pub(crate) trait SerializedSize {
  fn serialized_size(&self) -> usize;
}

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

impl SerializedSize for KeyValue {
  fn serialized_size(&self) -> usize {
      let mut counter = SizeCounter::new();
      self.write(&mut counter).unwrap();
      counter.size()
  }
}
