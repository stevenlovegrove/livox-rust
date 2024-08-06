use crate::datatype::*;
use binrw::{BinRead, BinWrite, VecArgs};

#[derive(BinRead, Debug)]
#[brw(little)]
pub struct LidarPacket {
    version: u8,
    length: u16,
    time_interval: u16,
    dot_num: u16,
    udp_cnt: u16,
    frame_cnt: u8,
    data_type: DataType,
    time_type: TimeType,
    reserved: [u8; 12],
    crc32: u32,
    timestamp: u64,
    #[br(args(data_type, dot_num))]
    data: Data,
}

#[derive(Debug)]
pub enum Data {
    IMU(Vec<IMUData>),
    PointCloud1(Vec<Point3D<i32>>),
    PointCloud2(Vec<Point3D<i16>>),
    PointCloud3(Vec<Bearing>),
}

impl BinRead for Data {
    type Args<'a> = (DataType, u16);

    fn read_options<R: std::io::Read + std::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let (data_type, num_points) = args;
        let args = VecArgs {
            count: num_points as usize,
            inner: (),
        };

        match data_type {
            DataType::IMUData => {
                let data = Vec::<IMUData>::read_options(reader, endian, args)?;
                Ok(Data::IMU(data))
            }
            DataType::PointCloudData1 => {
                let data = Vec::<Point3D<i32>>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud1(data))
            }
            DataType::PointCloudData2 => {
                let data = Vec::<Point3D<i16>>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud2(data))
            }
            DataType::PointCloudData3 => {
                let data = Vec::<Bearing>::read_options(reader, endian, args)?;
                Ok(Data::PointCloud3(data))
            }
        }
    }
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
pub struct IMUData {
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
}

#[derive(Debug)]
pub struct Point3D<T: num_traits::Num> {
    x: T,
    y: T,
    z: T,
    reflectivity: u8,
    tag: u8,
}

#[derive(BinRead, BinWrite, Debug)]
#[brw(little)]
pub struct Bearing {
    depth: u32,
    theta: u16,
    phi: u16,
    reflectivity: u8,
    tag: u8,
}

impl<T: BinRead + num_traits::Num> BinRead for Point3D<T>
where
    for<'a> T: BinRead<Args<'a> = ()>,
{
    type Args<'a> = ();

    fn read_options<R: std::io::Read + std::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let x = T::read_options(reader, endian, ())?;
        let y = T::read_options(reader, endian, ())?;
        let z = T::read_options(reader, endian, ())?;
        let reflectivity = u8::read_options(reader, endian, ())?;
        let tag = u8::read_options(reader, endian, ())?;
        Ok(Point3D {
            x,
            y,
            z,
            reflectivity,
            tag,
        })
    }
}
