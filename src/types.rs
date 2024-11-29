#[derive(Debug)]
#[allow(dead_code)]
pub enum PointFieldType {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
}

#[derive(Debug)]
#[allow(dead_code)]
pub enum GnssStatus {
    NoFix = -1,
    Fix = 0,
    SbasFix = 1,
    GbasFix = 2,
}

#[derive(Debug)]
#[allow(dead_code)]
pub enum GnssService {
    Gps = 1,
    Glonass = 2,
    Compass = 4,
    Galileo = 8,
}
