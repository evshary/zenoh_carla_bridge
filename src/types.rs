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
    StatusNoFix = -1,
    StatusFix = 0,
    StatusSbasFix = 1,
    StatusGbasFix = 2,
}

#[derive(Debug)]
#[allow(dead_code)]
pub enum GnssService {
    ServiceGps = 1,
    ServiceGlonass = 2,
    ServiceCompass = 4,
    ServiceGalileo = 8,
}
