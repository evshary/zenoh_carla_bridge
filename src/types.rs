use serde::Deserialize;
use serde_big_array::BigArray;
use zenoh_ros_type::{geometry_msgs, std_msgs};

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

// /initialpose payload — geometry_msgs/PoseWithCovarianceStamped.
//
// zenoh-ros-type 0.3.7 ships Pose / Quaternion / Point but not the covariance
// variants, so we declare them locally for CDR deserialization. CDR is a
// sequential binary format and can't skip fields, so the covariance bytes
// must still be parsed even though the bridge only reads the inner pose.
#[derive(Debug, Deserialize)]
pub struct PoseWithCovariance {
    pub pose: geometry_msgs::Pose,
    // serde_derive only supports arrays up to length 32; BigArray handles 36.
    #[serde(with = "BigArray")]
    #[allow(dead_code)]
    pub covariance: [f64; 36],
}

#[derive(Debug, Deserialize)]
pub struct PoseWithCovarianceStamped {
    #[allow(dead_code)]
    pub header: std_msgs::Header,
    pub pose: PoseWithCovariance,
}
