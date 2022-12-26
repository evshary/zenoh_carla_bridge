use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone)]
pub struct TimeStamp {
    pub sec: i32,
    pub nsec: u32,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct ServiceHeader {
    pub guid: i64,
    pub seq: u64,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct GateMode {
    pub data: u8, // 0: AUTO, 1: EXTERNAL
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct GetEngage {
    pub ts: TimeStamp,
    pub enable: bool,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct Engage {
    pub header: ServiceHeader,
    pub enable: bool,
}

/* We don't need to get service response currently
#[derive(Serialize, Deserialize, PartialEq)]
struct ResponseStatus {
    header: ServiceHeader,
    code: u32,
    message: String,
}
*/

#[allow(dead_code)]
pub const GEAR_CMD_DRIVE: u8 = 2;
#[allow(dead_code)]
pub const GEAR_CMD_REVERSE: u8 = 20;
#[allow(dead_code)]
pub const GEAR_CMD_PARK: u8 = 22;
#[allow(dead_code)]
pub const GEAR_CMD_LOW: u8 = 23;
#[derive(Serialize, Deserialize, PartialEq)]
pub struct GearCommand {
    pub ts: TimeStamp,
    pub command: u8,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct StdMsgsHeader {
    pub ts: TimeStamp,
    pub frameid: String,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct CurrentVelocity {
    pub header: StdMsgsHeader,
    pub longitudinal_velocity: f32,
    pub lateral_velocity: f32,
    pub heading_rate: f32,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct AckermannLateralCommand {
    pub ts: TimeStamp,
    pub steering_tire_angle: f32,
    pub steering_tire_rotation_rate: f32,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct LongitudinalCommand {
    pub ts: TimeStamp,
    pub speed: f32,
    pub acceleration: f32,
    pub jerk: f32,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct AckermannControlCommand {
    pub ts: TimeStamp,
    pub lateral: AckermannLateralCommand,
    pub longitudinal: LongitudinalCommand,
}
