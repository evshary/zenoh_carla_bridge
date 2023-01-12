use crate::error::Result;
use r2r::{std_msgs::msg::Header, Clock, ClockType};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

pub fn create_ros_header() -> Result<Header> {
    let mut clock = Clock::create(ClockType::RosTime)?;
    let duration = clock.get_now()?;
    let time = Clock::to_builtin_time(&duration);
    Ok(Header {
        stamp: time,
        frame_id: "".to_string(),
    })
}
