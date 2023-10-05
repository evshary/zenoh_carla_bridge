use std::time::{SystemTime, UNIX_EPOCH};
use zenoh_ros_type::{builtin_interfaces, std_msgs};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

pub fn create_ros_header(timestamp: Option<f64>) -> std_msgs::Header {
    let time = if let Some(sec) = timestamp {
        builtin_interfaces::Time {
            sec: sec.floor() as i32,
            nanosec: (sec.fract() * 1000_000_000_f64) as u32,
        }
    } else {
        // If there is no timestamp, use system time
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Unable to get current time");
        builtin_interfaces::Time {
            sec: now.as_secs() as i32,
            nanosec: now.subsec_nanos(),
        }
    };
    std_msgs::Header {
        stamp: time,
        frame_id: "".to_string(),
    }
}
