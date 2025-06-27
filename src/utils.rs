use std::time::{SystemTime, UNIX_EPOCH};

use zenoh_ros_type::{builtin_interfaces, std_msgs};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

pub fn create_ros_header(timestamp: Option<f64>) -> std_msgs::Header {
    let time = if let Some(sec) = timestamp {
        builtin_interfaces::Time {
            sec: sec.floor() as i32,
            nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
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

/// Utility function for generating the rmw_zenoh attachment format.
/// See: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#publishers
pub fn generate_attachment() -> Vec<u8> {
    let seq_num: i64 = 1;
    let mut attachment = seq_num.to_le_bytes().to_vec();
    attachment.extend_from_slice(&0i64.to_le_bytes());
    attachment.push(16u8);
    attachment.extend_from_slice(&[0xAB; 16]);
    attachment
}

// This macro will publish a message with or without an attachment depending on the mode.
// If the mode is RmwZenoh, it will add the attachment; otherwise, it will not.
#[macro_export]
macro_rules! put_with_attachment {
    ($publisher:expr, $payload:expr, $attachment:expr, $mode:expr) => {
        if $mode == $crate::Mode::RmwZenoh {
            $publisher
                .put($payload)
                .attachment($attachment.clone())
                .wait()
        } else {
            $publisher.put($payload).wait()
        }
    };
}
