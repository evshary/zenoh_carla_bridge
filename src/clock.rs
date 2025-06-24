use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use cdr::{CdrLe, Infinite};
use zenoh::{pubsub::Publisher, Session, Wait};
use zenoh_ros_type::{builtin_interfaces, rosgraph_msgs};

use crate::error::Result;
use crate::Mode;

pub struct SimulatorClock<'a> {
    publisher_clock: Publisher<'a>,
    attachment: Vec<u8>,
}

impl<'a> SimulatorClock<'a> {
    pub fn new(z_session: Arc<Session>, mode: Mode) -> Result<SimulatorClock<'a>> {
        let key = match mode {
            Mode::RmwZenoh => "*/*/clock/*/*".to_string(),
            Mode::ROS2 => "*/clock".to_string(),
            Mode::DDS => "*/rt/clock".to_string(),
        };
        let publisher_clock = z_session.declare_publisher(key).wait()?;

        // Initialize attachment
        let seq_num: i64 = 1;
        let mut attachment = seq_num.to_le_bytes().to_vec();
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
        let timestamp: i64 = now.as_nanos() as i64;
        attachment.extend_from_slice(&timestamp.to_le_bytes());
        attachment.push(16u8);
        attachment.extend_from_slice(&[0xAB; 16]);

        Ok(SimulatorClock {
            publisher_clock,
            attachment,
        })
    }

    pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
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
        let clock_msg = rosgraph_msgs::Clock { clock: time };
        let encoded = cdr::serialize::<_, _, CdrLe>(&clock_msg, Infinite)?;
        self.publisher_clock
            .put(encoded)
            .attachment(self.attachment.clone())
            .wait()?;
        Ok(())
    }
}
