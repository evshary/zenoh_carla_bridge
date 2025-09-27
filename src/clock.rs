use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use cdr::{CdrLe, Infinite};
use zenoh::{pubsub::Publisher, Session, Wait};
use zenoh_ros_type::{builtin_interfaces, rosgraph_msgs};

use crate::{autoware::topic, error::Result, put_with_attachment, utils, Mode};

pub struct SimulatorClock<'a> {
    publisher_clock: Publisher<'a>,
    attachment: Vec<u8>,
    mode: Mode,
}

impl<'a> SimulatorClock<'a> {
    pub fn new(z_session: Arc<Session>, mode: Mode) -> Result<SimulatorClock<'a>> {
        let key = match mode {
            Mode::DDS => topic("*/rt/", "clock"),
            _ => topic("*/", "clock"),
        };
        let publisher_clock = z_session.declare_publisher(key).wait()?;

        // Generate rmw_zenoh-compatible attachment
        let attachment = utils::generate_attachment();

        Ok(SimulatorClock {
            publisher_clock,
            attachment,
            mode,
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

        put_with_attachment!(self.publisher_clock, encoded, self.attachment, self.mode)?;
        Ok(())
    }
}
