use crate::error::Result;
use cdr::{CdrLe, Infinite};
use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};
use zenoh::{prelude::sync::*, publication::Publisher};
use zenoh_ros_type::{builtin_interfaces, rosgraph_msgs};

pub struct SimulatorClock<'a> {
    publisher_clock: Publisher<'a>,
}

impl<'a> SimulatorClock<'a> {
    pub fn new(z_session: Arc<Session>, ros2: bool) -> Result<SimulatorClock<'a>> {
        let key = if ros2 { "*/clock" } else { "*/rt/clock" };
        let publisher_clock = z_session.declare_publisher(key).res()?;
        Ok(SimulatorClock { publisher_clock })
    }

    pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
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
        let clock_msg = rosgraph_msgs::Clock { clock: time };
        let encoded = cdr::serialize::<_, _, CdrLe>(&clock_msg, Infinite)?;
        self.publisher_clock.put(encoded).res()?;
        Ok(())
    }
}
