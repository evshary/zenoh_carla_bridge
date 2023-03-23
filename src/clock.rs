use crate::error::Result;
use cdr::{CdrLe, Infinite};
use r2r::{builtin_interfaces::msg::Time, rosgraph_msgs::msg::Clock, Clock as r2rClock, ClockType};
use std::sync::Arc;
use zenoh::{prelude::sync::*, publication::Publisher};

pub struct SimulatorClock<'a> {
    publisher_clock: Publisher<'a>,
}

impl<'a> SimulatorClock<'a> {
    pub fn new(z_session: Arc<Session>) -> Result<SimulatorClock<'a>> {
        let publisher_clock = z_session.declare_publisher("*/rt/clock").res()?;
        Ok(SimulatorClock { publisher_clock })
    }

    pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
        let time = if let Some(sec) = timestamp {
            Time {
                sec: sec.floor() as i32,
                nanosec: (sec.fract() * 1000_000_000_f64) as u32,
            }
        } else {
            let mut clock = r2rClock::create(ClockType::RosTime)?;
            let duration = clock.get_now()?;
            r2rClock::to_builtin_time(&duration)
        };
        let clock_msg = Clock { clock: time };
        let encoded = cdr::serialize::<_, _, CdrLe>(&clock_msg, Infinite)?;
        self.publisher_clock.put(encoded).res()?;
        Ok(())
    }
}
