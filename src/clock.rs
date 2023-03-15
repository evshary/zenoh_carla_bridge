use crate::error::Result;
use cdr::{CdrLe, Infinite};
use r2r::{builtin_interfaces::msg::Time, rosgraph_msgs::msg::Clock, Clock as r2rClock, ClockType};
use std::{sync::Arc, time::Duration};
use zenoh::{prelude::sync::*, publication::Publisher};

pub struct SimulatorClock<'a> {
    real_time: bool,
    publisher_clock: Publisher<'a>,
    duration: Duration,
}

impl<'a> SimulatorClock<'a> {
    pub fn new(z_session: Arc<Session>, real_time: bool) -> Result<SimulatorClock<'a>> {
        let publisher_clock = z_session.declare_publisher("*/rt/clock").res()?;
        let duration = Duration::new(0, 0);
        Ok(SimulatorClock {
            publisher_clock,
            real_time,
            duration,
        })
    }
    pub fn update_clock(&mut self, new_duration: Duration) {
        self.duration += new_duration;
    }
    /*
    pub fn get_clock(&self) -> Time {
        Time {
            sec: self.duration.as_secs() as i32,
            nanosec: self.duration.subsec_nanos(),
        }
    }
    */
    pub fn publish_clock(&self) -> Result<()> {
        let time = if self.real_time {
            let mut clock = r2rClock::create(ClockType::RosTime)?;
            let duration = clock.get_now()?;
            r2rClock::to_builtin_time(&duration)
        } else {
            Time {
                sec: self.duration.as_secs() as i32,
                nanosec: self.duration.subsec_nanos(),
            }
        };
        let clock_msg = Clock { clock: time };
        let encoded = cdr::serialize::<_, _, CdrLe>(&clock_msg, Infinite)?;
        self.publisher_clock.put(encoded).res()?;
        Ok(())
    }
}
