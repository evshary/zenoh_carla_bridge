use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{client::Sensor, prelude::*};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::prelude::sync::*;

pub struct SensorBridge {
    _sensor_type: String,
    _actor: Sensor,
}

impl SensorBridge {
    pub fn new(_z_session: &Session, actor: Sensor) -> Result<SensorBridge> {
        let sensor_type = actor.type_id();
        info!("Detect sensors {sensor_type}");
        Ok(SensorBridge {
            _sensor_type: sensor_type,
            _actor: actor,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _stamp: &Time, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
