use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::client::TrafficLight;
use r2r::builtin_interfaces::msg::Time;
use zenoh::prelude::sync::*;

pub struct TrafficLightBridge {
    _actor: TrafficLight,
}

impl TrafficLightBridge {
    pub fn new(_z_session: &Session, _actor: TrafficLight) -> Result<TrafficLightBridge> {
        Ok(TrafficLightBridge { _actor })
    }
}

impl ActorBridge for TrafficLightBridge {
    fn step(&mut self, _stamp: &Time, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
