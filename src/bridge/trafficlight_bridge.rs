use std::sync::Arc;

use carla::client::TrafficLight;
use zenoh::Session;

use super::actor_bridge::ActorBridge;
use crate::error::Result;

pub struct TrafficLightBridge {
    _actor: TrafficLight,
}

impl TrafficLightBridge {
    pub fn new(_z_session: Arc<Session>, _actor: TrafficLight) -> Result<TrafficLightBridge> {
        Ok(TrafficLightBridge { _actor })
    }
}

impl ActorBridge for TrafficLightBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}
