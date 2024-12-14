use std::sync::Arc;

use carla::client::TrafficSign;
use zenoh::Session;

use super::actor_bridge::ActorBridge;
use crate::error::Result;

pub struct TrafficSignBridge {
    _actor: TrafficSign,
}

impl TrafficSignBridge {
    pub fn new(_z_session: Arc<Session>, _actor: TrafficSign) -> Result<TrafficSignBridge> {
        Ok(TrafficSignBridge { _actor })
    }
}

impl ActorBridge for TrafficSignBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}
