use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::client::TrafficSign;
use r2r::builtin_interfaces::msg::Time;
use zenoh::prelude::sync::*;

pub struct TrafficSignBridge {
    _actor: TrafficSign,
}

impl TrafficSignBridge {
    pub fn new(_z_session: &Session, _actor: TrafficSign) -> Result<TrafficSignBridge> {
        Ok(TrafficSignBridge { _actor })
    }
}

impl ActorBridge for TrafficSignBridge {
    fn step(&mut self, _stamp: &Time, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
