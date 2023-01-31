use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::client::Actor;
use std::sync::Arc;
use zenoh::prelude::sync::*;

pub struct OtherActorBridge {
    _actor: Actor,
}

impl OtherActorBridge {
    pub fn new(_z_session: Arc<Session>, _actor: Actor) -> Result<OtherActorBridge> {
        Ok(OtherActorBridge { _actor })
    }
}

impl ActorBridge for OtherActorBridge {
    fn step(&mut self, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
