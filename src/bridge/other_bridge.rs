use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{
    client::{Actor, ActorKind, Client},
    prelude::*,
    rpc::ActorId,
};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub struct OtherActorBridge {
    actor: Actor,
}

impl OtherActorBridge {
    pub fn new(z_session: &Session, actor: Actor) -> Result<OtherActorBridge> {
        Ok(OtherActorBridge { actor })
    }
}

impl ActorBridge for OtherActorBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
