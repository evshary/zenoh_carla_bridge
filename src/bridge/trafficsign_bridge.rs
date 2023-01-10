use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{
    client::{ActorKind, Client, TrafficSign},
    prelude::*,
    rpc::ActorId,
};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub struct TrafficSignBridge {
    actor: TrafficSign,
}

impl TrafficSignBridge {
    pub fn new(z_session: &Session, actor: TrafficSign) -> Result<TrafficSignBridge> {
        Ok(TrafficSignBridge { actor })
    }
}

impl ActorBridge for TrafficSignBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
