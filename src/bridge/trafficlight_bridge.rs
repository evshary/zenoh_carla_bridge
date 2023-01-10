use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{
    client::{ActorKind, Client, TrafficLight},
    prelude::*,
    rpc::ActorId,
};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub struct TrafficLightBridge {
    actor: TrafficLight,
}

impl TrafficLightBridge {
    pub fn new(z_session: &Session, actor: TrafficLight) -> Result<TrafficLightBridge> {
        Ok(TrafficLightBridge { actor })
    }
}

impl ActorBridge for TrafficLightBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
