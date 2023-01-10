use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{
    client::{ActorKind, Client, Sensor},
    prelude::*,
    rpc::ActorId,
};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub struct SensorBridge {
    sensor_type: String,
    actor: Sensor,
}

impl SensorBridge {
    pub fn new(z_session: &Session, actor: Sensor) -> Result<SensorBridge> {
        let sensor_type = actor.type_id();
        info!("Detect sensors {sensor_type}");
        Ok(SensorBridge { sensor_type, actor })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}
