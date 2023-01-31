use super::{
    other_bridge::OtherActorBridge, sensor_bridge::SensorBridge,
    trafficlight_bridge::TrafficLightBridge, trafficsign_bridge::TrafficSignBridge,
    vehicle_bridge::VehicleBridge,
};
use crate::error::Result;
use carla::client::{Actor, ActorKind};
use std::sync::Arc;
use zenoh::prelude::sync::*;

pub trait ActorBridge {
    fn step(&mut self, elapsed_sec: f64) -> Result<()>;
}

// z_session should outlive Box<>
pub fn create_bridge(z_session: Arc<Session>, actor: Actor) -> Result<Box<dyn ActorBridge>> {
    Ok(match actor.into_kinds() {
        ActorKind::Vehicle(vehicle) => Box::new(VehicleBridge::new(z_session, vehicle)?),
        ActorKind::Sensor(sensor) => Box::new(SensorBridge::new(z_session, sensor)?),
        ActorKind::TrafficLight(traffic_light) => {
            Box::new(TrafficLightBridge::new(z_session, traffic_light)?)
        }
        ActorKind::TrafficSign(traffic_sign) => {
            Box::new(TrafficSignBridge::new(z_session, traffic_sign)?)
        }
        ActorKind::Other(other) => Box::new(OtherActorBridge::new(z_session, other)?),
    })
}
