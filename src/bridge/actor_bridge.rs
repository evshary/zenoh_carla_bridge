use super::{
    other_bridge::OtherActorBridge, sensor_bridge::SensorBridge,
    trafficlight_bridge::TrafficLightBridge, trafficsign_bridge::TrafficSignBridge,
    vehicle_bridge::VehicleBridge,
};
use crate::error::Result;
use carla::client::{Actor, ActorKind};
use r2r::builtin_interfaces::msg::Time;
use std::sync::Arc;
use zenoh::prelude::sync::*;

pub trait ActorBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()>;
}

// z_session should outlive Box<>
pub fn create_bridge(z_session: Arc<Session>, actor: Actor) -> Box<dyn ActorBridge> {
    match actor.into_kinds() {
        ActorKind::Vehicle(vehicle) => Box::new(VehicleBridge::new(z_session, vehicle).unwrap()),
        ActorKind::Sensor(sensor) => Box::new(SensorBridge::new(z_session, sensor).unwrap()),
        ActorKind::TrafficLight(traffic_light) => {
            Box::new(TrafficLightBridge::new(z_session, traffic_light).unwrap())
        }
        ActorKind::TrafficSign(traffic_sign) => {
            Box::new(TrafficSignBridge::new(z_session, traffic_sign).unwrap())
        }
        ActorKind::Other(other) => Box::new(OtherActorBridge::new(z_session, other).unwrap()),
    }
}
