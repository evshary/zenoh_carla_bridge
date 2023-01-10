use super::other_bridge::OtherActorBridge;
use super::sensor_bridge::SensorBridge;
use super::trafficlight_bridge::TrafficLightBridge;
use super::trafficsign_bridge::TrafficSignBridge;
use super::vehicle_bridge::VehicleBridge;
use crate::error::Result;
use carla::{
    client::{Actor, ActorKind, Client, Sensor},
    prelude::*,
    rpc::ActorId,
};
use r2r::builtin_interfaces::msg::Time;
use zenoh::{
    buffers::reader::HasReader, prelude::sync::*, publication::Publisher, subscriber::Subscriber,
};

pub trait ActorBridge {
    fn step(&mut self, stamp: &Time, elapsed_sec: f64) -> Result<()>;
}

// z_session should outlive Box<>
pub fn create_bridge<'a: 'b, 'b>(
    z_session: &'a Session,
    actor: Actor,
) -> Box<dyn ActorBridge + 'b> {
    match actor.into_kinds() {
        ActorKind::Vehicle(vehicle) => Box::new(VehicleBridge::new(z_session, vehicle).unwrap()),
        ActorKind::Sensor(sensor) => Box::new(SensorBridge::new(&z_session, sensor).unwrap()),
        ActorKind::TrafficLight(traffic_light) => {
            Box::new(TrafficLightBridge::new(&z_session, traffic_light).unwrap())
        }
        ActorKind::TrafficSign(traffic_sign) => {
            Box::new(TrafficSignBridge::new(&z_session, traffic_sign).unwrap())
        }
        ActorKind::Other(other) => Box::new(OtherActorBridge::new(&z_session, other).unwrap()),
    }
    //Box::new(VehicleBridge::new(z_session, actor.into_kinds().try_into_vehicle().unwrap()).unwrap())
}
