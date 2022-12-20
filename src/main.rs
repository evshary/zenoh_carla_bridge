mod autoware_type;
mod vehicle_bridge;

use std::sync::Arc;
use carla::{client::Client, prelude::*, client::ActorKind};
use zenoh::prelude::sync::*;
use std::thread;
use std::time::Duration;

fn main() {
    println!("Running Carla Autoware Zenoh bridge...");
    let z_session = Arc::new(zenoh::open(Config::default()).res().unwrap());

    let client = Client::connect("127.0.0.1", 2000, None);
    let mut vehicle_bridge_list = Vec::new();
    for actor in client.world().actors().iter() {
        match actor.into_kinds() {
            ActorKind::Vehicle(actor) => {
                let role_name = actor.attributes()
                                             .iter()
                                             .find(|attr| attr.id() == "role_name")
                                             .unwrap()
                                             .value_string();
                println!("Detect vehicles {}", role_name);
                let v_bridge = vehicle_bridge::VehicleBridge::new(z_session.clone(), role_name, actor);
                vehicle_bridge_list.push(v_bridge);
            },
            ActorKind::Sensor(_) => {
                println!("Detect sensors");
            },
            ActorKind::TrafficLight(_) => {
                println!("Detect traffic light");
            },
            ActorKind::TrafficSign(_) => {
                println!("Detect traffic sign");
            },
            ActorKind::Other(_) => {
                println!("Detect others");
            },
        }
    }
    loop {
        thread::sleep(Duration::from_millis(1000));
    }
}
