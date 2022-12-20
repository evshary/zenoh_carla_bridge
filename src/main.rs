use carla::{client::Client, prelude::*, client::ActorKind};

fn main() {
    println!("Running Carla Autoware Zenoh bridge...");
    let client = Client::connect("127.0.0.1", 2000, None);
    for actor in client.world().actors().iter() {
        match actor.into_kinds() {
            ActorKind::Vehicle(actor) => {
                let role_name = actor.attributes()
                                             .iter()
                                             .find(|attr| attr.id() == "role_name")
                                             .unwrap()
                                             .value_string();
                println!("Detect vehicles {}", role_name);
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
}
