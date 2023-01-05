mod vehicle_bridge;

use carla::{
    client::{ActorKind, Client},
    prelude::*,
    rpc::ActorId,
};
use clap::Parser;
use log::info;
use r2r::{Clock, ClockType};
use std::{
    collections::{HashMap, HashSet},
    thread,
    time::{Duration, Instant},
};
use vehicle_bridge::VehicleBridge;
use zenoh::prelude::sync::*;

/// Command line options
#[derive(Debug, Clone, Parser)]
struct Opts {
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_address: String,
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,
}

fn main() {
    pretty_env_logger::init();

    let Opts {
        carla_address,
        carla_port,
    } = Opts::parse();

    info!("Running Carla Autoware Zenoh bridge...");
    let z_session = zenoh::open(Config::default()).res().unwrap();

    let client = Client::connect(&carla_address, carla_port, None);
    let world = client.world();
    let mut bridge_list: HashMap<ActorId, VehicleBridge> = HashMap::new();

    let mut last_time = Instant::now();
    let mut clock = Clock::create(ClockType::RosTime).unwrap();

    loop {
        let elapsed_time = last_time.elapsed().as_secs_f64();
        let time = Clock::to_builtin_time(&clock.get_now().unwrap());

        {
            let mut actor_list: HashMap<ActorId, _> = world
                .actors()
                .iter()
                .map(|actor| (actor.id(), actor))
                .collect();
            let prev_actor_ids = bridge_list.keys().cloned().collect::<HashSet<_>>();
            let cur_actor_ids = actor_list.keys().cloned().collect::<HashSet<_>>();
            let added_ids = &cur_actor_ids - &prev_actor_ids;
            let deleted_ids = &prev_actor_ids - &cur_actor_ids;

            for id in added_ids {
                match actor_list.remove(&id).unwrap().into_kinds() {
                    ActorKind::Vehicle(actor) => {
                        let role_name = actor
                            .attributes()
                            .iter()
                            .find(|attr| attr.id() == "role_name")
                            .unwrap()
                            .value_string();
                        info!("Detect vehicles {role_name}");
                        let v_bridge = VehicleBridge::new(&z_session, role_name, actor);
                        bridge_list.insert(id, v_bridge);
                    }
                    ActorKind::Sensor(_) => {
                        //info!("Detect sensors");
                    }
                    ActorKind::TrafficLight(_) => {
                        //info!("Detect traffic light");
                    }
                    ActorKind::TrafficSign(_) => {
                        //info!("Detect traffic sign");
                    }
                    ActorKind::Other(_) => {
                        //info!("Detect others");
                    }
                }
            }

            for id in deleted_ids {
                let removed_vehicle = bridge_list.remove(&id).unwrap();
                info!("Remove vehicle {}", removed_vehicle.vehicle_name());
            }
        }

        bridge_list
            .values_mut()
            .for_each(|bridge| bridge.step(&time, elapsed_time));
        last_time = Instant::now();
        world.wait_for_tick();

        // Sleep here, since the elapsed_time should be larger than certain value or carla_ackermann will have wrong result.
        thread::sleep(Duration::from_millis(100));
    }
}
