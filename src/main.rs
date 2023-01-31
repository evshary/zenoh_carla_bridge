mod bridge;
mod error;
mod types;
mod utils;

use anyhow::Result;
use bridge::actor_bridge::ActorBridge;
use carla::{client::Client, prelude::*, rpc::ActorId};
use clap::Parser;
use error::Error;
use log::{error, info};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
    thread,
    time::{Duration, Instant},
};
use zenoh::prelude::sync::*;

/// Command line options
#[derive(Debug, Clone, Parser)]
struct Opts {
    /// Carla simulator address.
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_address: String,

    /// Carla simulator port.
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();

    let Opts {
        carla_address,
        carla_port,
    } = Opts::parse();

    info!("Running Carla Autoware Zenoh bridge...");
    let z_session = Arc::new(zenoh::open(Config::default()).res()?);

    let client = Client::connect(&carla_address, carla_port, None);
    let world = client.world();
    let mut bridge_list: HashMap<ActorId, Box<dyn ActorBridge>> = HashMap::new();

    let mut last_time = Instant::now();

    loop {
        let elapsed_time = last_time.elapsed().as_secs_f64();
        {
            let mut actor_list: HashMap<ActorId, _> = world
                .actors()
                .iter()
                .map(|actor| (actor.id(), actor))
                .collect();
            let prev_actor_ids: HashSet<u32> = bridge_list.keys().cloned().collect();
            let cur_actor_ids: HashSet<u32> = actor_list.keys().cloned().collect();
            let added_ids = &cur_actor_ids - &prev_actor_ids;
            let deleted_ids = &prev_actor_ids - &cur_actor_ids;

            for id in added_ids {
                let actor = actor_list.remove(&id).unwrap();
                let bridge = match bridge::actor_bridge::create_bridge(z_session.clone(), actor) {
                    Ok(bridge) => bridge,
                    Err(Error::OwnerlessSensor { sensor_id }) => {
                        error!(
                            "Ignore the sensor with ID {sensor_id} is not attached to any vehicle."
                        );
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                bridge_list.insert(id, bridge);
                info!("Actor {id} created");
            }

            for id in deleted_ids {
                bridge_list.remove(&id).unwrap();
                info!("Actor {id} deleted");
            }
        }

        bridge_list
            .values_mut()
            .try_for_each(|bridge| bridge.step(elapsed_time))?;
        last_time = Instant::now();
        world.wait_for_tick();

        // Sleep here, since the elapsed_time should be larger than certain value or carla_ackermann will have wrong result.
        thread::sleep(Duration::from_millis(100));
    }
}
