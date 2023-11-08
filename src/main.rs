mod autoware;
mod bridge;
mod clock;
mod error;
mod types;
mod utils;

use bridge::actor_bridge::{ActorBridge, BridgeType};
use carla::{client::Client, prelude::*, rpc::ActorId};
use clap::{Parser, ValueEnum};
use clock::SimulatorClock;
use error::{BridgeError, Result};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
    thread,
    time::Duration,
};
use zenoh::prelude::sync::*;

// The interval between ticks
// TODO: If we set lower value, two vehicles don't work well.
const CARLA_TICK_INTERVAL_MS: u64 = 50;

#[derive(Debug, Clone, PartialEq, ValueEnum)]
enum Mode {
    /// Using zenoh-bridge-dds
    DDS,
    /// Using zenoh-bridge-ros2dds
    ROS2,
}

/// Zenoh Carla bridge for Autoware
#[derive(Debug, Clone, Parser)]
#[clap(version, about)]
struct Opts {
    /// Carla simulator address.
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_address: String,

    /// Carla simulator port.
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,

    /// Zenoh listen address.
    #[clap(long, default_value = "tcp/localhost:7447")]
    pub zenoh_listen: Vec<String>,

    /// Select which kind of bridge you're using: zenoh-bridge-dds or zenoh-bridge-ros2dds.
    #[clap(short, long, value_enum)]
    mode: Option<Mode>,
}

fn main() -> Result<()> {
    pretty_env_logger::init();

    let Opts {
        carla_address,
        carla_port,
        zenoh_listen,
        mode,
    } = Opts::parse();

    let mode = match mode {
        Some(m) => m,
        None => Mode::DDS,
    };

    log::info!("Running Carla Autoware Zenoh bridge...");
    let mut config = Config::default();
    config
        .listen
        .endpoints
        .extend(zenoh_listen.iter().map(|p| p.parse().unwrap()));
    let z_session = Arc::new(zenoh::open(config).res()?);

    // Carla
    let client = Client::connect(&carla_address, carla_port, None);
    let mut world = client.world();
    // Carla settings (synchronous)
    let mut carla_settings = world.settings();
    carla_settings.synchronous_mode = true; // Need to tick by ourselves
    carla_settings.fixed_delta_seconds = Some(CARLA_TICK_INTERVAL_MS as f64 * 0.001); // Interval between ticks
    world.apply_settings(&carla_settings, Duration::from_millis(1000));

    // Create bridge list
    let mut bridge_list: HashMap<ActorId, Box<dyn ActorBridge>> = HashMap::new();

    // Create clock publisher
    let simulator_clock = SimulatorClock::new(z_session.clone(), mode == Mode::ROS2)
        .expect("Unable to create simulator clock!");

    // Create thread for ticking
    let client_for_tick = Client::connect(&carla_address, carla_port, None);
    thread::spawn(move || loop {
        let mut world = client_for_tick.world();
        world.tick();
        let sec = world.snapshot().timestamp().elapsed_seconds;
        simulator_clock
            .publish_clock(Some(sec))
            .expect("Unable to publish clock");
        // Tick every 2 CARLA_TICK_INTERVAL_MS
        // => 1 simulated second = 2 real seconds
        thread::sleep(Duration::from_millis(CARLA_TICK_INTERVAL_MS * 2));
    });

    let mut autoware_list: HashMap<String, autoware::Autoware> = HashMap::new();

    loop {
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
                let actor = actor_list.remove(&id).expect("ID should be in the list!");
                let bridge = match bridge::actor_bridge::get_bridge_type(actor.clone()) {
                    Ok(BridgeType::BridgeTypeVehicle(vehicle_name)) => {
                        let aw = autoware_list.entry(vehicle_name.clone()).or_insert(
                            autoware::Autoware::new(vehicle_name.clone(), mode == Mode::ROS2),
                        );
                        bridge::actor_bridge::create_bridge(
                            z_session.clone(),
                            actor,
                            BridgeType::BridgeTypeVehicle(vehicle_name),
                            aw,
                        )?
                    }
                    Ok(BridgeType::BridgeTypeSensor(vehicle_name, sensor_type, sensor_name)) => {
                        let aw = autoware_list.entry(vehicle_name.clone()).or_insert(
                            autoware::Autoware::new(vehicle_name.clone(), mode == Mode::ROS2),
                        );
                        aw.add_sensors(sensor_type, sensor_name.clone());
                        bridge::actor_bridge::create_bridge(
                            z_session.clone(),
                            actor,
                            BridgeType::BridgeTypeSensor(vehicle_name, sensor_type, sensor_name),
                            aw,
                        )?
                    }
                    Ok(_) => {
                        log::debug!("Ignore type which are not vehicle and sensor.");
                        continue;
                    }
                    Err(BridgeError::OwnerlessSensor { sensor_id }) => {
                        log::debug!(
                            "Ignore the sensor with ID {sensor_id} is not attached to any vehicle."
                        );
                        continue;
                    }
                    Err(BridgeError::Npc { npc_role_name }) => {
                        log::debug!("Ignore NPC vehicle {npc_role_name}.");
                        continue;
                    }
                    Err(err) => {
                        log::error!("Unexpected error: {:?}", err);
                        return Err(err);
                    }
                };
                bridge_list.insert(id, bridge);
                log::info!("Actor {id} created");
            }

            let mut is_actor_removed = false;
            for id in deleted_ids {
                bridge_list.remove(&id).expect("ID should be in the list!");
                log::info!("Actor {id} deleted");
                is_actor_removed = true;
            }
            // If there is actors removed, reget all the actor's list. This can avoid getting non-existed vehicles.
            if is_actor_removed {
                continue;
            }
        }

        let sec = world.snapshot().timestamp().elapsed_seconds;
        bridge_list
            .values_mut()
            .try_for_each(|bridge| bridge.step(sec))?;

        world.wait_for_tick();
    }
}
