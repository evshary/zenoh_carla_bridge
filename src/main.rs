mod autoware;
mod bridge;
mod clock;
mod error;
mod types;
mod utils;

use std::{
    collections::{HashMap, HashSet},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use bridge::actor_bridge::{ActorBridge, BridgeType};
use carla::{
    client::{ActorBase, Client},
    rpc::ActorId,
};
use clap::{Parser, ValueEnum};
use clock::SimulatorClock;
use error::{BridgeError, Result};
use serde_json::json;
use zenoh::{Config, Wait};

// The default interval between ticks
const DEFAULT_CARLA_TICK_INTERVAL_MS: &str = "50";

#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Clone, PartialEq, ValueEnum)]
enum Mode {
    /// Using zenoh-bridge-dds
    DDS,
    /// Using zenoh-bridge-ros2dds
    ROS2,
    /// Using rmw_zenoh
    RmwZenoh,
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

    /// Zenoh Config
    #[clap(long, value_enum)]
    zenoh_config: Option<String>,

    /// Carla Tick interval (ms)
    #[clap(long, default_value = DEFAULT_CARLA_TICK_INTERVAL_MS)]
    pub tick: u64,

    /// The multiplier to slow down simulated time
    /// For example, if slowdown == 2, 1 simulated second = 2 real seconds
    /// Suggest to set higher if the machine is not powerful enough
    #[clap(long, default_value = "1")]
    pub slowdown: u16,
}

fn main() -> Result<()> {
    pretty_env_logger::init();

    let Opts {
        carla_address,
        carla_port,
        zenoh_listen,
        mode,
        zenoh_config,
        tick,
        slowdown,
    } = Opts::parse();

    let mode = match mode {
        Some(m) => m,
        None => Mode::ROS2,
    };

    // Flag for graceful shutdown when Ctrl-C is pressed
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Failed to set Ctrl-C handler");

    log::info!("Running Carla Autoware Zenoh bridge...");
    let mut config = match zenoh_config {
        Some(conf_file) => Config::from_file(conf_file).unwrap(),
        None => Config::default(),
    };
    config
        .insert_json5("listen/endpoints", &json!(zenoh_listen).to_string())
        .expect("Failed to set zenoh listen endpoints.");
    let z_session = Arc::new(zenoh::open(config).wait()?);

    // Carla
    let client = Client::connect(&carla_address, carla_port, None);
    let mut world = client.world();
    // Carla settings (synchronous)
    let mut carla_settings = world.settings();
    carla_settings.synchronous_mode = true; // Need to tick by ourselves
    carla_settings.fixed_delta_seconds = Some(tick as f64 * 0.001); // Interval between ticks
    world.apply_settings(&carla_settings, Duration::from_millis(1000));

    // Create bridge list
    let mut bridge_list: HashMap<ActorId, Box<dyn ActorBridge>> = HashMap::new();

    // Initialize Autoware topic map and liveliness
    autoware::setup_topics(mode.clone(), z_session.clone());

    // Create clock publisher
    let simulator_clock = SimulatorClock::new(z_session.clone(), mode.clone())
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
        thread::sleep(Duration::from_millis(tick * slowdown as u64));
    });

    let mut autoware_list: HashMap<String, autoware::Autoware> = HashMap::new();

    // === Main loop ===
    // Keep running until Ctrl-C is pressed
    while running.load(Ordering::SeqCst) {
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
                    Ok(BridgeType::Vehicle(vehicle_name)) => {
                        let aw = autoware_list
                            .entry(vehicle_name.clone())
                            .or_insert_with(|| {
                                autoware::Autoware::new(vehicle_name.clone(), mode.clone())
                            });
                        bridge::actor_bridge::create_bridge(
                            z_session.clone(),
                            actor,
                            BridgeType::Vehicle(vehicle_name),
                            aw,
                        )?
                    }
                    Ok(BridgeType::Sensor(vehicle_name, sensor_type, sensor_name)) => {
                        let aw = autoware_list
                            .entry(vehicle_name.clone())
                            .or_insert_with(|| {
                                autoware::Autoware::new(vehicle_name.clone(), mode.clone())
                            });
                        aw.add_sensors(sensor_type, sensor_name.clone());
                        bridge::actor_bridge::create_bridge(
                            z_session.clone(),
                            actor,
                            BridgeType::Sensor(vehicle_name, sensor_type, sensor_name),
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
                        log::error!("Unexpected error: {err:?}");
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

    // Clean up all declared liveliness tokens before exit
    autoware::undeclare_all_liveliness();
    Ok(())
}
