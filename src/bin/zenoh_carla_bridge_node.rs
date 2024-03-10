use anyhow::{bail, Result};
use r2r::{Context, Node, Parameter, ParameterValue};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};
use zenoh_carla_bridge::{Mode, Opts};

fn main() -> Result<()> {
    // Create a ROS node
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, env!("CARGO_BIN_NAME"), "")?;

    // Parse parameters provided to Node
    let opts = parse_params(&node)?;

    // Spin the node in a new thread
    let is_running = Arc::new(AtomicBool::new(true));
    let node_spinner = {
        let is_running = is_running.clone();
        thread::spawn(move || {
            while is_running.load(Ordering::Acquire) {
                node.spin_once(Duration::from_millis(100));
            }
        })
    };

    // Run the bridge
    zenoh_carla_bridge::run(opts)?;

    // Once the bridge is turned off, stop spinning the node.
    is_running.store(false, Ordering::Release);
    node_spinner.join().unwrap();

    Ok(())
}

fn parse_params(node: &Node) -> Result<Opts> {
    let params = node.params.lock().unwrap();

    macro_rules! bail_parse_error {
        ($name:expr) => {{
            bail!("Unable to parse \"{}\" parameter", $name);
        }};
    }

    let get_string = |name: &str| -> Result<Option<String>> {
        match params.get(name) {
            Some(Parameter {
                value: ParameterValue::String(value),
                ..
            }) => Ok(Some(value.to_string())),
            Some(_) => bail_parse_error!(name),
            None => Ok(None),
        }
    };
    let get_string_or_default = |name: &str, default: &str| -> Result<String> {
        match params.get(name) {
            Some(Parameter {
                value: ParameterValue::String(value),
                ..
            }) => Ok(value.to_string()),
            Some(_) => bail_parse_error!(name),
            None => Ok(default.to_string()),
        }
    };
    let get_i64_or_default = |name: &str, default: i64| -> Result<i64> {
        match params.get(name) {
            Some(Parameter {
                value: ParameterValue::Integer(value),
                ..
            }) => Ok(*value),
            Some(_) => bail_parse_error!(name),
            None => Ok(default),
        }
    };
    let get_string_array_or_default = |name: &str, default| -> Result<Vec<String>> {
        match params.get(name) {
            Some(Parameter {
                value: ParameterValue::StringArray(value),
                ..
            }) => Ok(value.to_owned()),
            Some(_) => bail_parse_error!(name),
            None => Ok(default),
        }
    };

    let carla_address = get_string_or_default("carla_address", "127.0.0.1")?;
    let carla_port = {
        let port = get_i64_or_default("carla_port", 2000)?;
        let Ok(port) = u16::try_from(port) else {
            bail!("Invalid port number {port}.");
        };
        port
    };
    let zenoh_listen =
        get_string_array_or_default("zenoh_listen", vec!["tcp/localhost:7447".to_string()])?;
    let mode = match get_string("zenoh_config")? {
        Some(value) => Some(match value.as_str() {
            "dds" => Mode::DDS,
            "ros2" => Mode::ROS2,
            _ => bail!(
                "Invalid mode parameter \"{value}\". It should be one of \"dds\" or \"ros2\"."
            ),
        }),
        None => None,
    };
    let zenoh_config = get_string("zenoh_config")?;

    Ok(Opts {
        carla_address,
        carla_port,
        zenoh_listen,
        mode,
        zenoh_config,
    })
}
