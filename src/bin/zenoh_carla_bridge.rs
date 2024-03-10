use clap::Parser;
use zenoh_carla_bridge::{error::Result, Opts};

fn main() -> Result<()> {
    let opts = Opts::parse();
    zenoh_carla_bridge::run(opts)?;
    Ok(())
}
