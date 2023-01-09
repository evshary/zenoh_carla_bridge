use crate::error::Result;
use carla::client::Sensor;

pub struct SensorBridge {
    sensor_type: String,
    actor: Sensor,
}

impl SensorBridge {
    pub fn new(actor: Sensor, sensor_type: String) -> Result<SensorBridge> {
        Ok(SensorBridge { sensor_type, actor })
    }

    pub fn step(&mut self) -> Result<()> {
        Ok(())
    }
}
