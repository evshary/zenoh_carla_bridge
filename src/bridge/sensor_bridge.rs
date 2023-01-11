use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{client::Sensor, prelude::*, sensor::data::LidarMeasurement};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use zenoh::prelude::sync::*;

pub enum SensorType {
    CameraRgb,
    LidarRayCast,
    LidarRayCastSemantic,
    Imu,
    Collision,
    NotSupport,
}

pub struct SensorBridge {
    _vehicle_name: String,
    _sensor_type: SensorType,
    _actor: Sensor,
}

impl SensorBridge {
    pub fn new(_z_session: &Session, actor: Sensor) -> Result<SensorBridge> {
        let vehicle_name = actor
            .parent()
            .unwrap()
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .unwrap()
            .value_string();
        let sensor_type_id = actor.type_id();
        info!("Detect sensors {sensor_type_id} from {vehicle_name}");
        let sensor_type = match sensor_type_id.as_str() {
            "sensor.camera.rgb" => {
                actor.listen(|data| {
                    lidar_callback(data.try_into().unwrap());
                });
                SensorType::CameraRgb
            }
            "sensor.lidar.ray_cast" => SensorType::LidarRayCast,
            "sensor.lidar.ray_cast_semantic" => SensorType::LidarRayCastSemantic,
            "sensor.other.imu" => SensorType::Imu,
            "sensor.other.collision" => SensorType::Collision,
            _ => SensorType::NotSupport,
        };
        Ok(SensorBridge {
            _vehicle_name: vehicle_name,
            _sensor_type: sensor_type,
            _actor: actor,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _stamp: &Time, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}

fn lidar_callback(_measure: LidarMeasurement) {}
