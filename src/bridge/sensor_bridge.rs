use super::actor_bridge::ActorBridge;
use crate::error::Result;
use carla::{client::Sensor, prelude::*, sensor::data::LidarMeasurement};
use log::info;
use r2r::builtin_interfaces::msg::Time;
use std::sync::Arc;
use zenoh::{prelude::sync::*, publication::Publisher};

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
    pub fn new<'a>(z_session: Arc<Session>, actor: Sensor) -> Result<SensorBridge> {
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
            "sensor.camera.rgb" => SensorType::CameraRgb,
            "sensor.lidar.ray_cast" => {
                let z_publisher = z_session
                    .declare_publisher(format!(
                        "{vehicle_name}/rt/sensing/lidar/top/pointcloud_raw"
                    ))
                    .res()
                    .unwrap();
                actor.listen(move |data| {
                    lidar_callback(data.try_into().unwrap(), &z_publisher);
                    // This line fails
                    //z_publisher.put("Error").res().unwrap();
                });
                SensorType::LidarRayCast
            }
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

fn lidar_callback(_measure: LidarMeasurement, _publisher: &Publisher) {}
