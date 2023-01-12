use super::actor_bridge::ActorBridge;
use crate::error::Result;
use crate::types::PointFieldType;
use crate::utils;
use carla::{
    client::Sensor,
    geom::Location,
    prelude::*,
    sensor::data::{LidarDetection, LidarMeasurement},
};
use cdr::{CdrLe, Infinite};
use log::info;
use r2r::{
    builtin_interfaces::msg::Time,
    sensor_msgs::msg::{PointCloud2, PointField},
    std_msgs::msg::Header,
};
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
                let pcd_publisher = z_session
                    .declare_publisher(format!(
                        "{vehicle_name}/rt/sensing/lidar/top/pointcloud_raw"
                    ))
                    .res()
                    .unwrap();
                actor.listen(move |data| {
                    let header = utils::create_ros_header().unwrap();
                    lidar_callback(header, data.try_into().unwrap(), &pcd_publisher);
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

fn lidar_callback(header: Header, measure: LidarMeasurement, pcd_publisher: &Publisher) {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return;
    }
    let point_step = std::mem::size_of_val(&lidar_data[0]) as u32;
    let row_step = lidar_data.len() as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(
            |&LidarDetection {
                 point: Location { x, y, z },
                 intensity,
             }| { [x, y, z, intensity] },
        )
        .flat_map(|elem| elem.to_ne_bytes())
        .collect();
    let fields = vec![
        PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: "intensity".to_string(),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
    ];
    let lidar_msg = PointCloud2 {
        header,
        height: 1,
        width: lidar_data.len() as u32,
        fields,
        is_bigendian: utils::is_bigendian(),
        point_step,
        row_step,
        data,
        is_dense: true,
    };
    let encoded = cdr::serialize::<_, _, CdrLe>(&lidar_msg, Infinite).unwrap();
    pcd_publisher.put(encoded).res().unwrap();
}
