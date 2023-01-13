use super::actor_bridge::ActorBridge;
use crate::error::Result;
use crate::types::PointFieldType;
use crate::utils;
use carla::{
    client::Sensor,
    geom::Location,
    prelude::*,
    sensor::data::{
        Color, Image as CarlaImage, LidarDetection, LidarMeasurement, SemanticLidarDetection,
        SemanticLidarMeasurement,
    },
};
use cdr::{CdrLe, Infinite};
use log::info;
use r2r::{
    builtin_interfaces::msg::Time,
    sensor_msgs::msg::{Image as RosImage, PointCloud2, PointField},
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
    GodView,
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
        let role_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .unwrap()
            .value_string();

        let sensor_type = if role_name == "godview" {
            SensorType::GodView
        } else {
            match sensor_type_id.as_str() {
                "sensor.camera.rgb" => {
                    let image_publisher = z_session
                        .declare_publisher(format!(
                            "{vehicle_name}/rt/sensing/camera/traffic_light/image_raw"
                        ))
                        .res()
                        .unwrap();
                    actor.listen(move |data| {
                        let header = utils::create_ros_header().unwrap();
                        camera_callback(header, data.try_into().unwrap(), &image_publisher);
                    });
                    SensorType::CameraRgb
                }
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
                "sensor.lidar.ray_cast_semantic" => {
                    let pcd_publisher = z_session
                        .declare_publisher(format!(
                            "{vehicle_name}/rt/sensing/lidar/top/pointcloud_raw"
                        ))
                        .res()
                        .unwrap();
                    actor.listen(move |data| {
                        let header = utils::create_ros_header().unwrap();
                        senmatic_lidar_callback(header, data.try_into().unwrap(), &pcd_publisher);
                    });
                    SensorType::LidarRayCastSemantic
                }
                "sensor.other.imu" => SensorType::Imu,
                "sensor.other.collision" => SensorType::Collision,
                _ => SensorType::NotSupport,
            }
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

fn camera_callback(header: Header, image: CarlaImage, image_publisher: &Publisher) {
    let image_data = image.as_slice();
    if image_data.is_empty() {
        return;
    }
    let width = image.width();
    let height = image.height();
    let data: Vec<_> = image_data
        .iter()
        .flat_map(|&Color { b, g, r, a }| [b, g, r, a])
        .collect();

    let image_msg = RosImage {
        header,
        height: height as u32,
        width: width as u32,
        encoding: "bgra8".to_string(),
        is_bigendian: utils::is_bigendian().into(),
        step: (width * 4) as u32,
        data,
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&image_msg, Infinite).unwrap();
    image_publisher.put(encoded).res().unwrap();
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

fn senmatic_lidar_callback(
    header: Header,
    measure: SemanticLidarMeasurement,
    pcd_publisher: &Publisher,
) {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return;
    }
    let point_step = std::mem::size_of_val(&lidar_data[0]) as u32;
    let row_step = lidar_data.len() as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(
            |&SemanticLidarDetection {
                 point: Location { x, y, z },
                 cos_inc_angle,
                 object_idx,
                 object_tag,
             }| {
                [
                    x.to_ne_bytes(),
                    y.to_ne_bytes(),
                    z.to_ne_bytes(),
                    cos_inc_angle.to_ne_bytes(),
                    object_idx.to_ne_bytes(),
                    object_tag.to_ne_bytes(),
                ]
            },
        )
        .flatten()
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
            name: "cos_inc_angle".to_string(),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: "object_idx".to_string(),
            offset: 16,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        PointField {
            name: "object_tag".to_string(),
            offset: 20,
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
