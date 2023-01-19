use super::actor_bridge::ActorBridge;
use crate::{
    error::{Error, Result},
    types::PointFieldType,
    utils,
};
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
use log::{info, warn};
use nalgebra::coordinates::XYZ;
use r2r::{
    builtin_interfaces::msg::Time,
    sensor_msgs::msg::{Image as RosImage, PointCloud2, PointField},
    std_msgs::msg::Header,
};
use std::{convert::Infallible, mem, str::FromStr, sync::Arc};
use zenoh::{prelude::sync::*, publication::Publisher};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorType {
    CameraRgb,
    LidarRayCast,
    LidarRayCastSemantic,
    Imu,
    Collision,
    NotSupport,
}

impl FromStr for SensorType {
    type Err = Infallible;

    fn from_str(type_id: &str) -> Result<Self, Self::Err> {
        Ok(match type_id {
            "sensor.camera.rgb" => SensorType::CameraRgb,
            "sensor.lidar.ray_cast" => SensorType::LidarRayCast,
            "sensor.lidar.ray_cast_semantic" => SensorType::LidarRayCastSemantic,
            "sensor.other.imu" => SensorType::Imu,
            "sensor.other.collision" => SensorType::Collision,
            _ => SensorType::NotSupport,
        })
    }
}

pub struct SensorBridge {
    _sensor_type: SensorType,
    _actor: Sensor,
}

impl SensorBridge {
    pub fn new(z_session: Arc<Session>, actor: Sensor) -> Result<SensorBridge> {
        let sensor_id = actor.id();
        let sensor_type_id = actor.type_id();

        let vehicle_name = actor
            .parent()
            .ok_or(Error::OwnerlessSensor { sensor_id })?
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .expect("'role_name' attribute is missing")
            .value_string();
        let sensor_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .map(|attr| attr.value_string())
            .unwrap_or_else(|| generate_sensor_name(&actor));

        info!("Detected a sensor '{sensor_name}' on '{vehicle_name}'");
        let sensor_type: SensorType = sensor_type_id.parse().unwrap();

        match sensor_type {
            SensorType::CameraRgb => {
                register_camera_rgb(z_session, &actor, &vehicle_name, &sensor_name)?;
            }
            SensorType::LidarRayCast => {
                register_lidar_raycast(z_session, &actor, &vehicle_name, &sensor_name)?;
            }
            SensorType::LidarRayCastSemantic => {
                register_lidar_raycast_semantic(z_session, &actor, &vehicle_name, &sensor_name)?;
            }
            SensorType::Imu => {
                warn!("IMU sensor is not supported yet");
            }
            SensorType::Collision => {
                warn!("Collision sensor is not supported yet");
            }
            SensorType::NotSupport => {
                warn!("Unsupported sensor type '{sensor_type_id}'");
            }
        }

        Ok(SensorBridge {
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

fn register_camera_rgb(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/camera/{sensor_name}/image_raw");

    let image_publisher = z_session.declare_publisher(key).res()?;
    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        camera_callback(header, data.try_into().unwrap(), &image_publisher).unwrap();
    });

    Ok(())
}

fn register_lidar_raycast(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/lidar/{sensor_name}/pointcloud_raw");
    let pcd_publisher = z_session.declare_publisher(key).res()?;
    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        lidar_callback(header, data.try_into().unwrap(), &pcd_publisher).unwrap();
    });

    Ok(())
}

fn register_lidar_raycast_semantic(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/lidar/{sensor_name}/pointcloud_raw");
    let pcd_publisher = z_session.declare_publisher(key).res()?;
    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        senmatic_lidar_callback(header, data.try_into().unwrap(), &pcd_publisher).unwrap();
    });

    Ok(())
}

fn camera_callback(header: Header, image: CarlaImage, image_publisher: &Publisher) -> Result<()> {
    let image_data = image.as_slice();
    if image_data.is_empty() {
        return Ok(());
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

    let encoded = cdr::serialize::<_, _, CdrLe>(&image_msg, Infinite)?;
    image_publisher.put(encoded).res()?;
    Ok(())
}

fn lidar_callback(
    header: Header,
    measure: LidarMeasurement,
    pcd_publisher: &Publisher,
) -> Result<()> {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return Ok(());
    }
    let point_step = mem::size_of_val(&lidar_data[0]) as u32;
    let row_step = lidar_data.len() as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(|det| {
            let LidarDetection {
                point: Location { x, y, z },
                intensity,
            } = *det;
            [x, y, z, intensity]
        })
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
    let encoded = cdr::serialize::<_, _, CdrLe>(&lidar_msg, Infinite)?;
    pcd_publisher.put(encoded).res()?;
    Ok(())
}

fn senmatic_lidar_callback(
    header: Header,
    measure: SemanticLidarMeasurement,
    pcd_publisher: &Publisher,
) -> Result<()> {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return Ok(());
    }
    let point_step = mem::size_of_val(&lidar_data[0]) as u32;
    let row_step = lidar_data.len() as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(|det| {
            let SemanticLidarDetection {
                point: Location { x, y, z },
                cos_inc_angle,
                object_idx,
                object_tag,
            } = *det;

            [
                x.to_ne_bytes(),
                y.to_ne_bytes(),
                z.to_ne_bytes(),
                cos_inc_angle.to_ne_bytes(),
                object_idx.to_ne_bytes(),
                object_tag.to_ne_bytes(),
            ]
        })
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
    let encoded = cdr::serialize::<_, _, CdrLe>(&lidar_msg, Infinite)?;
    pcd_publisher.put(encoded).res()?;
    Ok(())
}

fn generate_sensor_name(actor: &Sensor) -> String {
    let XYZ { x, y, z } = *actor.location();
    format!("{x}_{y}_{z}")
}
