use super::actor_bridge::ActorBridge;
use crate::{
    error::{Error, Result},
    types::{GnssService, GnssStatus, PointFieldType},
    utils::{self, ToRosType},
};
use carla::{
    client::Sensor,
    geom::Location,
    prelude::*,
    sensor::data::{
        Color, GnssMeasurement, Image as CarlaImage, ImuMeasurement, LidarDetection,
        LidarMeasurement, SemanticLidarDetection, SemanticLidarMeasurement,
    },
};
use cdr::{CdrLe, Infinite};
use log::{info, warn};
use nalgebra::{coordinates::XYZ, UnitQuaternion};
use r2r::{
    sensor_msgs::msg::{
        CameraInfo, Image as RosImage, Imu, NavSatFix, NavSatStatus, PointCloud2, PointField,
        RegionOfInterest,
    },
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
    Gnss,
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
            "sensor.other.gnss" => SensorType::Gnss,
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
                register_imu(z_session, &actor, &vehicle_name, &sensor_name)?;
            }
            SensorType::Gnss => {
                register_gnss(z_session, &actor, &vehicle_name, &sensor_name)?;
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
    fn step(&mut self, _elapsed_sec: f64) -> Result<()> {
        Ok(())
    }
}

fn register_camera_rgb(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let raw_key = format!("{vehicle_name}/rt/sensing/camera/{sensor_name}/image_raw");
    let info_key = format!("{vehicle_name}/rt/sensing/camera/{sensor_name}/camera_info");

    let image_publisher = z_session.declare_publisher(raw_key).res()?;
    let info_publisher = z_session.declare_publisher(info_key).res()?;
    let width = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_x")
        .unwrap()
        .value()
        .unwrap()
        .try_into_int()
        .unwrap() as u32;
    let height = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_y")
        .unwrap()
        .value()
        .unwrap()
        .try_into_int()
        .unwrap() as u32;
    let fov = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "fov")
        .unwrap()
        .value()
        .unwrap()
        .try_into_f32()
        .unwrap() as f64;

    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        camera_callback(header.clone(), data.try_into().unwrap(), &image_publisher).unwrap();
        camera_info_callback(header, width, height, fov, &info_publisher).unwrap();
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

fn register_imu(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/imu/{sensor_name}/imu_raw");
    let imu_publisher = z_session.declare_publisher(key).res()?;
    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        imu_callback(header, data.try_into().unwrap(), &imu_publisher).unwrap();
    });
    Ok(())
}

fn register_gnss(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/gnss/{sensor_name}/nav_sat_fix");
    let gnss_publisher = z_session.declare_publisher(key).res()?;
    actor.listen(move |data| {
        let header = utils::create_ros_header().unwrap();
        gnss_callback(header, data.try_into().unwrap(), &gnss_publisher).unwrap();
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

fn camera_info_callback(
    header: Header,
    width: u32,
    height: u32,
    fov: f64,
    info_publisher: &Publisher,
) -> Result<()> {
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;
    let fx = width as f64 / (2.0 * (fov * std::f64::consts::PI / 360.0).tan());
    let fy = fx;

    let camera_info = CameraInfo {
        header,
        width,
        height,
        distortion_model: String::from("plumb_bob"),
        k: vec![fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        d: vec![0.0, 0.0, 0.0, 0.0, 0.0],
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: vec![fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&camera_info, Infinite)?;
    info_publisher.put(encoded).res()?;
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

fn imu_callback(header: Header, measure: ImuMeasurement, imu_publisher: &Publisher) -> Result<()> {
    let accel = measure.accelerometer();
    let compass = measure.compass();
    let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, -compass);
    let gyro = measure.gyroscope();

    let imu_msg = Imu {
        header,
        orientation: orientation.to_ros_type(),
        orientation_covariance: utils::identity_matrix(3).into_raw_vec(),
        angular_velocity: gyro.to_ros_type(),
        angular_velocity_covariance: utils::identity_matrix(3).into_raw_vec(),
        linear_acceleration: accel.to_ros_type(),
        linear_acceleration_covariance: utils::identity_matrix(3).into_raw_vec(),
    };
    let encoded = cdr::serialize::<_, _, CdrLe>(&imu_msg, Infinite)?;
    imu_publisher.put(encoded).res()?;
    Ok(())
}

fn gnss_callback(
    header: Header,
    measure: GnssMeasurement,
    gnss_publisher: &Publisher,
) -> Result<()> {
    let gnss_msg = NavSatFix {
        header,
        latitude: measure.latitude(),
        longitude: measure.longitude(),
        altitude: measure.attitude() + 17.0,
        status: NavSatStatus {
            status: GnssStatus::StatusSbasFix as i8,
            service: GnssService::ServiceGps as u16
                | GnssService::ServiceGlonass as u16
                | GnssService::ServiceCompass as u16
                | GnssService::ServiceGalileo as u16,
        },
        position_covariance: vec![0.0],
        position_covariance_type: 0, // unknown type
    };
    let encoded = cdr::serialize::<_, _, CdrLe>(&gnss_msg, Infinite)?;
    gnss_publisher.put(encoded).res()?;
    Ok(())
}

fn generate_sensor_name(actor: &Sensor) -> String {
    let XYZ { x, y, z } = *actor.location();
    format!("{x}_{y}_{z}")
}
