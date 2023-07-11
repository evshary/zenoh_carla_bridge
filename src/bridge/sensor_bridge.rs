use super::actor_bridge::ActorBridge;
use crate::{
    error::{Error, Result},
    types::{GnssService, GnssStatus, PointFieldType},
    utils,
};
use carla::{
    client::Sensor,
    geom::Location,
    prelude::*,
    sensor::{
        data::{
            Color, GnssMeasurement, Image as CarlaImage, ImuMeasurement, LidarDetection,
            LidarMeasurement, SemanticLidarDetection, SemanticLidarMeasurement,
        },
        SensorDataBase,
    },
};
use cdr::{CdrLe, Infinite};
use log::{error, info, warn};
use nalgebra::{coordinates::XYZ, UnitQuaternion};
use r2r::{
    //geometry_msgs::msg::{Quaternion, Vector3},
    sensor_msgs::msg::{
        CameraInfo, Image as RosImage, /*Imu,*/ NavSatFix, NavSatStatus, PointCloud2,
        PointField, RegionOfInterest,
    },
    std_msgs::msg::Header,
};
use std::{
    convert::Infallible,
    mem,
    str::FromStr,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread,
};
use zenoh::prelude::sync::*;

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

enum MessageType {
    StopThread,
    SensorData,
    InfoData,
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
    sensor_type: SensorType,
    _actor: Sensor,
    sensor_name: String,
    tx: Sender<(MessageType, Vec<u8>)>,
}

impl SensorBridge {
    pub fn new(z_session: Arc<Session>, actor: Sensor) -> Result<SensorBridge> {
        let sensor_id = actor.id();
        let sensor_type_id = actor.type_id();

        let mut vehicle_name = actor
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

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(Error::Npc {
                npc_role_name: vehicle_name,
            });
        } else {
            vehicle_name = vehicle_name.replace("autoware_", "");
        }

        info!("Detected a sensor '{sensor_name}' on '{vehicle_name}'");
        let sensor_type: SensorType = sensor_type_id.parse().unwrap();
        let (tx, rx) = mpsc::channel();

        match sensor_type {
            SensorType::CameraRgb => {
                register_camera_rgb(
                    z_session,
                    &actor,
                    &vehicle_name,
                    &sensor_name,
                    tx.clone(),
                    rx,
                )?;
            }
            SensorType::LidarRayCast => {
                register_lidar_raycast(
                    z_session,
                    &actor,
                    &vehicle_name,
                    &sensor_name,
                    tx.clone(),
                    rx,
                )?;
            }
            SensorType::LidarRayCastSemantic => {
                register_lidar_raycast_semantic(
                    z_session,
                    &actor,
                    &vehicle_name,
                    &sensor_name,
                    tx.clone(),
                    rx,
                )?;
            }
            SensorType::Imu => {
                register_imu(
                    z_session,
                    &actor,
                    &vehicle_name,
                    &sensor_name,
                    tx.clone(),
                    rx,
                )?;
            }
            SensorType::Gnss => {
                register_gnss(
                    z_session,
                    &actor,
                    &vehicle_name,
                    &sensor_name,
                    tx.clone(),
                    rx,
                )?;
            }
            SensorType::Collision => {
                warn!("Collision sensor is not supported yet");
            }
            SensorType::NotSupport => {
                warn!("Unsupported sensor type '{sensor_type_id}'");
            }
        }

        Ok(SensorBridge {
            sensor_type,
            _actor: actor,
            sensor_name,
            tx,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _elapsed_sec: f64, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}

fn register_camera_rgb(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let raw_key = format!("{vehicle_name}/rt/sensing/camera/{sensor_name}/image_raw");
    let info_key = format!("{vehicle_name}/rt/sensing/camera/{sensor_name}/camera_info");

    let image_publisher = z_session.declare_publisher(raw_key.clone()).res()?;
    let info_publisher = z_session.declare_publisher(info_key.clone()).res()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = image_publisher.put(sensor_data).res() {
                    error!("Failed to publish to {}", raw_key);
                }
            }
            Ok((MessageType::InfoData, info_data)) => {
                if let Err(_) = info_publisher.put(info_data).res() {
                    error!("Failed to publish to {}", info_key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                info!("Sensor actor thread for {} stop.", raw_key);
                break;
            }
        }
    });
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
        let mut header = utils::create_ros_header(Some(data.timestamp())).unwrap();
        header.frame_id = String::from("camera4/camera_link");
        camera_callback(header.clone(), data.try_into().unwrap(), &tx).unwrap();
        camera_info_callback(header, width, height, fov, &tx).unwrap();
    });

    Ok(())
}

fn register_lidar_raycast(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    _sensor_name: &str,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/carla_pointcloud");
    let pcd_publisher = z_session.declare_publisher(key.clone()).res()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = pcd_publisher.put(sensor_data).res() {
                    error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp())).unwrap();
        header.frame_id = String::from("velodyne_top");
        lidar_callback(header, data.try_into().unwrap(), &tx).unwrap();
    });

    Ok(())
}

fn register_lidar_raycast_semantic(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    _sensor_name: &str,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/carla_pointcloud");
    let pcd_publisher = z_session.declare_publisher(key.clone()).res()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = pcd_publisher.put(sensor_data).res() {
                    error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp())).unwrap();
        header.frame_id = String::from("velodyne_top");
        senmatic_lidar_callback(header, data.try_into().unwrap(), &tx).unwrap();
    });

    Ok(())
}

fn register_imu(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/imu/{sensor_name}/imu_raw");
    let imu_publisher = z_session.declare_publisher(key.clone()).res()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = imu_publisher.put(sensor_data).res() {
                    error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp())).unwrap();
        header.frame_id = String::from("tamagawa/imu_link");
        imu_callback(header, data.try_into().unwrap(), &tx).unwrap();
    });
    Ok(())
}

fn register_gnss(
    z_session: Arc<Session>,
    actor: &Sensor,
    vehicle_name: &str,
    sensor_name: &str,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key = format!("{vehicle_name}/rt/sensing/gnss/{sensor_name}/nav_sat_fix");
    let gnss_publisher = z_session.declare_publisher(key.clone()).res()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = gnss_publisher.put(sensor_data).res() {
                    error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp())).unwrap();
        header.frame_id = String::from("gnss_link");
        gnss_callback(header, data.try_into().unwrap(), &tx).unwrap();
    });
    Ok(())
}

fn camera_callback(
    header: Header,
    image: CarlaImage,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
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
    if let Err(_) = tx.send((MessageType::SensorData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

fn camera_info_callback(
    header: Header,
    width: u32,
    height: u32,
    fov: f64,
    tx: &Sender<(MessageType, Vec<u8>)>,
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
    if let Err(_) = tx.send((MessageType::InfoData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

fn lidar_callback(
    header: Header,
    measure: LidarMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return Ok(());
    }
    let point_step = mem::size_of_val(&lidar_data[0]) as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(|det| {
            let LidarDetection {
                point: Location { x, y, z },
                intensity,
            } = *det;
            [y, x, z, intensity]
        })
        .flat_map(|elem| elem.to_ne_bytes())
        .collect();
    let row_step = data.len() as u32;
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
    if let Err(_) = tx.send((MessageType::SensorData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

fn senmatic_lidar_callback(
    header: Header,
    measure: SemanticLidarMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
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
    if let Err(_) = tx.send((MessageType::SensorData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

/* TODO: Temporarily solution, since r2r generates wrong IMU message type */
use serde_derive::{Deserialize, Serialize};
#[derive(Serialize, Deserialize, PartialEq)]
struct IMU {
    header: Header,
    orientation: [f64; 4],
    orientation_covariance: [f64; 9],
    angular_velocity: [f64; 3],
    angular_velocity_covariance: [f64; 9],
    linear_acceleration: [f64; 3],
    linear_acceleration_covariance: [f64; 9],
}

fn imu_callback(
    header: Header,
    measure: ImuMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let accel = measure.accelerometer();
    let gyro = measure.gyroscope();
    let compass = measure.compass().to_radians();
    let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, -compass);

    /*
    TODO: We generates IMU message type by ourselves, since r2r views array as Vec.
          Vec includes some header, so we can't send it directly.
    */
    let imu_msg = IMU {
        header,
        orientation: [
            /*x:*/ orientation.coords.data.0[0][3] as f64,
            /*y:*/ orientation.coords.data.0[0][1] as f64,
            /*z:*/ orientation.coords.data.0[0][0] as f64,
            /*w:*/ orientation.coords.data.0[0][2] as f64,
        ],
        orientation_covariance: [0.0; 9],
        angular_velocity: [
            /*x:*/ -gyro[0] as f64,
            /*y:*/ gyro[1] as f64,
            /*z:*/ -gyro[2] as f64,
        ],
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: [
            /*x:*/ accel[0] as f64,
            /*y:*/ -accel[1] as f64,
            /*z:*/ accel[2] as f64,
        ],
        linear_acceleration_covariance: [0.0; 9],
    };

    /* Original IMU message
    let imu_msg = Imu {
        header,
        orientation: Quaternion {
            x: orientation.coords.data.0[0][3] as f64,
            y: orientation.coords.data.0[0][1] as f64,
            z: orientation.coords.data.0[0][0] as f64,
            w: orientation.coords.data.0[0][2] as f64,
        },
        orientation_covariance: [0.0; 9].to_vec(),
        angular_velocity: Vector3 {
            x: -gyro[0] as f64,
            y: gyro[1] as f64,
            z: -gyro[2] as f64,
        },
        angular_velocity_covariance: [0.0; 9].to_vec(),
        linear_acceleration: Vector3 {
            x: accel[0] as f64,
            y: -accel[1] as f64,
            z: accel[2] as f64,
        },
        linear_acceleration_covariance: [0.0; 9].to_vec(),
    };
    */

    let encoded = cdr::serialize::<_, _, CdrLe>(&imu_msg, Infinite)?;
    if let Err(_) = tx.send((MessageType::SensorData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

fn gnss_callback(
    header: Header,
    measure: GnssMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
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
    if let Err(_) = tx.send((MessageType::SensorData, encoded)) {
        error!("Failed to send message");
    }
    Ok(())
}

fn generate_sensor_name(actor: &Sensor) -> String {
    let XYZ { x, y, z } = *actor.location();
    format!("{x}_{y}_{z}")
}

impl Drop for SensorBridge {
    fn drop(&mut self) {
        info!("Remove sensor name {}", self.sensor_name);
        if self.sensor_type != SensorType::Collision && self.sensor_type != SensorType::NotSupport {
            // Not sure why the tx doesn't release in sensor callback, so rx can't use RecvErr to close the thread
            // I create another message type to notify the thread to close
            if let Err(_) = self.tx.send((MessageType::StopThread, vec![])) {
                error!("Unable to stop the thread")
            }
        }
    }
}
