use super::actor_bridge::{ActorBridge, BridgeType};
use crate::{
    autoware::Autoware,
    error::{BridgeError, Result},
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
use nalgebra::{coordinates::XYZ, UnitQuaternion};
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
use zenoh::{prelude::*, Session};
use zenoh_ros_type::{geometry_msgs, sensor_msgs, std_msgs};

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
    _vehicle_name: String,
    sensor_type: SensorType,
    _actor: Sensor,
    sensor_name: String,
    tx: Sender<(MessageType, Vec<u8>)>,
}

impl SensorBridge {
    pub fn get_bridge_type(actor: Sensor) -> Result<BridgeType> {
        let sensor_id = actor.id();
        let sensor_type_id = actor.type_id();

        let mut vehicle_name = actor
            .parent()
            .ok_or(BridgeError::OwnerlessSensor { sensor_id })?
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .ok_or(BridgeError::CarlaIssue("'role_name' attribute is missing"))?
            .value_string();
        let sensor_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .map(|attr| attr.value_string())
            .unwrap_or_else(|| generate_sensor_name(&actor));

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(BridgeError::Npc {
                npc_role_name: vehicle_name,
            });
        }
        vehicle_name = vehicle_name.replace("autoware_", "");

        log::info!("Detected a sensor '{sensor_name}' on '{vehicle_name}'");
        let sensor_type: SensorType = sensor_type_id.parse().or(Err(BridgeError::CarlaIssue(
            "Unable to recognize sensor type",
        )))?;
        Ok(BridgeType::BridgeTypeSensor(
            vehicle_name,
            sensor_type,
            sensor_name,
        ))
    }
    pub fn new(
        z_session: Arc<Session>,
        actor: Sensor,
        bridge_type: BridgeType,
        autoware: &Autoware,
    ) -> Result<SensorBridge> {
        let (vehicle_name, sensor_type, sensor_name) = match bridge_type {
            BridgeType::BridgeTypeSensor(v, t, s) => (v, t, s),
            _ => panic!("Should never happen!"),
        };

        let (tx, rx) = mpsc::channel();
        let key_list = autoware.get_sensors_key(sensor_type, &sensor_name);

        match sensor_type {
            SensorType::CameraRgb => {
                register_camera_rgb(z_session, &actor, key_list, tx.clone(), rx)?;
            }
            SensorType::LidarRayCast => {
                register_lidar_raycast(z_session, &actor, key_list, tx.clone(), rx)?;
            }
            SensorType::LidarRayCastSemantic => {
                register_lidar_raycast_semantic(z_session, &actor, key_list, tx.clone(), rx)?;
            }
            SensorType::Imu => {
                register_imu(z_session, &actor, key_list, tx.clone(), rx)?;
            }
            SensorType::Gnss => {
                register_gnss(z_session, &actor, key_list, tx.clone(), rx)?;
            }
            SensorType::Collision => {
                log::warn!("Collision sensor is not supported yet");
            }
            SensorType::NotSupport => {
                log::warn!("Unsupported sensor type '{}'", actor.type_id());
            }
        }

        Ok(SensorBridge {
            _vehicle_name: vehicle_name,
            sensor_type,
            _actor: actor,
            sensor_name,
            tx,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}

fn register_camera_rgb(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sesnsor exists"))?;
    let raw_key = key_list[0].clone();
    let info_key = key_list[1].clone();

    let image_publisher = z_session.declare_publisher(raw_key.clone()).wait()?;
    let info_publisher = z_session.declare_publisher(info_key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = image_publisher.put(sensor_data).wait() {
                    log::error!("Failed to publish to {}", raw_key);
                }
            }
            Ok((MessageType::InfoData, info_data)) => {
                if let Err(_) = info_publisher.put(info_data).wait() {
                    log::error!("Failed to publish to {}", info_key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                log::info!("Sensor actor thread for {} stop.", raw_key);
                break;
            }
        }
    });
    let width = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_x")
        .ok_or(BridgeError::CarlaIssue("no image_size_x"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let height = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_y")
        .ok_or(BridgeError::CarlaIssue("no image_size_y"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let fov = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "fov")
        .ok_or(BridgeError::CarlaIssue("no fov"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_f32()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into f32")))? as f64;

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("camera4/camera_link");
        if let Ok(data) = data.try_into() {
            if let Err(e) = camera_callback(header.clone(), data, &tx) {
                log::error!("Failed to call camera_callback: {:?}", e);
            }
            if let Err(e) = camera_info_callback(header, width, height, fov, &tx) {
                log::error!("Failed to call camera_info_callback: {:?}", e);
            }
        } else {
            log::error!("Failed to transform camera image");
        }
    });

    Ok(())
}

fn register_lidar_raycast(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sesnsor exists"))?;
    let key = key_list[0].clone();
    let pcd_publisher = z_session.declare_publisher(key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = pcd_publisher.put(sensor_data).wait() {
                    log::error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                log::info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("velodyne_top");
        if let Ok(data) = data.try_into() {
            if let Err(e) = lidar_callback(header, data, &tx) {
                log::error!("Failed to call lidar_callback: {:?}", e);
            }
        } else {
            log::error!("Failed to transform lidar data");
        }
    });

    Ok(())
}

fn register_lidar_raycast_semantic(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sesnsor exists"))?;
    let key = key_list[0].clone();
    let pcd_publisher = z_session.declare_publisher(key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = pcd_publisher.put(sensor_data).wait() {
                    log::error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                log::info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("velodyne_top");
        if let Ok(data) = data.try_into() {
            if let Err(e) = sematic_lidar_callback(header, data, &tx) {
                log::error!("Failed to call sematic_lidar_callback: {:?}", e);
            }
        } else {
            log::error!("Failed to transform lidar data");
        }
    });

    Ok(())
}

fn register_imu(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sesnsor exists"))?;
    let key = key_list[0].clone();
    let imu_publisher = z_session.declare_publisher(key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = imu_publisher.put(sensor_data).wait() {
                    log::error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                log::info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("tamagawa/imu_link");
        if let Ok(data) = data.try_into() {
            if let Err(e) = imu_callback(header, data, &tx) {
                log::error!("Failed to call imu_callback: {:?}", e);
            }
        } else {
            log::error!("Failed to transform IMU data");
        }
    });
    Ok(())
}

fn register_gnss(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sesnsor exists"))?;
    let key = key_list[0].clone();
    let gnss_publisher = z_session.declare_publisher(key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(_) = gnss_publisher.put(sensor_data).wait() {
                    log::error!("Failed to publish to {}", key);
                }
            }
            _ => {
                // If tx is released, then the thread will stop
                log::info!("Sensor actor thread for {} stop.", key);
                break;
            }
        }
    });
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("gnss_link");
        if let Ok(data) = data.try_into() {
            if let Err(e) = gnss_callback(header, data, &tx) {
                log::error!("Failed to call gnss_callback: {:?}", e);
            }
        } else {
            log::error!("Failed to transform GNSS data");
        }
    });
    Ok(())
}

fn camera_callback(
    header: std_msgs::Header,
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

    let image_msg = sensor_msgs::Image {
        header,
        height: height as u32,
        width: width as u32,
        encoding: "bgra8".to_string(),
        is_bigendian: utils::is_bigendian().into(),
        step: (width * 4) as u32,
        data,
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&image_msg, Infinite)?;
    tx.send((MessageType::SensorData, encoded))
        .or(Err(BridgeError::Communication(
            "Unable to send camera data",
        )))?;
    Ok(())
}

fn camera_info_callback(
    header: std_msgs::Header,
    width: u32,
    height: u32,
    fov: f64,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;
    let fx = width as f64 / (2.0 * (fov * std::f64::consts::PI / 360.0).tan());
    let fy = fx;

    let camera_info = sensor_msgs::CameraInfo {
        header,
        width,
        height,
        distortion_model: String::from("plumb_bob"),
        k: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        d: vec![0.0, 0.0, 0.0, 0.0, 0.0],
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: sensor_msgs::RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&camera_info, Infinite)?;
    tx.send((MessageType::InfoData, encoded))
        .or(Err(BridgeError::Communication(
            "Unable to send camera info data",
        )))?;
    Ok(())
}

fn lidar_callback(
    header: std_msgs::Header,
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
        sensor_msgs::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "intensity".to_string(),
            offset: 12,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "return_type".to_string(),
            offset: 13,
            datatype: PointFieldType::UINT8 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "channel".to_string(),
            offset: 14,
            datatype: PointFieldType::UINT16 as u8,
            count: 1,
        },
    ];
    let lidar_msg = sensor_msgs::PointCloud2 {
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
    tx.send((MessageType::SensorData, encoded))
        .or(Err(BridgeError::Communication("Unable to send lidar data")))?;
    Ok(())
}

fn sematic_lidar_callback(
    header: std_msgs::Header,
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
        sensor_msgs::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "cos_inc_angle".to_string(),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "object_idx".to_string(),
            offset: 16,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::PointField {
            name: "object_tag".to_string(),
            offset: 20,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
    ];
    let lidar_msg = sensor_msgs::PointCloud2 {
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
    tx.send((MessageType::SensorData, encoded))
        .or(Err(BridgeError::Communication("Unable to send lidar data")))?;
    Ok(())
}

fn imu_callback(
    header: std_msgs::Header,
    measure: ImuMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let accel = measure.accelerometer();
    let gyro = measure.gyroscope();
    let compass = measure.compass().to_radians();
    let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, -compass);

    let imu_msg = sensor_msgs::IMU {
        header,
        orientation: geometry_msgs::Quaternion {
            x: orientation.coords.data.0[0][3] as f64,
            y: orientation.coords.data.0[0][1] as f64,
            z: orientation.coords.data.0[0][0] as f64,
            w: orientation.coords.data.0[0][2] as f64,
        },
        orientation_covariance: [0.0; 9],
        angular_velocity: geometry_msgs::Vector3 {
            x: -gyro[0] as f64,
            y: gyro[1] as f64,
            z: -gyro[2] as f64,
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: geometry_msgs::Vector3 {
            x: accel[0] as f64,
            y: -accel[1] as f64,
            z: accel[2] as f64,
        },
        linear_acceleration_covariance: [0.0; 9],
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&imu_msg, Infinite)?;
    tx.send((MessageType::SensorData, encoded))
        .or(Err(BridgeError::Communication("Unable to send IMU data")))?;
    Ok(())
}

fn gnss_callback(
    header: std_msgs::Header,
    measure: GnssMeasurement,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let gnss_msg = sensor_msgs::NavSatFix {
        header,
        latitude: measure.latitude(),
        longitude: measure.longitude(),
        altitude: measure.attitude() + 17.0,
        status: sensor_msgs::NavSatStatus {
            status: GnssStatus::StatusSbasFix as i8,
            service: GnssService::ServiceGps as u16
                | GnssService::ServiceGlonass as u16
                | GnssService::ServiceCompass as u16
                | GnssService::ServiceGalileo as u16,
        },
        position_covariance: [0.0; 9],
        position_covariance_type: 0, // unknown type
    };
    let encoded = cdr::serialize::<_, _, CdrLe>(&gnss_msg, Infinite)?;
    tx.send((MessageType::SensorData, encoded))
        .or(Err(BridgeError::Communication("Unable to send GNSS data")))?;
    Ok(())
}

fn generate_sensor_name(actor: &Sensor) -> String {
    let XYZ { x, y, z } = *actor.location();
    format!("{x}_{y}_{z}")
}

impl Drop for SensorBridge {
    fn drop(&mut self) {
        log::info!("Remove sensor name {}", self.sensor_name);
        if self.sensor_type != SensorType::Collision && self.sensor_type != SensorType::NotSupport {
            // Not sure why the tx doesn't release in sensor callback, so rx can't use RecvErr to close the thread
            // I create another message type to notify the thread to close
            if let Err(_) = self.tx.send((MessageType::StopThread, vec![])) {
                log::error!("Unable to stop the thread")
            }
        }
    }
}
