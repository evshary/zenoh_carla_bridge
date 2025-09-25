use std::collections::HashMap;

use crate::{bridge::sensor_bridge::SensorType, Mode};

// This function will format the topic depending on the mode.
// If the mode is RmwZenoh, it will use the Zenoh key expression format; otherwise, it will use the standard ROS 2 format.
// See: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#topic-and-service-name-mapping-to-zenoh-key-expressions
#[inline]
fn topic_fmt(prefix: &str, mode: &Mode, base: &str) -> String {
    if *mode == Mode::RmwZenoh {
        format!("{prefix}*/{base}/*/*")
    } else {
        format!("{prefix}{base}")
    }
}

#[derive(Clone)]
pub struct Autoware {
    pub mode: Mode,
    pub prefix: String,
    pub _vehicle_name: String,
    // Vehicle publish topic
    pub topic_actuation_status: String,
    pub topic_velocity_status: String,
    pub topic_steering_status: String,
    pub topic_gear_status: String,
    pub topic_control_mode: String,
    pub topic_turn_indicators_status: String,
    pub topic_hazard_lights_status: String,
    // Vehicle subscribe topic
    pub topic_actuation_cmd: String,
    pub topic_gear_cmd: String,
    pub topic_current_gate_mode: String,
    pub topic_turn_indicators_cmd: String,
    pub topic_hazard_lights_cmd: String,
    // Sensor publish topic
    pub list_image_raw: HashMap<String, String>,
    pub list_camera_info: HashMap<String, String>,
    pub list_lidar: HashMap<String, String>,
    pub list_lidar_semantics: HashMap<String, String>,
    pub list_gnss: HashMap<String, String>,
    pub list_imu: HashMap<String, String>,
}

impl Autoware {
    pub fn new(vehicle_name: String, mode: Mode) -> Autoware {
        let prefix = match mode {
            Mode::ROS2 | Mode::RmwZenoh => format!("{vehicle_name}/"),
            Mode::DDS => format!("{vehicle_name}/rt/"),
        };
        let topic = |base: &str| topic_fmt(&prefix, &mode, base);
        Autoware {
            mode: mode.clone(),
            prefix: prefix.clone(),
            _vehicle_name: vehicle_name,
            // Vehicle publish topic
            topic_actuation_status: topic("vehicle/status/actuation_status"),
            topic_velocity_status: topic("vehicle/status/velocity_status"),
            topic_steering_status: topic("vehicle/status/steering_status"),
            topic_gear_status: topic("vehicle/status/gear_status"),
            topic_control_mode: topic("vehicle/status/control_mode"),
            topic_turn_indicators_status: topic("vehicle/status/turn_indicators_status"),
            topic_hazard_lights_status: topic("vehicle/status/hazard_lights_status"),
            // Vehicle subscribe topic
            topic_actuation_cmd: topic("control/command/actuation_cmd"),
            topic_gear_cmd: topic("control/command/gear_cmd"),
            topic_current_gate_mode: topic("control/current_gate_mode"),
            topic_turn_indicators_cmd: topic("control/command/turn_indicators_cmd"),
            topic_hazard_lights_cmd: topic("control/command/hazard_lights_cmd"),
            // Sensor publish topic
            list_image_raw: HashMap::new(),
            list_camera_info: HashMap::new(),
            list_lidar: HashMap::new(),
            list_lidar_semantics: HashMap::new(),
            list_gnss: HashMap::new(),
            list_imu: HashMap::new(),
        }
    }

    pub fn add_sensors(&mut self, sensor_type: SensorType, sensor_name: String) {
        match sensor_type {
            SensorType::CameraRgb => {
                let raw_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/camera/{sensor_name}/image_raw"),
                );
                let info_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/camera/{sensor_name}/camera_info"),
                );
                self.list_image_raw.insert(sensor_name.clone(), raw_key);
                self.list_camera_info.insert(sensor_name, info_key);
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/imu/{sensor_name}/imu_raw"),
                );
                self.list_imu.insert(sensor_name.clone(), imu_key);
            }
            SensorType::LidarRayCast => {
                let lidar_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/lidar/{sensor_name}/pointcloud"),
                );
                self.list_lidar.insert(sensor_name.clone(), lidar_key);
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/lidar/{sensor_name}/pointcloud"),
                );
                self.list_lidar_semantics
                    .insert(sensor_name.clone(), lidar_key);
            }
            SensorType::Gnss => {
                let gnss_key = topic_fmt(
                    &self.prefix,
                    &self.mode,
                    &format!("sensing/gnss/{sensor_name}/nav_sat_fix"),
                );
                self.list_gnss.insert(sensor_name.clone(), gnss_key);
            }
            SensorType::NotSupport => {}
        }
    }

    pub fn get_sensors_key(
        &self,
        sensor_type: SensorType,
        sensor_name: &str,
    ) -> Option<Vec<String>> {
        match sensor_type {
            SensorType::CameraRgb => {
                let raw_key = self.list_image_raw.get(sensor_name);
                let info_key = self.list_camera_info.get(sensor_name);
                if let (Some(raw_key), Some(info_key)) = (raw_key, info_key) {
                    return Some(vec![raw_key.to_owned(), info_key.to_owned()]);
                }
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = self.list_imu.get(sensor_name);
                if let Some(imu_key) = imu_key {
                    return Some(vec![imu_key.to_owned()]);
                }
            }
            SensorType::LidarRayCast => {
                let lidar_key = self.list_lidar.get(sensor_name);
                if let Some(lidar_key) = lidar_key {
                    return Some(vec![lidar_key.to_owned()]);
                }
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = self.list_lidar_semantics.get(sensor_name);
                if let Some(lidar_key) = lidar_key {
                    return Some(vec![lidar_key.to_owned()]);
                }
            }
            SensorType::Gnss => {
                let gnss_key = self.list_gnss.get(sensor_name);
                if let Some(gnss_key) = gnss_key {
                    return Some(vec![gnss_key.to_owned()]);
                }
            }
            SensorType::NotSupport => {}
        };
        None
    }
}
