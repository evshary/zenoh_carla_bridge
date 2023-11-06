use crate::bridge::sensor_bridge::SensorType;
use std::collections::HashMap;

#[derive(Default, Clone)]
pub struct Autoware {
    pub ros2: bool,
    pub prefix: String,
    pub vehicle_name: String,
    // Vehicle publish topic
    pub topic_velocity_status: String,
    pub topic_steering_status: String,
    pub topic_gear_status: String,
    pub topic_control_mode: String,
    pub topic_turn_indicators_status: String,
    pub topic_hazard_lights_status: String,
    // Vehicle subscribe topic
    pub topic_control_cmd: String,
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
    pub fn new(vehicle_name: String, ros2: bool) -> Autoware {
        let prefix = if ros2 {
            format!("{vehicle_name}/")
        } else {
            format!("{vehicle_name}/rt/")
        };
        Autoware {
            ros2,
            prefix: prefix.clone(),
            vehicle_name,
            // Vehicle publish topic
            topic_velocity_status: prefix.clone() + "vehicle/status/velocity_status",
            topic_steering_status: prefix.clone() + "vehicle/status/steering_status",
            topic_gear_status: prefix.clone() + "vehicle/status/gear_status",
            topic_control_mode: prefix.clone() + "vehicle/status/control_mode",
            topic_turn_indicators_status: prefix.clone() + "vehicle/status/turn_indicators_status",
            topic_hazard_lights_status: prefix.clone() + "vehicle/status/hazard_lights_status",
            // Vehicle subscribe topic
            topic_control_cmd: prefix.clone() + "control/command/control_cmd",
            topic_gear_cmd: prefix.clone() + "control/command/gear_cmd",
            topic_current_gate_mode: prefix.clone() + "control/command/current_gate_mode",
            topic_turn_indicators_cmd: prefix.clone() + "control/command/turn_indicators_cmd",
            topic_hazard_lights_cmd: prefix.clone() + "control/command/hazard_lights_cmd",
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
                let raw_key = format!(
                    "{}sensing/camera/{sensor_name}/image_raw",
                    self.prefix.clone()
                );
                let info_key = format!(
                    "{}sensing/camera/{sensor_name}/camera_info",
                    self.prefix.clone()
                );
                self.list_image_raw.insert(sensor_name.clone(), raw_key);
                self.list_camera_info.insert(sensor_name, info_key);
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = format!("{}sensing/imu/{sensor_name}/imu_raw", self.prefix.clone());
                self.list_imu.insert(sensor_name.clone(), imu_key);
            }
            SensorType::LidarRayCast => {
                let lidar_key = format!("{}carla_pointcloud", self.prefix.clone());
                self.list_lidar.insert(sensor_name.clone(), lidar_key);
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = format!("{}carla_pointcloud", self.prefix.clone());
                self.list_lidar_semantics
                    .insert(sensor_name.clone(), lidar_key);
            }
            SensorType::Gnss => {
                let gnss_key = format!(
                    "{}sensing/gnss/{sensor_name}/nav_sat_fix",
                    self.prefix.clone()
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
                if raw_key != None && info_key != None {
                    return Some(vec![
                        raw_key.unwrap().to_owned(),
                        info_key.unwrap().to_owned(),
                    ]);
                }
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = self.list_imu.get(sensor_name);
                if imu_key != None {
                    return Some(vec![imu_key.unwrap().to_owned()]);
                }
            }
            SensorType::LidarRayCast => {
                let lidar_key = self.list_lidar.get(sensor_name);
                if lidar_key != None {
                    return Some(vec![lidar_key.unwrap().to_owned()]);
                }
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = self.list_lidar.get(sensor_name);
                if lidar_key != None {
                    return Some(vec![lidar_key.unwrap().to_owned()]);
                }
            }
            SensorType::Gnss => {
                let gnss_key = self.list_gnss.get(sensor_name);
                if gnss_key != None {
                    return Some(vec![gnss_key.unwrap().to_owned()]);
                }
            }
            SensorType::NotSupport => {}
        };
        None
    }
}
