use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc, Mutex, OnceLock,
    },
};

use zenoh::{liveliness::LivelinessToken, Session, Wait};

use crate::{bridge::sensor_bridge::SensorType, Mode};

const CARLA_BRIDGE_NODE_ID: &str = "0";
const NODE_NAME: &str = "carla_bridge";

#[derive(Debug)]
struct TopicInfo {
    entity_kind: &'static str,
    type_name: &'static str,
    type_hash: &'static str,
    qos: &'static str,
}

#[rustfmt::skip]
fn build_topic_map() -> HashMap<&'static str, TopicInfo> {
    let mut m = HashMap::new();

    // === Publishers ===
    m.insert("vehicle/status/actuation_status", TopicInfo { entity_kind: "MP", type_name: "tier4_vehicle_msgs::msg::dds_::ActuationStatusStamped_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/velocity_status", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::VelocityReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/steering_status", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::SteeringReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/gear_status", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::GearReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/control_mode", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::ControlModeReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/turn_indicators_status", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::TurnIndicatorsReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("vehicle/status/hazard_lights_status", TopicInfo { entity_kind: "MP", type_name: "autoware_vehicle_msgs::msg::dds_::HazardLightsReport_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });

    // === Subscribers ===
    m.insert("control/command/actuation_cmd", TopicInfo { entity_kind: "MS", type_name: "tier4_vehicle_msgs::msg::dds_::ActuationCommandStamped_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("control/command/gear_cmd", TopicInfo { entity_kind: "MS", type_name: "autoware_vehicle_msgs::msg::dds_::GearCommand_", type_hash: "TypeHashNotSupported", qos: ":1:,1:,:,:,," });
    m.insert("control/current_gate_mode", TopicInfo { entity_kind: "MS", type_name: "tier4_control_msgs::msg::dds_::GateMode_", type_hash: "TypeHashNotSupported", qos: ":1:,1:,:,:,," });
    m.insert("control/command/turn_indicators_cmd", TopicInfo { entity_kind: "MS", type_name: "autoware_vehicle_msgs::msg::dds_::TurnIndicatorsCommand_", type_hash: "TypeHashNotSupported", qos: ":1:,1:,:,:,," });
    m.insert("control/command/hazard_lights_cmd", TopicInfo { entity_kind: "MS", type_name: "autoware_vehicle_msgs::msg::dds_::HazardLightsCommand_", type_hash: "TypeHashNotSupported", qos: ":1:,1:,:,:,," });

    // === Sensors and Clock ===
    m.insert("sensing/camera/traffic_light/image_raw", TopicInfo { entity_kind: "MP", type_name: "sensor_msgs::msg::dds_::Image_", type_hash: "TypeHashNotSupported", qos: "2::,5:,:,:,," });
    m.insert("sensing/camera/traffic_light/camera_info", TopicInfo { entity_kind: "MP", type_name: "sensor_msgs::msg::dds_::CameraInfo_", type_hash: "TypeHashNotSupported", qos: "2::,5:,:,:,," });
    m.insert("sensing/imu/tamagawa/imu_raw", TopicInfo { entity_kind: "MP", type_name: "sensor_msgs::msg::dds_::Imu_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("sensing/lidar/top/pointcloud", TopicInfo { entity_kind: "MP", type_name: "sensor_msgs::msg::dds_::PointCloud2_", type_hash: "TypeHashNotSupported", qos: "2::,5:,:,:,," });
    m.insert("sensing/gnss/ublox/nav_sat_fix", TopicInfo { entity_kind: "MP", type_name: "sensor_msgs::msg::dds_::NavSatFix_", type_hash: "TypeHashNotSupported", qos: "::,1:,:,:,," });
    m.insert("clock", TopicInfo { entity_kind: "MP", type_name: "rosgraph_msgs::msg::dds_::Clock_", type_hash: "TypeHashNotSupported", qos: "2::,1:,:,:,," });

    m
}

struct Topics {
    mode: Mode,
    z_session: Arc<Session>,
    map: HashMap<&'static str, TopicInfo>,
    tokens: Mutex<Vec<LivelinessToken>>,
    entity_seq: AtomicU64,
}

impl Topics {
    /// Create a new Topics instance with an empty liveliness token list.
    fn new(mode: Mode, z_session: Arc<Session>) -> Self {
        Self {
            mode,
            z_session,
            map: build_topic_map(),
            tokens: Mutex::new(Vec::new()),
            entity_seq: AtomicU64::new(0),
        }
    }

    /// This function will be called to declare node liveliness if the mode is RmwZenoh.
    /// See: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#graph-cache
    fn declare_node_liveliness(&self, prefix: &str) {
        let zid = self.z_session.zid().to_string();
        let nid = CARLA_BRIDGE_NODE_ID;
        let node_name = NODE_NAME;
        let keyexpr = format!("{prefix}@ros2_lv/0/{zid}/{nid}/{nid}/NN/%/%/{node_name}");
        let token = self
            .z_session
            .liveliness()
            .declare_token(keyexpr)
            .wait()
            .unwrap();
        self.tokens.lock().unwrap().push(token);
    }

    /// This function will format the topic depending on the mode.
    /// If the mode is RmwZenoh, it will use the Zenoh key expression format; otherwise, it will use the standard ROS 2 format.
    /// See: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#topic-and-service-name-mapping-to-zenoh-key-expressions
    #[inline]
    fn format_topic_key(&self, prefix: &str, base: &str) -> String {
        match self.mode {
            Mode::RmwZenoh => {
                if let Some(info) = self.map.get(base) {
                    self.declare_topic_liveliness(prefix, base, info);
                    format!("{prefix}*/{base}/{}/{}", info.type_name, info.type_hash)
                } else {
                    log::warn!("unknown base '{base}', using wildcard type/hash");
                    format!("{prefix}*/{base}/*/*")
                }
            }
            Mode::ROS2 => format!("{prefix}{base}"),
            Mode::DDS => format!("{prefix}{base}"),
        }
    }

    /// This function will be called to declare topic liveliness if the mode is RmwZenoh.
    /// See: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#graph-cache
    fn declare_topic_liveliness(&self, prefix: &str, base: &str, info: &TopicInfo) {
        let zid = self.z_session.zid().to_string();
        let nid = CARLA_BRIDGE_NODE_ID;
        let eid = self.entity_seq.fetch_add(1, Ordering::Relaxed).to_string();
        let node_name = NODE_NAME;
        let mangled_qualified_name = format!("/{base}").replace('/', "%");
        let keyexpr = format!(
            "{}@ros2_lv/0/{}/{}/{}/{}/%/%/{}/{}/{}/{}/{}",
            prefix,
            zid,
            nid,
            eid,
            info.entity_kind,
            node_name,
            mangled_qualified_name,
            info.type_name,
            info.type_hash,
            info.qos
        );
        let token = self
            .z_session
            .liveliness()
            .declare_token(keyexpr)
            .wait()
            .unwrap();
        self.tokens.lock().unwrap().push(token);
    }

    /// This function will undeclare all previously declared liveliness tokens.
    fn undeclare_all_liveliness(&self) {
        let mut tokens = self.tokens.lock().unwrap();
        for t in tokens.drain(..) {
            if let Err(e) = t.undeclare().wait() {
                log::warn!("Failed to undeclare liveliness token: {e:?}");
            }
        }
    }
}

static TOPICS: OnceLock<Topics> = OnceLock::new();

pub fn setup_topics(mode: Mode, z_session: Arc<Session>) {
    TOPICS.get_or_init(|| Topics::new(mode, z_session));
}

pub fn declare_node_liveliness(prefix: &str) {
    if let Some(t) = TOPICS.get() {
        if t.mode == Mode::RmwZenoh {
            t.declare_node_liveliness(prefix);
        }
    } else {
        log::error!("TOPICS not initialized! setup_topics() must be called before declare_node_liveliness()");
    }
}

pub fn topic(prefix: &str, base: &str) -> String {
    TOPICS
        .get()
        .expect("setup_topics() must be called first")
        .format_topic_key(prefix, base)
}

pub fn undeclare_all_liveliness() {
    if let Some(t) = TOPICS.get() {
        if t.mode == Mode::RmwZenoh {
            t.undeclare_all_liveliness();
        }
    } else {
        log::error!("TOPICS not initialized! setup_topics() must be called before undeclare_all_liveliness()");
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
        declare_node_liveliness(&prefix);
        Autoware {
            mode: mode.clone(),
            prefix: prefix.clone(),
            _vehicle_name: vehicle_name,
            // Vehicle publish topic
            topic_actuation_status: topic(&prefix, "vehicle/status/actuation_status"),
            topic_velocity_status: topic(&prefix, "vehicle/status/velocity_status"),
            topic_steering_status: topic(&prefix, "vehicle/status/steering_status"),
            topic_gear_status: topic(&prefix, "vehicle/status/gear_status"),
            topic_control_mode: topic(&prefix, "vehicle/status/control_mode"),
            topic_turn_indicators_status: topic(&prefix, "vehicle/status/turn_indicators_status"),
            topic_hazard_lights_status: topic(&prefix, "vehicle/status/hazard_lights_status"),
            // Vehicle subscribe topic
            topic_actuation_cmd: topic(&prefix, "control/command/actuation_cmd"),
            topic_gear_cmd: topic(&prefix, "control/command/gear_cmd"),
            topic_current_gate_mode: topic(&prefix, "control/current_gate_mode"),
            topic_turn_indicators_cmd: topic(&prefix, "control/command/turn_indicators_cmd"),
            topic_hazard_lights_cmd: topic(&prefix, "control/command/hazard_lights_cmd"),
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
                let raw_key = topic(
                    &self.prefix,
                    &format!("sensing/camera/{sensor_name}/image_raw"),
                );
                let info_key = topic(
                    &self.prefix,
                    &format!("sensing/camera/{sensor_name}/camera_info"),
                );
                self.list_image_raw.insert(sensor_name.clone(), raw_key);
                self.list_camera_info.insert(sensor_name, info_key);
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = topic(&self.prefix, &format!("sensing/imu/{sensor_name}/imu_raw"));
                self.list_imu.insert(sensor_name.clone(), imu_key);
            }
            SensorType::LidarRayCast => {
                let lidar_key = topic(
                    &self.prefix,
                    &format!("sensing/lidar/{sensor_name}/pointcloud"),
                );
                self.list_lidar.insert(sensor_name.clone(), lidar_key);
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = topic(
                    &self.prefix,
                    &format!("sensing/lidar/{sensor_name}/semantic_pointcloud"),
                );
                self.list_lidar_semantics
                    .insert(sensor_name.clone(), lidar_key);
            }
            SensorType::Gnss => {
                let gnss_key = topic(
                    &self.prefix,
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
