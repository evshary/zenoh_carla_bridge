# carla_autoware_zenoh_bridge

# Build

* Get the code

```shell
git clone https://github.com/evshary/carla_autoware_zenoh_bridge.git
cd carla_autoware_zenoh_bridge
cargo build
```

# Usage

* Terminal1: Run simulator

```shell
./CarlaUE4.sh
```

* Terminal2: Run manual control agent

```shell
cd carla_agent/
poetry run python3 main.py --rolename "v1"
```

* Terminal3: Run bridge DDS

```shell
./zenoh-plugin-dds/target/release/zenoh-bridge-dds -s "v1"
```

* Terminal4: Run carla\_autoware\_zenoh\_bridge

```shell
source /path/to/autoware/install/setup.bash
RUST_LOG=c=info cargo run
```

* Terminal5: Run ROS package to control vehicle

```shell
# For example
ros2 run autoware_manual_control keyboard_control 
```

# ROS topic

## Input topic

### Control

* `/control/command/control_cmd`
* `/control/command/emergency_cmd`
* `/control/command/gear_cmd`
* `/control/command/hazard_lights_cmd`: Not support
* `/control/command/turn_indicators_cmd`: Not support

### Sensing

* `/sensing/camera/traffic_light/camera_info`
* `/sensing/camera/traffic_light/image_raw`
* `/sensing/gnss/pose`: Not support
* `/sensing/gnss/pose_with_covariance`: Not support
* `/sensing/imu/tamagawa/imu_raw`
* `/sensing/lidar/top/pointcloud_raw`
* `/sensing/lidar/top/pointcloud_raw_ex`

## Output topic

* `/vehicle/status/control_mode`
* `/vehicle/status/gear_status`
* `/vehicle/status/steering_status`
* `/vehicle/status/velocity_status`
* `/vehicle/status/hazard_lights_status`: Not support
* `/vehicle/status/turn_indicators_status`: Not support
