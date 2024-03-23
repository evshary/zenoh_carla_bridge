# zenoh_carla_bridge

The repository can bridge Carla and Autoware with Zenoh.
The bridge tranforms Carla data into Autoware ROS 2 topic and then sends it with Zenoh.

If you want to have a quick demo, refer to [autoware_carla_launch](https://github.com/evshary/autoware_carla_launch)

## Build

* Get the code

```shell
git clone https://github.com/evshary/zenoh_carla_bridge.git
cd zenoh_carla_bridge
cargo build
# Run lint
cargo clippy --all -- -W clippy::all -W clippy::pedantic -W clippy::restriction -W clippy::nursery -D warnings
```

## Usage

* Terminal1: Run simulator

```shell
./CarlaUE4.sh
```

* Terminal2: Run manual control agent

```shell
cd carla_agent/
poetry run python3 main.py --rolename "v1"
```

* Terminal3: Run zenoh-bridge-ros2dds

```shell
./zenoh-plugin-dds/target/release/zenoh-bridge-ros2dds -n "/v1"
```

* Terminal4: Run zenoh\_carla\_bridge

```shell
source /path/to/autoware/install/setup.bash
RUST_LOG=c=info cargo run
```

* Terminal5: Run ROS package to control vehicle

```shell
# For example
ros2 run autoware_manual_control keyboard_control 
```

## ROS topic

### Input topic

#### Control

* `/control/command/control_cmd`
* `/control/command/emergency_cmd`
* `/control/command/gear_cmd`
* `/control/command/hazard_lights_cmd`: Not support
* `/control/command/turn_indicators_cmd`: Not support

#### Sensing

* `/sensing/camera/traffic_light/camera_info`
* `/sensing/camera/traffic_light/image_raw`
* `/sensing/gnss/pose`: Not support
* `/sensing/gnss/pose_with_covariance`: Not support
* `/sensing/imu/tamagawa/imu_raw`
* `/sensing/lidar/top/pointcloud_raw`
* `/sensing/lidar/top/pointcloud_raw_ex`

### Output topic

* `/vehicle/status/control_mode`
* `/vehicle/status/gear_status`
* `/vehicle/status/steering_status`
* `/vehicle/status/velocity_status`
* `/vehicle/status/hazard_lights_status`: Not support
* `/vehicle/status/turn_indicators_status`: Not support
