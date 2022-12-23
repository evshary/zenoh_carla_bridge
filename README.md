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
python3 PythonAPI/examples/manual_control.py --rolename "v1"
```

* Terminal3: Run bridge DDS

```shell
./zenoh-plugin-dds/target/release/zenoh-bridge-dds -s "v1"
```

* Terminal4: Run carla_autoware_zenoh_bridge

```shell
cargo run
```

* Terminal5: Run ROS package to control vehicle

```shell
# For example
ros2 run autoware_manual_control keyboard_control 
```
