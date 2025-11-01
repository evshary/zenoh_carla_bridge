# zenoh_carla_bridge

The repository can bridge Carla and Autoware with Zenoh.
The bridge transforms Carla data into Autoware ROS 2 messages and then sends it via Zenoh.

If you want to have a quick demo, refer to [autoware_carla_launch](https://autoware-carla-launch.readthedocs.io/en/latest/)

## Build

* Get the code

```shell
git clone https://github.com/evshary/zenoh_carla_bridge.git
cd zenoh_carla_bridge
CARLA_VERSION=0.9.14 cargo build
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
RUST_LOG=c=info CARLA_VERSION=0.9.14 cargo run
```

## Note

If you want to build in Ubuntu 22.04, remember to switch compiler's version.

```shell
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
```

## For Developers

You can use pre-commit and Ruff to have correct Python format

```shell
python3 -m pip install pre-commit ruff
pre-commit install --install-hooks
```
