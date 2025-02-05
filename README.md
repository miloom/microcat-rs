![build](https://github.com/miloom/microcat-rs/actions/workflows/build.yml/badge.svg) ![lint](https://github.com/miloom/microcat-rs/actions/workflows/lint.yml/badge.svg) ![format](https://github.com/miloom/microcat-rs/actions/workflows/format.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Dependencies

This needs to be built on Ubuntu machine because of the ROS2 dependency. The version of the ubuntu depends on the
version of ROS used. Currently the project uses ROS2 Humble and needs Ubuntu 22.04 to be built and ran.

The Microcat robot uses an ARM64V8 processor, which means that the code has to be built on the same architecture.

1. Install rust on the development machine

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

2. Install protobuf compiler

```bash
sudo apt install protobuf-compiler -y
protoc --version # Ensure you are using compiler version 3+
```

3. Install ROS2

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install ros-humble-ros-base ros-dev-tools -y
```

4. Install ROS2 Rust

```bash
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

For pip packages in ubuntu distributions 24.04 and above it's recommended to use pipx.
Pip no longer allows installing directly to system packages.
Testing with pipx showed that it fails to install the packages correctly which causes colcon build to fail.
To see if the packages have been installed correctly you can run ```colcon list``` and verify that the packages
rosidl_runtime_rs and rosidl_generator_rs do not end with *(ros.ament_cargo)*

5. Install extra dependencies to build
   These dependencies are used by some packages in Cargo.toml

```bash
sudo apt install libudev-dev -y
```

## Build Instructions

When building you have to source your ROS2 install
e.g.

```bash 
source /opt/ros/humble/setup.bash
```

This sourcing needs to be done once per shell where build is ran. Consecutive builds do not need to source the files
again

1. Build the binary

```bash
colcon build --merge-install --packages-up-to microcat_rs
```

## Running the binary

The code is setup for running on the raspberry. While building is possible on other machines the code directly
interfaces with the Raspberry PI hardware, which means other computers will fail to run it.

1. Copy the built files over to the raspberry
    - Archive the 'install' directory
    ```bash
    tar cvJf microcat.tar.xz install
    ```
    - Copy the archive to raspberry
    ```bash
    scp microcat.tar.xz raspberry@rpi.local:~/
    ```
    - Extract the 'install' directory on the raspberry pi
    ```bash
   tar -xvJf microcat.tar.xz
    ```
2. Source the ROS2 installation

```bash
source /opt/ros/humble/setup.bash
```

3. Source the 'install' directory script

```bash
source ./install/setup.bash
```

4. Run the Microcat node using ROS2

```bash
ros2 run microcat-rs microcat-rs
```

