![build](https://github.com/miloom/microcat-rs/actions/workflows/build.yml/badge.svg) ![lint](https://github.com/miloom/microcat-rs/actions/workflows/lint.yml/badge.svg) ![format](https://github.com/miloom/microcat-rs/actions/workflows/format.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Dependencies
This needs to be built on Ubuntu machine because of the ROS2 dependency. The version of the ubuntu depends on the version of ROS used. Currently the project uses ROS2 Humble and needs Ubuntu 22.04 to be built and ran.

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
just setup
```
For pip packages in ubuntu distributions 24.04 and above it's recommended to use pipx.
Pip no longer allows installing directly to system packages.
for this solution use:
```bash
sudo apt install pipx
pipx install git+https://github.com/colcon/colcon-cargo.git --include-deps
pipx install git+https://github.com/colcon/colcon-ros-cargo.git --include-deps
```


## Build Instructions
When building you have to specifiy features depending on whether the target that the code will be ran on is a raspberry pi or not. These features enable or disable specific features which allow the same code to be ran on devicest that are not raspberry pi. Most functionality will be missing when running on another device, but it allows for simple verification.

When building you have to source your ROS2 install

1. Build the binary 
```bash
colcon build
```
