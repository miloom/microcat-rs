![Build](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Fmiloom%2Fmicrocat-rs%2Fgh-pages%2Fbuild.json)
![Lint](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Fmiloom%2Fmicrocat-rs%2Fgh-pages%2Flint.json)
![Format](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Fmiloom%2Fmicrocat-rs%2Fgh-pages%2Ffmt.json)

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
sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y
```

4. Install ROS2 Rust

```bash
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

5. Update git submodules
```
git submodule init && git submodule update
```

6. On the first build set up ROS2 repositories

```bash
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

For pip packages in ubuntu distributions 24.04 and above it's recommended to use pipx.
Pip no longer allows installing directly to system packages.
Testing with pipx showed that it fails to install the packages correctly which causes colcon build to fail.
To see if the packages have been installed correctly you can run ```colcon list``` and verify that the packages
rosidl_runtime_rs and rosidl_generator_rs do not end with *(ros.ament_cargo)*

7. Install extra dependencies to build
   These dependencies are used by some packages in Cargo.toml

```bash
sudo apt install libudev-dev clang libstdc++-12-dev -y
```

## Building Libcamera

We will be cross compiling libcamera because RPI is quite weak and development PC is most likely more powerful.

1. Download newer version of Meson Build from https://github.com/mesonbuild/meson/releases
2. Extract the archive
    ```bash
    tar -xvf meson-1.x.y.tar.gz
    ```
3. Install on the system
    ```bash
    sudo python meson-1.x.y/setup.py install
    ```
4. Clone libcamera
    ```bash
    git clone https://git.libcamera.org/libcamera/libcamera.git
    cd libcamera
    git checkout v0.3.2
    ```
5. Add arm64 package repositories
    ```bash
    sudo dpkg --add-architecture arm64 && echo -e 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse\ndeb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse\ndeb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse' | sudo tee /etc/apt/sources.list.d/ubuntu-ports-arm64.list > /dev/null && sudo apt update
    ```
6. Install dependencies
    ```bash
    sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu pkg-config ninja-build libyaml-dev:arm64 python3-yaml python3-ply python3-jinja2 openssl libudev-dev:arm64 libevent-dev libssl-dev:arm64 
    ```
7. Create cross file `aarch64_cross.txt`
    ```text
    [binaries]
    c = 'aarch64-linux-gnu-gcc'
    cpp = 'aarch64-linux-gnu-g++'
    ar = 'aarch64-linux-gnu-gcc-ar'
    strip = 'aarch64-linux-gnu-strip'
    pkg-config = 'aarch64-linux-gnu-pkg-config'
    cmake = 'cmake'
    
    [host_machine]
    system = 'linux'
    cpu_family = 'aarch64'
    cpu = 'aarch64'
    endian = 'little'
    ```
8. Build
   ```bash
   meson setup cross-build --cross-file aarch64_cross.txt --buildtype=release
   ninja -C cross-build 
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

### Setup

1. Configure full uart
   This will disable bluetooth
   Add this line below any section following a `[all]` name  
   `dtoverlay=disable-bt`
2. Disable console for uart
   ```bash
   sudo systemctl stop serial-getty@ttyS0.service
   sudo systemctl disable serial-getty@ttyS0.service
   sudo systemctl mask serial-getty@ttyS0.service
   ```
3. Add user to tty group
   ```bash
   sudo adduser $USER tty
   ```
4. Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`
5. Configure hardware PWM
   We will need hardware PWM to run.
   This will disable analog audio
   Add this line below any section following a `[all]` name  
   `dtoverlay=pwm-2chan,pin=18,pin2=13,func=2,func2=4`
    * Set up udev rules for permission to access the PWM  
      Create a file `/etc/udev/rules.d/99-pwm.rules` with following contents:
      ```
      SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
        chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
        chown -R root:gpio /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*\
        '"
      ```
    * Reload the udev rules:
      ```bash
      sudo udevadm control --reload-rules
      sudo udevadm trigger
      ```
6. Reboot


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

