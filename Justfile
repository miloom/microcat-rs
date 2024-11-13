set ignore-comments

setup:
  sudo apt install protobuf-compiler -y
  sudo apt install software-properties-common -y
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt install ros-humble-ros-base ros-dev-tools -y
  sudo apt install -y git libclang-dev python3-pip python3-vcstool
  cargo install cargo-ament-build
  pip install git+https://github.com/colcon/colcon-cargo.git
  pip install git+https://github.com/colcon/colcon-ros-cargo.git
  vcs import src < src/ros2_rust/ros2_rust_humble.repos


