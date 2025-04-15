FROM ros:humble
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    git \
    tmux \
    protobuf-compiler \
    libclang-dev \
    python3-pip \
    python3-vcstool \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

RUN cargo install cargo-ament-build

RUN pip install --upgrade pytest
RUN pip install git+https://github.com/colcon/colcon-cargo.git
RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git

COPY libcamera-3.2.0/usr /usr

WORKDIR /workspace
