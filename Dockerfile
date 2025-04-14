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

#RUN groupadd -g 1000 builder && \
#    useradd -m -u 1000 -g builder -s /bin/bash builder

USER 1000:1000

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
ENV PATH="$PATH:/root/.cargo/bin/"

RUN pip install --upgrade pytest

RUN cargo install cargo-ament-build

RUN pip install git+https://github.com/colcon/colcon-cargo.git

RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git


WORKDIR /workspace


#CMD ["colcon", "--log-base", "arm64/log", "build", "--build-base", "arm64/build", "--install-base", "arm64/install", "--merge-install", "--packages-up-to", "microcat_rs"]
#CMD ["/bin/sh"]