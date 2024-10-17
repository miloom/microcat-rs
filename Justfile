set ignore-comments

build:
    just generate
    cargo build --features proto


setup:
  vcs import src < src/ros2_rust/ros2_rust_humble.repos

write:
  just build
  scp target/aarch64-unknown-linux-gnueabihf/release/microcat-rs raspberry@rpi.local:~/
