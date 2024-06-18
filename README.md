![workflow](https://github.com/miloom/microcat-rs/actions/workflows/rust.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Dependencies
1. Install rust `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`


## Build Instructions
1. Install prerequisites (`gcc-arm-linux-gnueabihf`)

2. Run `cargo build` to build the binary

3. Copy the built binary to Raspberry pi with scp `scp ./target/arm-unknown-linux-gnueabihf/debug/microcat-rs raspberry@rpi:/home/raspberry`

4. Run the built binary `ssh raspberry@rpi bash -ic /home/raspberry/microcat-rs`

### Using Cross-rs

1. Install docker and podman `sudo apt install docker podman`

2. Install cross-rs `cargo install cross --git https://github.com/cross-rs/cross`

3. Copy the built binary to Raspberry pi with scp `scp ./target/arm-unknown-linux-gnueabihf/debug/microcat-rs raspberry@rpi:/home/raspberry`

4. Run the built binary `ssh raspberry@rpi bash -ic /home/raspberry/microcat-rs`
