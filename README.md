![workflow](https://github.com/miloom/microcat-rs/actions/workflows/rust.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Build Instructions
1. Install prerequisites (`gcc-arm-linux-gnueabihf`)

2. Run `cargo build` to build the binary

3. Copy the built binary to Raspberry pi with scp `scp ./target/armv7-unknown-linux-gnueabihf/debug/microcat-rs raspberry@rpi:/home/raspberry`

4. Run the built binary `ssh raspberry@rpi bash -ic /home/raspberry/microcast-rs`
