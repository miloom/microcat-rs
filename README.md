![build](https://github.com/miloom/microcat-rs/actions/workflows/build.yml/badge.svg) ![lint](https://github.com/miloom/microcat-rs/actions/workflows/lint.yml/badge.svg) ![format](https://github.com/miloom/microcat-rs/actions/workflows/format.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Dependencies
1. Install rust on the development machine 
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

2. Install the static libraries to raspberry pi
```bash
sudo apt install libc6:armhf libstdc++6:armhf
```

## Build Instructions
1. Install prerequisites (`gcc-arm-linux-gnueabihf`)

2. Build the binary 
```bash
cargo build
```

3. Copy the built binary to Raspberry pi with scp 
```bash
scp ./target/arm-unknown-linux-gnueabihf/debug/microcat-rs raspberry@rpi:/home/raspberry
```

4. Run the built binary 
```bash
ssh raspberry@rpi bash -ic /home/raspberry/microcat-rs
```

### Using Cross-rs

1. Install docker and podman 
```bash
sudo apt install docker podman
```

2. Install cross-rs 
```bash
cargo install cross --git https://github.com/cross-rs/cross
```

3. Copy the built binary to Raspberry pi with scp 
```bash
scp ./target/arm-unknown-linux-gnueabihf/debug/microcat-rs raspberry@rpi:/home/raspberry
```

4. Run the built binary 
```bash
ssh raspberry@rpi bash -ic /home/raspberry/microcat-rs
```


# Current solution for proto files
1. Copy proto files from `microcat-avr` repo to ./proto directory in this repo
2. Run this command to generate rust code `protoc --prost_out=src/serial  --prost_opt=enable_type_names=true,compile_well_known_types=true,default_package_filename=encoder.rs -I proto proto/encoder.proto`
