![workflow](https://github.com/miloom/microcat-rs/actions/workflows/rust.yml/badge.svg)

microcat-rs
===========

Rust project for _Microcat_.

## Dependencies
1. Install rust 
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
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
