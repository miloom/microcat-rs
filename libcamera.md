## Crosscompiling libcamera for aarch64

Since we are running on raspberry pi we will need libcamera that is built for the correct target.
Since we need to link this to Rust code we will also need the header files from that build.
Easiest way to achieve both is to crosscompile libcamera and link against that from our machine.

1. Get libcamera 0.3.2 from git
    ```bash
    git clone https://github.com/raspberrypi/libcamera.git
    cd libcamera
    git checkout v0.3.2
    ```

2. Install dependencies
   ```bash
   sudo apt install gcc-aarch64-linux-gnu meson libssl-dev:arm64 libyaml-dev:arm64 libudev-dev:arm64 libjpeg-dev:arm64 libdrm-dev:arm64 libudev-dev:arm64  libsdl2-dev:arm64 libgstreamer1.0-dev:arm64 libgstreamer-plugins-base1.0-dev:arm64 libgnutls28-dev:arm64 libcamera-dev python3-yaml python3-ply python3-jinja2
   ```

3. Create cross-build file for Meson (called aarch64_cross.txt in this guide)
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
4. Cross compile in libcamera directory
    ```bash
    meson setup cross-build --cross-file aarch64_cross.txt --buildtype=release --prefix=/usr -Dpipelines=uvcvideo,rpi/vc4
    DESTDIR=~/libcamera-aarch64 ninja -C cross-build install
    ```

## Building Microcat

1. Setup environment variables
   ```bash
   export PKG_CONFIG_PATH=~/libcamera-aarch64/lib/pkgconfig/
   export PKG_CONFIG_ALLOW_CROSS=1
   export CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc
   export CXX=aarch64-linux-gnu-g++
   export CFLAGS="--sysroot=~/libcamera-aarch64 -I~/libcamera-aarch64/include"
   export CXXFLAGS="--sysroot=~/libcamera-aarch64 -I~/libcamera-aarch64/include"
   export CRATE_CC_NO_DEFAULTS=1
   export AR=aarch64-linux-gnu-ar
   export RUSTFLAGS="-C linker=aarch64-linux-gnu-gcc"
   export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
   ```