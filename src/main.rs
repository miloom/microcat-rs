use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use rppal::i2c::I2c;
use rppal::system::DeviceInfo;
use tokio_serial::{DataBits, FlowControl, Parity, SerialPortBuilderExt, StopBits};

#[allow(dead_code)]
mod consts;
mod imu;
mod motors;
#[allow(dead_code)]
mod rgb;
mod serial;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());

    // let mut rgb = rgb::init_leds()?;
    let i2c = I2c::new()?;
    println!("I2c: {:?}", i2c.capabilities());

    println!("Motors test");
    let i2c = Arc::from(Mutex::from(i2c));
    motors::test(i2c.clone()).await?;

    println!("IMU");
    imu::setup(&mut i2c.lock().unwrap())?;

    let mut serial = tokio_serial::new("/dev/ttyUSB0", 115_200)
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .open_native_async()?;
    let mut message_buffer = bytes::BytesMut::with_capacity(1024);
    let mut initialized = false;

    for _ in 0..100 {
        let accel = imu::get_accel(&mut i2c.lock().unwrap())?;
        println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        serial::read_step(&mut serial, &mut message_buffer, &mut initialized);
        thread::sleep(Duration::from_millis(500));
    }

    Ok(())
}
