use std::error::Error;
use std::thread;
use std::time::Duration;
use std::sync::{Arc, Mutex};

use rppal::i2c::I2c;
use rppal::system::DeviceInfo;

#[allow(dead_code)]
mod consts;
mod imu;
#[allow(dead_code)]
mod rgb;
mod motors;

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
    imu::setup(&mut *i2c.lock().unwrap())?;

    for _ in 0..100 {
        let accel = imu::get_accel(&mut *i2c.lock().unwrap())?;
        println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        thread::sleep(Duration::from_millis(500));
    }

    Ok(())
}
