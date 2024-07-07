use std::error::Error;
use std::thread;
use std::time::Duration;

use rppal::i2c::I2c;
use rppal::system::DeviceInfo;

#[allow(dead_code)]
mod consts;
mod imu;
#[allow(dead_code)]
mod rgb;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());

    // let mut rgb = rgb::init_leds()?;
    let mut i2c = I2c::new()?;
    println!("I2c: {:?}", i2c.capabilities());

    imu::setup(&mut i2c)?;

    for _ in 0..100 {
        let accel = imu::get_accel(&mut i2c)?;
        println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        thread::sleep(Duration::from_millis(500));
    }

    Ok(())
}
