use std::error::Error;
use std::thread;
use std::time::Duration;

use icm20608g as imu;
use icm20608g::structs::{AccelConfig1, Config, PowerManagement1, ReadRegister, WriteRegister};
use rppal::i2c::I2c;
use rppal::system::DeviceInfo;

#[allow(dead_code)]
mod consts;
#[allow(dead_code)]
mod rgb;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());

    // let mut rgb = rgb::init_leds()?;
    let mut i2c = I2c::new()?;
    println!("I2c: {:?}", i2c.capabilities());
    imu::who_am_i(&mut i2c);
    let mut pwr_mgmt = PowerManagement1::new(&mut i2c)?;
    pwr_mgmt.print_table(&mut i2c)?;
    pwr_mgmt.sleep = false;
    pwr_mgmt.write(&mut i2c)?;
    let mut config = Config::new(&mut i2c)?;
    config.print_table(&mut i2c)?;
    config.dlpf_cfg = 6;
    config.write(&mut i2c)?;
    let accel_config = AccelConfig1::new(&mut i2c)?;
    accel_config.print_table(&mut i2c)?;
    for _ in 0..100 {
        let acceleration = imu::accel_data(&mut i2c)?;
        println!(
            "Acceleration x: {} y: {} z: {}",
            acceleration.x, acceleration.y, acceleration.z
        );
        thread::sleep(Duration::from_millis(1000));
    }

    Ok(())
}
