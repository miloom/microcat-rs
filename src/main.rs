use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use rppal::gpio::Gpio;
use rppal::i2c::I2c;
use rppal::system::DeviceInfo;
use tokio_serial::{DataBits, FlowControl, Parity, SerialPortBuilderExt, StopBits};

#[allow(dead_code)]
mod consts;
mod imu;
#[allow(dead_code)]
mod motors;
#[allow(dead_code)]
mod rgb;
mod serial;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());
    rosrust::init("talker_test");

    let chatter_pub = rosrust::publish("chatter", 2).unwrap();
    chatter_pub.wait_for_subscribers(None).unwrap();

    let log_names = rosrust::param("~log_names").unwrap().get().unwrap_or(false);

    let mut count = 0;

    let rate = rosrust::rate(10.0); //10Hz

    while rosrust::is_ok() {
        let msg = rosrust_msg::microcat_msgs::string {
            data: format!("hello world from rosrust {}", count),
        };

        rosrust::ros_info!("Publishing {}", msg.data);

        chatter_pub.send(msg).unwrap();

        if log_names {
            rosrust::ros_info!("Subscriber names: {:?}", chatter_pub.subscriber_names());
        }

        rate.sleep();

        count += 1;
    }

    #[cfg(feature = "proto")]
    println!("Using Proto");

    /*
    let mut rgb = rgb::init_leds()?;
    
    tokio::task::spawn(async move {
        rgb::test_leds(&mut rgb).await.unwrap();
    });*/


    let gpio = Gpio::new()?;
    let mut pin = gpio.get(19)?.into_output();
    pin.set_low();
    let mut new_pin = gpio.get(26)?.into_output();
    new_pin.set_low();
    println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

    let i2c = I2c::new()?;
    println!("I2c: {:?}", i2c.capabilities());

    println!("Motors test");
    let i2c = Arc::from(Mutex::from(i2c));
    // motors::test(i2c.clone()).await?;

    println!("IMU");
    imu::setup(&mut i2c.lock().unwrap())?;

    let mut serial = tokio_serial::new("/dev/ttyS0", 115_200)
        .timeout(std::time::Duration::from_secs(2))
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .open_native_async()?;

    /*
    let mut serial = rppal::uart::Uart::with_path("/dev/serial0",115_200, rppal::uart::Parity::None, 8, 1)?;
    serial.set_read_mode(16, std::time::Duration::ZERO)?;
    dbg!(serial.input_len()?);
    dbg!(serial.is_read_blocking());
    */
    let mut message_buffer = bytes::BytesMut::with_capacity(1024);
    let mut initialized = false;

    for _ in 0..100_000 {
        let accel = imu::get_accel(&mut i2c.lock().unwrap())?;
        // println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        serial::read_step(&mut serial, &mut message_buffer, &mut initialized).await;
        thread::sleep(Duration::from_millis(5));
    }

    Ok(())
}
