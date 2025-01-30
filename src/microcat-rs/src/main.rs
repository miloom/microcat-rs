#![allow(dead_code)]
#![allow(unused_variables)]
use crate::serial::MotorPos;
#[cfg(feature = "rppal")]
use rppal::gpio::Gpio;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;
#[cfg(feature = "ros")]
use std_msgs::msg::String as StringMsg;
use tokio::sync::Mutex;

#[allow(dead_code)]
mod consts;
#[cfg(feature = "rppal")]
mod imu;
// #[allow(dead_code), cfg(feature = "rppal")]
#[cfg(feature = "rppal")]
mod motors;
// #[allow(dead_code), cfg(feature="rppal")]
#[cfg(feature = "rppal")]
mod rgb;
#[cfg(feature = "proto")]
mod serial;

#[cfg(feature = "ros")]
struct MicrocatNode {
    node: Arc<rclrs::Node>,
    motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    publisher: Arc<rclrs::Publisher<StringMsg>>,
    data: Arc<std::sync::Mutex<Option<StringMsg>>>,
}

#[cfg(feature = "ros")]
impl MicrocatNode {
    fn new(context: &rclrs::Context, serial_stream: Arc<Mutex<Box<dyn SerialPort>>>) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "microcat_node")?;
        let data = Arc::new(std::sync::Mutex::new(None));
        let data_cb = Arc::clone(&data);

        let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
        let serial_stream_cb = Arc::clone(&serial_stream);
        let motor_control_subscription = {
            node.create_subscription(
                "motor_control",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: microcat_msgs::msg::MotorControl| {
                    let command = serial::Command::MotorPosition(MotorPos {
                        location: match msg.location {
                            microcat_msgs::msg::MotorControl::FRONT_LEFT => {
                                serial::MotorLocation::FrontLeft
                            }
                            microcat_msgs::msg::MotorControl::FRONT_RIGHT => {
                                serial::MotorLocation::FrontRight
                            }
                            microcat_msgs::msg::MotorControl::REAR_LEFT => {
                                serial::MotorLocation::RearLeft
                            }
                            microcat_msgs::msg::MotorControl::REAR_RIGHT => {
                                serial::MotorLocation::RearRight
                            }
                            _ => {
                                // We will not panic since the sender might send invalid data
                                return;
                            }
                        },
                        target_position: msg.position,
                        frequency: msg.frequency,
                        amplitude: msg.amplitude,
                    });
                    let mut guard = tokio::runtime::Runtime::new().unwrap().block_on(serial_stream_cb.lock());
                    serial::write(&mut *guard, command);
                },
            )?
        };
        Ok(Self {
            node,
            motor_control_subscription,
            publisher,
            data,
        })
    }

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }
        Ok(())
    }
}


#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let serial = Arc::new(Mutex::new(serialport::new("/dev/ttyS0", 115_200)
        .timeout(Duration::from_millis(100))
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .open().unwrap()));

    #[cfg(feature = "ros")]
    let context = rclrs::Context::new(std::env::args())?;
    #[cfg(feature = "ros")]
    let microcat_node = Arc::new(MicrocatNode::new(&context, Arc::clone(&serial))?);
    #[cfg(feature = "ros")]
    let microcat_other_thread = Arc::clone(&microcat_node);
    #[cfg(feature = "ros")]
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            microcat_other_thread.republish()?;
        }
    });

    /*
    let mut rgb = rgb::init_leds()?;

    tokio::task::spawn(async move {
        rgb::test_leds(&mut rgb).await.unwrap();
    });*/

    #[cfg(feature = "rppal")]
    let gpio = Gpio::new()?;
    #[cfg(feature = "rppal")]
    let mut pin = gpio.get(19)?.into_output();
    #[cfg(feature = "rppal")]
    pin.set_low();
    #[cfg(feature = "rppal")]
    let mut new_pin = gpio.get(26)?.into_output();
    #[cfg(feature = "rppal")]
    new_pin.set_low();
    #[cfg(feature = "rppal")]
    println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

    // let i2c = I2c::new()?;
    // println!("I2c: {:?}", i2c.capabilities());

    // let i2c = Arc::from(std::sync::Mutex::from(i2c));
    // motors::test(i2c.clone()).await?;

    // imu::setup(&mut i2c.lock().unwrap())?;

    let message_buffer = bytes::BytesMut::with_capacity(1024);
    let initialized = false;

    for _ in 0..100_000 {
        let mut buf = [0u8; 128];
        {
            let mut port_guard = serial.lock().await;
            if let Ok(size) = port_guard.read(&mut buf) {
                println!("{}", buf.iter().map(|&b| b as char).collect::<String>());
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    #[cfg(feature = "ros")]
    rclrs::spin(Arc::clone(&microcat_node.node))?;
    Ok(())
}
