#![allow(dead_code)]
#![allow(unused_variables)]
use crate::serial::MotorPos;
#[cfg(feature = "rppal")]
use rppal::gpio::Gpio;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;
use bytes::BytesMut;
#[cfg(feature = "ros")]
use std_msgs::msg::String as StringMsg;
use tokio::sync::{mpsc, Mutex};
use tokio::sync::mpsc::Receiver;

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
    motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    imu_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Imu>>,
    tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
    data: Arc<std::sync::Mutex<Option<StringMsg>>>,
    rx: Receiver<Telemetry>,
}

#[cfg(feature = "ros")]
impl MicrocatNode {
    fn new(context: &rclrs::Context, serial_stream: Arc<Mutex<Box<dyn SerialPort>>>, rx: Receiver<Telemetry>) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "microcat_node")?;
        let data = Arc::new(std::sync::Mutex::new(None));
        let data_cb = Arc::clone(&data);

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
                    let mut guard = serial_stream_cb.blocking_lock();
                    serial::write(&mut *guard, command);
                },
            )?
        };
        let motor_status_publisher = node.create_publisher("motor_status", rclrs::QOS_PROFILE_DEFAULT)?;
        let imu_publisher = node.create_publisher("imu", rclrs::QOS_PROFILE_DEFAULT)?;
        let tone_detector_publisher = node.create_publisher("tone_detector", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            motor_control_subscription,
            data,
            rx,
            motor_status_publisher,
            imu_publisher,
            tone_detector_publisher,
        })
    }

    async fn write(&mut self) {
        while let Some(msg) = self.rx.recv().await {
            match msg {
                Telemetry::MotorPosition(position) => {
                    self.motor_status_publisher.publish(position).unwrap()
                }
                Telemetry::Imu(imu) => {
                    self.imu_publisher.publish(imu).unwrap()
                }
                Telemetry::ToneDetector(tone_detector) => {
                    self.tone_detector_publisher.publish(tone_detector).unwrap()
                }
            }
        }
    }
}

enum Telemetry {
    MotorPosition(microcat_msgs::msg::MotorStatus),
    Imu(microcat_msgs::msg::Imu),
    ToneDetector(microcat_msgs::msg::ToneDetector),
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
    let (mut tx, rx) = mpsc::channel::<Telemetry>(10);
    
    #[cfg(feature = "ros")]
    let context = rclrs::Context::new(std::env::args())?;
    #[cfg(feature = "ros")]
    let microcat_node = Arc::new(Mutex::new(MicrocatNode::new(&context, Arc::clone(&serial), rx)?));
    #[cfg(feature = "ros")]
    let microcat_other_thread = Arc::clone(&microcat_node);
    #[cfg(feature = "ros")]
    tokio::spawn(async move {
       tokio::time::sleep(Duration::from_millis(10)).await;
        microcat_other_thread.lock().await.write().await;
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
    let mut initialized = false;

    for _ in 0..100_000 {
        let mut buf =  BytesMut::default();
        {
            let mut port_guard = serial.lock().await;
            serial::read(&mut *port_guard, &mut buf, &mut initialized,&mut tx).await;
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    #[cfg(feature = "ros")]
    rclrs::spin(Arc::clone(&microcat_node.blocking_lock().node))?;
    Ok(())
}
