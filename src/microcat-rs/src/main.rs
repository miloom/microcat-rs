#![allow(dead_code)]
#![allow(unused_variables)]
use crate::rgb::Rgb;
use crate::serial::MotorPos;
use bytes::BytesMut;
use rclrs::CreateBasicExecutor;
use rppal::gpio::Gpio;
use std::error::Error;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio_serial::SerialPortBuilderExt;

#[allow(dead_code)]
mod consts;
mod imu;
mod motors;
mod rgb;
mod serial;

struct MicrocatNode {
    node: Arc<rclrs::Node>,
    motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    imu_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Imu>>,
    tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
    pressure_data_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::PressureData>>,
    rx: Receiver<Telemetry>,
}

impl MicrocatNode {
    fn new(
        executor: &rclrs::Executor,
        rgb: Rgb,
        rx: Receiver<Telemetry>,
        tx: Sender<serial::Command>,
    ) -> Result<Self, rclrs::RclrsError> {
        let node = executor.create_node("microcat")?;

        let motor_control_subscription = node.create_subscription(
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
                if let Err(error) = tx.try_send(command) {
                    // No room for message log the error
                }
            },
        )?;
        let motor_status_publisher =
            node.create_publisher("motor_status", rclrs::QOS_PROFILE_DEFAULT)?;
        let imu_publisher = node.create_publisher("imu", rclrs::QOS_PROFILE_DEFAULT)?;
        let tone_detector_publisher =
            node.create_publisher("tone_detector", rclrs::QOS_PROFILE_DEFAULT)?;
        let pressure_data_publisher =
            node.create_publisher("pressure_data", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            motor_control_subscription,
            rx,
            motor_status_publisher,
            imu_publisher,
            tone_detector_publisher,
            pressure_data_publisher,
        })
    }

    async fn write(&mut self) {
        while let Some(msg) = self.rx.recv().await {
            match msg {
                Telemetry::MotorPosition(position) => {
                    self.motor_status_publisher.publish(position).unwrap()
                }
                Telemetry::Imu(imu) => self.imu_publisher.publish(imu).unwrap(),
                Telemetry::ToneDetector(tone_detector) => {
                    self.tone_detector_publisher.publish(tone_detector).unwrap()
                }
                Telemetry::PressureData(pressure_data) => {
                    self.pressure_data_publisher.publish(pressure_data).unwrap()
                }
            }
        }
    }
}

enum Telemetry {
    MotorPosition(microcat_msgs::msg::MotorStatus),
    Imu(microcat_msgs::msg::Imu),
    ToneDetector(microcat_msgs::msg::ToneDetector),
    PressureData(microcat_msgs::msg::PressureData),
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let rgb = rgb::init_leds()?;

    let (mut telemetry_tx, telemetry_rx) = mpsc::channel::<Telemetry>(10);
    let (command_tx, mut command_rx) = mpsc::channel::<serial::Command>(10);
    let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
    let shutdown_signal_task = {
        let shutdown_tx = shutdown_tx.clone();
        tokio::spawn(async move {
            tokio::signal::ctrl_c()
                .await
                .expect("Failed to install Ctrl+C handler");
            let _ = shutdown_tx.send(true);
        })
    };

    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();

    let ros_task = {
        let mut shutdown_rx = shutdown_rx.clone();
        tokio::spawn(async move {
            let mut microcat_node = MicrocatNode::new(&executor, rgb, telemetry_rx, command_tx)
                .expect("Failed to create microcat node");
            loop {
                tokio::select! {
                    _ = shutdown_rx.changed() => {
                        println!("Shutting down ROS node...");
                        break;
                    }
                    _ = microcat_node.write() => {}
                }
            }
        })
    };

    let gpio = Gpio::new()?;
    // Setting pins 19 and 26 will configure the MUX to connect arduino to rpi
    let mut pin = gpio.get(19)?.into_output();
    pin.set_low();
    let mut new_pin = gpio.get(26)?.into_output();
    new_pin.set_low();
    println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

    let serial_task = {
        let mut shutdown_rx = shutdown_rx.clone();
        tokio::spawn(async move {
            let mut serial_buf = BytesMut::default();
            let mut initialized = false;
            let mut serial = tokio_serial::new("/dev/ttyAMA0", 115_200)
                .open_native_async()
                .expect("Failed to open serial port");

            loop {
                tokio::select! {
                     _ = serial::read(
                        &mut serial,
                        &mut serial_buf,
                        &mut initialized,
                        &mut telemetry_tx) => {}
                    Some(command) = command_rx.recv() => {
                        serial::write(&mut serial, command).await;
                    }
                    _ = shutdown_rx.changed() => {
                        println!("Shutting down Serial task...");
                        break;
                    }
                }
            }
        })
    };

    let (_, _, _) = tokio::join!(serial_task, ros_task, shutdown_signal_task);

    Ok(())
}
