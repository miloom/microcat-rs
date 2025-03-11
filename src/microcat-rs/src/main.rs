#![allow(dead_code)]
#![allow(unused_variables)]
use crate::consts::MAIN_LOOP_TIME_PERIOD_MS;
use crate::serial::MotorPos;
use bytes::BytesMut;
use rclrs::CreateBasicExecutor;
use rppal::gpio::Gpio;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;
use std_msgs::msg::String as StringMsg;
use tokio::sync::mpsc::Receiver;
use tokio::sync::{mpsc, Mutex};

#[allow(dead_code)]
mod consts;
mod imu;
// #[allow(dead_code), cfg(feature = "rppal")]
mod motors;
// #[allow(dead_code), cfg(feature="rppal")]
mod rgb;
mod serial;

struct MicrocatNode {
    node: Arc<rclrs::Node>,
    motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    imu_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Imu>>,
    tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
    pressure_data_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::PressureData>>,
    data: Arc<std::sync::Mutex<Option<StringMsg>>>,
    rx: Receiver<Telemetry>,
}

impl MicrocatNode {
    fn new(
        executor: &rclrs::Executor,
        serial_stream: Arc<Mutex<Box<dyn SerialPort>>>,
        rx: Receiver<Telemetry>,
    ) -> Result<Self, rclrs::RclrsError> {
        let node = executor.create_node("microcat")?;
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
            data,
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
    let serial = Arc::new(Mutex::new(
        serialport::new("/dev/ttyS0", 115_200)
            .timeout(Duration::from_millis(100))
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .open()
            .unwrap(),
    ));
    let (mut tx, rx) = mpsc::channel::<Telemetry>(10);

    let context = rclrs::Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let microcat_node = Arc::new(Mutex::new(MicrocatNode::new(
        &executor,
        Arc::clone(&serial),
        rx,
    )?));
    let microcat_other_thread = Arc::clone(&microcat_node);
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(10)).await;
        microcat_other_thread.lock().await.write().await;
    });

    /*
    let mut rgb = rgb::init_leds()?;

    tokio::task::spawn(async move {
        rgb::test_leds(&mut rgb).await.unwrap();
    });*/

    let gpio = Gpio::new()?;
    // Setting pins 19 and 26 will configure the MUX to connect arduino to rpi
    let mut pin = gpio.get(19)?.into_output();
    pin.set_low();
    let mut new_pin = gpio.get(26)?.into_output();
    new_pin.set_low();
    println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

    let message_buffer = bytes::BytesMut::with_capacity(1024);
    let mut initialized = false;

    let next_loop_exec = tokio::time::Instant::now()
        .checked_add(Duration::from_millis(MAIN_LOOP_TIME_PERIOD_MS as u64))
        .unwrap();
    let mut serial_buf = BytesMut::default();

    while context.ok() {
        while tokio::time::Instant::now() < next_loop_exec {}
        {
            let mut port_guard = serial.lock().await;
            serial::read(&mut *port_guard, &mut serial_buf, &mut initialized, &mut tx).await;
        }

        next_loop_exec
            .checked_add(Duration::from_millis(MAIN_LOOP_TIME_PERIOD_MS as u64))
            .unwrap();
    }
    Ok(())
}
