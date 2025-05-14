use crate::rgb::Rgb;
use crate::serial::MotorPos;
use bytes::BytesMut;
use rclrs::{CreateBasicExecutor, PublisherOptions, QOS_PROFILE_DEFAULT, QOS_PROFILE_SENSOR_DATA};
use rppal::gpio::Gpio;
use std::error::Error;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio_serial::SerialPortBuilderExt;
use tracing::{debug, error, info, span, trace};
use tracing_appender::rolling;
use tracing_subscriber::EnvFilter;

mod camera;
#[allow(dead_code)]
mod consts;
mod rgb;
mod serial;

struct MicrocatNode {
    _node: Arc<rclrs::Node>,
    _motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    _rgb_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::Led>>,
    motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    imu_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Imu>>,
    tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
    pressure_data_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::PressureData>>,
    camera_image_publisher: Arc<rclrs::Publisher<sensor_msgs::msg::Image>>,
    battery_data_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Battery>>,
    rx: Receiver<Telemetry>,
}

impl MicrocatNode {
    fn new(
        executor: &rclrs::Executor,
        mut rgb: Rgb,
        rx: Receiver<Telemetry>,
        tx: Sender<serial::Command>,
    ) -> Result<Self, rclrs::RclrsError> {
        info!("Creating microcat node");
        let node = executor.create_node("microcat")?;

        let _motor_control_subscription = node.create_subscription(
            "motor_control",
            move |msg: microcat_msgs::msg::MotorControl| {
                let my_span = span!(tracing::Level::INFO, "motor_control");
                let _enter = my_span.enter();
                trace!("Received motor_control msg {msg:?}");

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
                    error!("Failed to send command {error:?}");
                }
            },
        )?;

        let _rgb_subscription =
            node.create_subscription("rgb_led", move |msg: microcat_msgs::msg::Led| {
                let my_span = span!(tracing::Level::INFO, "rgb control");
                let _enter = my_span.enter();
                trace!("Received rgb_led msg {msg:?}");
                rgb.set_color(
                    msg.red.clamp(0.0, 100.0),
                    msg.green.clamp(0.0, 100.0),
                    msg.blue.clamp(0.0, 100.0),
                );
            })?;

        let mut options = PublisherOptions::new("motor_status");
        options.qos = QOS_PROFILE_DEFAULT;
        let motor_status_publisher = node.create_publisher(options)?;
        let mut options = PublisherOptions::new("imu/data");
        options.qos = QOS_PROFILE_DEFAULT;
        let imu_publisher = node.create_publisher(options)?;
        let mut options = PublisherOptions::new("tone_detector/data");
        options.qos = QOS_PROFILE_DEFAULT;
        let tone_detector_publisher = node.create_publisher(options)?;
        let mut options = PublisherOptions::new("pressure/data");
        options.qos = QOS_PROFILE_DEFAULT;
        let pressure_data_publisher = node.create_publisher(options)?;
        let mut options = PublisherOptions::new("camera/image_raw");
        options.qos = QOS_PROFILE_DEFAULT;
        let camera_image_publisher = node.create_publisher(options)?;
        let mut options = PublisherOptions::new("battery/voltage");
        options.qos = QOS_PROFILE_DEFAULT;
        let battery_data_publisher = node.create_publisher(options)?;
        Ok(Self {
            _node: node,
            _motor_control_subscription,
            _rgb_subscription,
            rx,
            motor_status_publisher,
            imu_publisher,
            tone_detector_publisher,
            pressure_data_publisher,
            camera_image_publisher,
            battery_data_publisher,
        })
    }

    #[tracing::instrument(level = "trace", skip(self))]
    async fn write(&mut self) {
        trace!("{} messages in queue", self.rx.len());
        while let Some(msg) = self.rx.recv().await {
            match msg {
                Telemetry::MotorPosition(position) => {
                    trace!("Sending motor_position msg {position:?}");
                    let _ = self.motor_status_publisher.publish(position);
                }
                Telemetry::Imu(imu) => {
                    trace!("Sending imu msg {imu:?}");
                    let _ = self.imu_publisher.publish(imu);
                }
                Telemetry::ToneDetector(tone_detector) => {
                    trace!("Sending tone_detector msg {tone_detector:?}");
                    let _ = self.tone_detector_publisher.publish(tone_detector);
                }
                Telemetry::PressureData(pressure_data) => {
                    trace!("Sending pressure_data msg {pressure_data:?}");
                    let _ = self.pressure_data_publisher.publish(pressure_data);
                }
                Telemetry::CameraData(image) => {
                    trace!("Sending camera_data msg");
                    let _ = self.camera_image_publisher.publish(image);
                }
                Telemetry::BatteryVoltage(battery_data) => {
                    trace!("Sending battery_voltage msg {battery_data:?}");
                    let _ = self.battery_data_publisher.publish(battery_data);
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
    CameraData(sensor_msgs::msg::Image),
    BatteryVoltage(microcat_msgs::msg::Battery),
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let home_dir = std::env::var("HOME").unwrap();
    let log_path = format!("{}/.microcat/logs", home_dir);
    std::fs::create_dir_all(&log_path).unwrap();
    let file_appender = rolling::daily(log_path, "microcat_log");
    let (non_blocking, _guard) = tracing_appender::non_blocking(file_appender);
    println!("Starting logger");
    tracing_subscriber::fmt()
        .with_writer(non_blocking)
        .with_env_filter(EnvFilter::from_default_env())
        .init();
    info!("Starting microcat");

    let rgb = rgb::Rgb::init_leds()?;
    info!("LED initialized");

    let (telemetry_tx, telemetry_rx) = mpsc::channel::<Telemetry>(10);
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
    info!("Channels created!");

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
                        info!("Shutting down ROS node...");
                        break;
                    }
                    _ = microcat_node.write() => {}
                }
            }
        })
    };

    debug!("Acquiring GPIO");
    let gpio = Gpio::new()?;
    info!("Setting up UART mux");
    #[cfg(feature = "v21_hardware")]
    let (_, _) = {
        // Setting pins 19 and 26 will configure the MUX to connect arduino to rpi
        let mut mux_a = gpio.get(19)?.into_output();
        mux_a.set_low();
        let mut mux_b = gpio.get(26)?.into_output();
        mux_b.set_low();
        (mux_a, mux_b)
    };

    #[cfg(feature = "v26_hardware")]
    let (_, _, _) = {
        let mut mux_enable_pin = gpio.get(22)?.into_output();
        mux_enable_pin.set_high(); // Disable mux to configure
        let mut mux_a = gpio.get(24)?.into_output();
        let mut mux_b = gpio.get(25)?.into_output();
        // Configure mux for RPI to Atmega connection
        mux_a.set_low();
        mux_b.set_low();
        // Enable mux after configuring
        mux_enable_pin.set_low();
        (mux_a, mux_b, mux_enable_pin)
    };

    let mut serial_telemetry_tx = telemetry_tx.clone();
    let serial_task = {
        let mut shutdown_rx = shutdown_rx.clone();
        tokio::spawn(async move {
            let mut serial_buf = BytesMut::default();
            let mut initialized = false;
            let mut serial = tokio_serial::new("/dev/ttyAMA0", 115_200)
                .open_native_async()
                .expect("Failed to open serial port");

            info!("{:?}", serial);

            loop {
                tokio::select! {
                     _ = serial::read(
                        &mut serial,
                        &mut serial_buf,
                        &mut initialized,
                        &mut serial_telemetry_tx) => {}
                    Some(command) = command_rx.recv() => {
                        serial::write(&mut serial, command).await;
                    }
                    _ = shutdown_rx.changed() => {
                        println!("Shutting down Serial task...");
                        info!("Shutting down Serial task...");
                        break;
                    }
                }
            }
        })
    };

    {
        let camera_telemetry_tx = telemetry_tx.clone();
        let shutdown_rx = shutdown_rx.clone();
        crate::camera::run_camera(camera_telemetry_tx, shutdown_rx)
    };

    let (_, _, _) = tokio::join!(serial_task, ros_task, shutdown_signal_task,);

    Ok(())
}
