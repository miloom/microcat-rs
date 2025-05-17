use crate::rgb::Rgb;
use crate::serial::{MotorLocation, MotorPos};
use bytes::BytesMut;
use rclrs::CreateBasicExecutor;
use rppal::gpio::Gpio;
use std::collections::HashMap;
use std::error::Error;
use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::UNIX_EPOCH;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio_serial::{DataBits, FlowControl, Parity, SerialPortBuilderExt, StopBits};
use tracing::{debug, error, info, trace};
use tracing_appender::rolling;
use tracing_subscriber::EnvFilter;

mod camera;
#[allow(dead_code)]
mod consts;
mod rgb;
mod serial;
mod time_sync;

struct MicrocatNode {
    _node: Arc<rclrs::Node>,
    _fl_motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    _fr_motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    _rl_motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    _rr_motor_control_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::MotorControl>>,
    _rgb_subscription: Arc<rclrs::Subscription<microcat_msgs::msg::Led>>,
    fl_motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    fr_motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    rl_motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    rr_motor_status_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::MotorStatus>>,
    imu_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::Imu>>,
    left_tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
    right_tone_detector_publisher: Arc<rclrs::Publisher<microcat_msgs::msg::ToneDetector>>,
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
        timing_tx: Sender<TimingFrame>,
    ) -> Result<Self, rclrs::RclrsError> {
        info!("Creating microcat node");
        let node = executor.create_node("microcat")?;

        let timing_counter = Arc::new(AtomicU32::new(0));
        let tx_clone = tx.clone();
        let _fl_motor_control_subscription = node
            .create_subscription::<microcat_msgs::msg::MotorControl, _>(
                "/motor/front_left/control",
                move |msg: microcat_msgs::msg::MotorControl| {
                    trace!("Received front_left motor_control msg {msg:?}");

                    let _ = timing_tx.blocking_send(TimingFrame {
                        timestamp: std::time::SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap()
                            .as_millis(),
                        frame_number: timing_counter.load(Ordering::Relaxed),
                    });
                    debug!("Took timestamp {}", timing_counter.load(Ordering::Relaxed));
                    timing_counter.fetch_add(1, Ordering::Relaxed);

                    let command = serial::Command::MotorTarget(MotorPos {
                        location: MotorLocation::FrontLeft,
                        target_position: msg.position,
                        frequency: msg.frequency,
                        amplitude: msg.amplitude,
                    });
                    if let Err(error) = tx_clone.try_send(command) {
                        error!("Failed to send command {error:?}");
                    }
                },
            )?;
        let tx_clone = tx.clone();
        let _fr_motor_control_subscription = node
            .create_subscription::<microcat_msgs::msg::MotorControl, _>(
                "/motor/front_right/control",
                move |msg: microcat_msgs::msg::MotorControl| {
                    trace!("Received front_right motor_control msg {msg:?}");

                    let command = serial::Command::MotorTarget(MotorPos {
                        location: MotorLocation::FrontRight,
                        target_position: msg.position,
                        frequency: msg.frequency,
                        amplitude: msg.amplitude,
                    });
                    if let Err(error) = tx_clone.try_send(command) {
                        error!("Failed to send command {error:?}");
                    }
                },
            )?;
        let tx_clone = tx.clone();
        let _rl_motor_control_subscription = node
            .create_subscription::<microcat_msgs::msg::MotorControl, _>(
                "/motor/rear_left/control",
                move |msg: microcat_msgs::msg::MotorControl| {
                    trace!("Received rear_left motor_control msg {msg:?}");

                    let command = serial::Command::MotorTarget(MotorPos {
                        location: MotorLocation::RearLeft,
                        target_position: msg.position,
                        frequency: msg.frequency,
                        amplitude: msg.amplitude,
                    });
                    if let Err(error) = tx_clone.try_send(command) {
                        error!("Failed to send command {error:?}");
                    }
                },
            )?;
        let tx_clone = tx.clone();
        let _rr_motor_control_subscription = node
            .create_subscription::<microcat_msgs::msg::MotorControl, _>(
                "/motor/rear_right/control",
                move |msg: microcat_msgs::msg::MotorControl| {
                    debug!("Received rear_right motor_control msg {msg:?}");

                    let command = serial::Command::MotorTarget(MotorPos {
                        location: MotorLocation::RearRight,
                        target_position: msg.position,
                        frequency: msg.frequency,
                        amplitude: msg.amplitude,
                    });
                    if let Err(error) = tx_clone.try_send(command) {
                        error!("Failed to send command {error:?}");
                    }
                },
            )?;

        let _rgb_subscription = node.create_subscription::<microcat_msgs::msg::Led, _>(
            "/led",
            move |msg: microcat_msgs::msg::Led| {
                debug!("Received led msg {msg:?}");
                rgb.set_color(
                    msg.red.clamp(0.0, 100.0),
                    msg.green.clamp(0.0, 100.0),
                    msg.blue.clamp(0.0, 100.0),
                );
            },
        )?;

        let fl_motor_status_publisher = node.create_publisher("motor/front_left/status")?;
        let fr_motor_status_publisher = node.create_publisher("motor/front_right/status")?;
        let rl_motor_status_publisher = node.create_publisher("motor/rear_left/status")?;
        let rr_motor_status_publisher = node.create_publisher("motor/rear_right/status")?;
        let imu_publisher = node.create_publisher("imu/data")?;
        let left_tone_detector_publisher = node.create_publisher("tone_detector/left/data")?;
        let right_tone_detector_publisher = node.create_publisher("tone_detector/right/data")?;
        let pressure_data_publisher = node.create_publisher("pressure/data")?;
        let camera_image_publisher = node.create_publisher("camera/image_raw")?;
        let battery_data_publisher = node.create_publisher("battery/voltage")?;
        Ok(Self {
            _node: node,
            _fl_motor_control_subscription,
            _fr_motor_control_subscription,
            _rl_motor_control_subscription,
            _rr_motor_control_subscription,
            _rgb_subscription,
            rx,
            fl_motor_status_publisher,
            fr_motor_status_publisher,
            rl_motor_status_publisher,
            rr_motor_status_publisher,
            imu_publisher,
            left_tone_detector_publisher,
            right_tone_detector_publisher,
            pressure_data_publisher,
            camera_image_publisher,
            battery_data_publisher,
        })
    }

    #[tracing::instrument(level = "trace", skip(self))]
    async fn write(&mut self) {
        while let Some(msg) = self.rx.recv().await {
            match msg {
                Telemetry::FLMotorPosition(position) => {
                    trace!("Sending front left motor_position msg {position:?}");
                    let _ = self.fl_motor_status_publisher.publish(position);
                }
                Telemetry::FRMotorPosition(position) => {
                    trace!("Sending front right motor_position msg {position:?}");
                    let _ = self.fr_motor_status_publisher.publish(position);
                }
                Telemetry::RLMotorPosition(position) => {
                    trace!("Sending rear left motor_position msg {position:?}");
                    let _ = self.rl_motor_status_publisher.publish(position);
                }
                Telemetry::RRMotorPosition(position) => {
                    trace!("Sending rear right motor_position msg {position:?}");
                    let _ = self.rr_motor_status_publisher.publish(position);
                }
                Telemetry::Imu(imu) => {
                    trace!("Sending imu msg {imu:?}");
                    let _ = self.imu_publisher.publish(imu);
                }
                Telemetry::LeftToneDetector(tone_detector) => {
                    trace!("Sending left tone_detector msg {tone_detector:?}");
                    let _ = self.left_tone_detector_publisher.publish(tone_detector);
                }
                Telemetry::RightToneDetector(tone_detector) => {
                    trace!("Sending right tone_detector msg {tone_detector:?}");
                    let _ = self.right_tone_detector_publisher.publish(tone_detector);
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
    FLMotorPosition(microcat_msgs::msg::MotorStatus),
    FRMotorPosition(microcat_msgs::msg::MotorStatus),
    RLMotorPosition(microcat_msgs::msg::MotorStatus),
    RRMotorPosition(microcat_msgs::msg::MotorStatus),
    Imu(microcat_msgs::msg::Imu),
    LeftToneDetector(microcat_msgs::msg::ToneDetector),
    RightToneDetector(microcat_msgs::msg::ToneDetector),
    PressureData(microcat_msgs::msg::PressureData),
    CameraData(sensor_msgs::msg::Image),
    BatteryVoltage(microcat_msgs::msg::Battery),
}

struct TimingFrame {
    timestamp: u128,
    frame_number: u32,
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
    let (timing_tx, mut timing_rx) = mpsc::channel::<TimingFrame>(10);
    let (time_offset_tx, time_offset_rx) = tokio::sync::watch::channel(0i64);
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
    let mut executor = context.create_basic_executor();
    let ros_task = {
        let command_tx = command_tx.clone();
        let timing_tx = timing_tx.clone();

        let mut shutdown_rx = shutdown_rx.clone();
        tokio::spawn(async move {
            let mut microcat_node =
                MicrocatNode::new(&executor, rgb, telemetry_rx, command_tx, timing_tx)
                    .expect("Failed to create microcat node");

            let _ = std::thread::spawn(move || {
                executor.spin(Default::default());
            });

            loop {
                tokio::select! {
                    _ = shutdown_rx.changed() => {
                        println!("Shutting down ROS node...");
                        info!("Shutting down ROS node...");
                        break;
                    }
                    () = microcat_node.write() => {
                        continue;
                    }
                }
            }
            info!("Stopped ROS executor");
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
        let mut timing_tx = timing_tx.clone();
        let mut time_offset_tx = time_offset_tx.clone();
        let mut time_offset_rx = time_offset_rx.clone();
        tokio::spawn(async move {
            let mut serial_buf = BytesMut::default();
            let mut initialized = false;
            let mut serial = tokio_serial::new("/dev/ttyAMA0", 115_200)
                .data_bits(DataBits::Eight)
                .flow_control(FlowControl::None)
                .parity(Parity::None)
                .stop_bits(StopBits::One)
                .open_native_async()
                .expect("Failed to open serial port");

            info!("{:?}", serial);

            time_sync::time_sync(&mut serial).await;
            let mut count = 1;

            loop {
                tokio::select! {
                     _ = serial::read(
                        &mut serial,
                        &mut serial_buf,
                        &mut initialized,
                        &mut serial_telemetry_tx,
                        &mut timing_tx,
                        &mut time_offset_tx,
                        &mut time_offset_rx) => {
                    }
                     val = command_rx.recv() => {
                        if let Some(command) = val {
                            debug!("Serial got command {command:?}");
                            serial::write(&mut serial, command, &timing_tx, &mut count).await;
                        }
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

    let timing_task = {
        let mut shutdown_rx = shutdown_rx.clone();

        tokio::task::spawn(async move {
            let mut measurements: HashMap<u32, Vec<u128>> = HashMap::new();
            loop {
                tokio::select! {
                    Some(timing) = timing_rx.recv() => {
                        debug!("Got timing for {}", timing.frame_number);
                        measurements.entry(timing.frame_number).or_insert_with(Vec::new).push(timing.timestamp);
                    }
                    _ = shutdown_rx.changed() => {
                        info!("Calculating timing data...");
                        let mut file = std::fs::OpenOptions::new().write(true).append(true).create(true).open(format!("{}/timing_data.csv", home_dir)).unwrap();
                        for (nr, measurements) in measurements.iter() {
                            info!("Nr {nr} measurements: {measurements:?}");
                            write!(file, "{nr},", nr = nr,).unwrap();

                            for measurement in measurements {
                                write!(file, "{measurement},", measurement = measurement).unwrap();
                            }
                            writeln!(file, "").unwrap();
                        }


                        println!("Shutting down timing task...");
                        info!("Shutting down timing task...");
                        break;
                    }
                }
            }
        })
    };

    let camera_handle = {
        let camera_telemetry_tx = telemetry_tx.clone();
        let shutdown_rx = shutdown_rx.clone();
        camera::run_camera(camera_telemetry_tx, shutdown_rx)
    };

    let (_, _, _, _) = tokio::join!(serial_task, ros_task, shutdown_signal_task, timing_task);

    tokio::task::spawn_blocking(move || {
        camera_handle.join().expect("Failed to join camera task");
    })
    .await
    .expect("Join task failed");

    info!("Done!");

    Ok(())
}
