#![allow(unused_imports)]
use crate::serial::message::message::Data;
use crate::Telemetry;
use crate::Telemetry::BatteryVoltage;
use bytes::BytesMut;
use prost::Message;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc::Sender;
use tokio_serial::SerialStream;
use tracing::{debug, error, trace};

mod imu;
#[allow(clippy::all, clippy::nursery, clippy::pedantic)]
mod message;
mod motor;
mod pressure;
mod tone_detector;

#[tracing::instrument(level = "trace", skip(serial, tx))]
pub async fn read(
    serial: &mut SerialStream,
    message_buffer: &mut BytesMut,
    initialized: &mut bool,
    tx: &mut Sender<crate::Telemetry>,
) -> Result<(), std::io::Error> {
    let mut buf: [u8; 100] = [0; 100];
    match serial.read(&mut buf).await {
        Ok(0) => {
            trace!("No data read");
        }
        Ok(n) => {
            let bytes: BytesMut = buf[0..n].iter().collect::<BytesMut>().freeze().into();
            if *initialized {
                message_buffer.extend(bytes);
            } else if let Some(start) = bytes.iter().rposition(|&v| v == 0) {
                message_buffer.extend(bytes[start + 1..].iter());
                *initialized = true;
            }

            while let Some(end) = message_buffer.iter().position(|&v| v == 0) {
                let message = message_buffer.split_to(end + 1);
                let mut dest = [0u8; 256];
                let len = cobs::decode(message.iter().as_slice(), &mut dest);
                if len.is_err() {
                    return Ok(());
                }
                let len = len.unwrap();

                #[allow(unused_variables)]
                let message = &dest[..len];

                let decoded = message::Message::decode(message).inspect_err(|e| println!("{e}"));

                #[allow(unused_variables)]
                if let Ok(msg) = decoded {
                    trace!("Protobuf message decoded successfully {msg:?}");
                    match msg.data {
                        Some(message::message::Data::Imu(msg)) => {
                            if let Some(gyro) = msg.gyro {
                                if let Some(accel) = msg.accel {
                                    let _ = tx
                                        .send(Telemetry::Imu(microcat_msgs::msg::Imu {
                                            gyro_x: gyro.x as f32 / 131.0,
                                            gyro_y: gyro.y as f32 / 131.0,
                                            gyro_z: gyro.z as f32 / 131.0,
                                            accel_x: accel.x as f32 / 100.0,
                                            accel_y: accel.y as f32 / 100.0,
                                            accel_z: accel.z as f32 / 100.0,
                                        }))
                                        .await;
                                }
                            }
                        }
                        Some(message::message::Data::MotorTarget(msg)) => {}
                        Some(message::message::Data::MotorPosition(msg)) => {
                            let _ = tx
                                .send(Telemetry::MotorPosition(microcat_msgs::msg::MotorStatus {
                                    location: if let Ok(loc) =
                                        motor::Location::try_from(msg.location)
                                    {
                                        match loc {
                                            motor::Location::FrontLeft => {
                                                microcat_msgs::msg::MotorStatus::FRONT_LEFT
                                            }
                                            motor::Location::FrontRight => {
                                                microcat_msgs::msg::MotorStatus::FRONT_RIGHT
                                            }
                                            motor::Location::BackLeft => {
                                                microcat_msgs::msg::MotorStatus::REAR_LEFT
                                            }
                                            motor::Location::BackRight => {
                                                microcat_msgs::msg::MotorStatus::REAR_RIGHT
                                            }
                                        }
                                    } else {
                                        255
                                    },
                                    position: msg.position,
                                }))
                                .await;
                        }
                        Some(message::message::Data::ToneDetectorStatus(msg)) => {
                            let _ = tx
                                .send(Telemetry::ToneDetector(microcat_msgs::msg::ToneDetector {
                                    location: if let Ok(loc) =
                                        tone_detector::Location::try_from(msg.location)
                                    {
                                        match loc {
                                            tone_detector::Location::Left => {
                                                microcat_msgs::msg::ToneDetector::LEFT
                                            }
                                            tone_detector::Location::Right => {
                                                microcat_msgs::msg::ToneDetector::RIGHT
                                            }
                                        }
                                    } else {
                                        255
                                    },
                                    is_active: msg.is_high,
                                }))
                                .await;
                        }
                        Some(message::message::Data::PressureData(msg)) => {
                            let _ = tx
                                .send(Telemetry::PressureData(microcat_msgs::msg::PressureData {
                                    pressure: msg.pressure as f32 / 100.0,
                                    temperature: msg.temperature as f32 / 100.0,
                                }))
                                .await;
                        }
                        Some(Data::BatterVoltage(voltage)) => {
                            let _ = tx
                                .send(BatteryVoltage(microcat_msgs::msg::Battery {
                                    battery_voltage: voltage,
                                }))
                                .await;
                        }
                        None => {
                            return Ok(());
                        }
                    }
                } else {
                    debug!("Failed to decode serial")
                }
            }
        }
        Err(e) => {
            debug!("Failed to read serial buffer: {}", e);
            return Err(e);
        }
    }
    Ok(())
}

#[derive(Debug)]
pub enum MotorLocation {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

#[derive(Debug)]
pub struct MotorPos {
    pub location: MotorLocation,
    pub amplitude: u32,
    pub target_position: i32,
    pub frequency: f32,
}

#[derive(Debug)]
pub enum Command {
    MotorPosition(MotorPos),
}

#[tracing::instrument(level = "trace", skip(serial))]
pub async fn write(serial: &mut SerialStream, command: Command) {
    trace!("Writing to serial");
    let message = match command {
        Command::MotorPosition(pos) => {
            let data = motor::MotorTarget {
                target_position: pos.target_position,
                amplitude: pos.amplitude,
                frequency: (pos.frequency * 1000.0) as u32,
                location: match pos.location {
                    MotorLocation::FrontLeft => motor::Location::FrontLeft.into(),
                    MotorLocation::FrontRight => motor::Location::FrontRight.into(),
                    MotorLocation::RearRight => motor::Location::BackRight.into(),
                    MotorLocation::RearLeft => motor::Location::BackLeft.into(),
                },
            };
            message::Message {
                data: Some(message::message::Data::MotorTarget(data)),
            }
        }
    };

    let mut buf = BytesMut::new();
    if message.encode(&mut buf).is_err() {
        error!("Failed to encode protobuf message");
        return;
    }
    let mut dest = [0u8; 128];
    let len = cobs::encode(buf.iter().as_slice(), &mut dest);
    dest[len] = 0;
    let _ = serial.write(&dest[..=len]).await;
}
