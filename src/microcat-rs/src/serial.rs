#![allow(unused_imports)]
use crate::Telemetry;
use bytes::BytesMut;
use prost::Message;
use serialport::SerialPort;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc::Sender;

#[cfg(feature = "proto")]
#[allow(clippy::all, clippy::nursery, clippy::pedantic)]
mod message;
#[cfg(feature = "proto")]
mod tone_detector;
#[cfg(feature = "proto")]
mod imu;
#[cfg(feature = "proto")]
mod motor;
pub async fn read(
    // serial: &mut rppal::uart::Uart,
    serial: &mut Box<dyn SerialPort>,
    message_buffer: &mut BytesMut,
    initialized: &mut bool,
    tx: &mut Sender<crate::Telemetry>
) {
    // let mut buf = bytes::BytesMut::with_capacity(1024);
    let mut buf: [u8; 100] = [0; 100];
    match serial.read(&mut buf) {
        Ok(0) => {
            println!("No data read, returning");
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
                //println!("Message {message:?}");
                let mut dest = [0u8; 256];
                let len = cobs::decode(message.iter().as_slice(), &mut dest)
                    .expect("Expected to be able to decode COBS packet");

                #[allow(unused_variables)]
                let message = &dest[..len];

                #[cfg(feature = "proto")]
                let decoded =
                    message::Message::decode(message).inspect_err(|e| println!("{e}"));
                #[cfg(not(feature = "proto"))]
                let decoded: Result<(), anyhow::Error> = Ok(());

                #[allow(unused_variables)]
                if let Ok(msg) = decoded {

                    #[cfg(feature = "proto")]
                    match msg.data.unwrap() {
                        message::message::Data::Imu(msg) => {
                            if let Some(gyro) = msg.gyro {
                                if let Some(accel) = msg.accel {
                                    tx.send(Telemetry::Imu(microcat_msgs::msg::Imu {
                                        gyro_x: gyro.x as f32 / 1000.0,
                                        gyro_y: gyro.y as f32 / 1000.0,
                                        gyro_z: gyro.z as f32 / 1000.0,
                                        accel_x: accel.x,
                                        accel_y: accel.y,
                                        accel_z: accel.z,
                                    })).await.unwrap();
                                }
                            }
                        }
                        message::message::Data::MotorTarget(msg) => {

                        }
                        message::message::Data::MotorPosition(msg) => {
                            tx.send(Telemetry::MotorPosition(microcat_msgs::msg::MotorStatus {
                                location: if let Ok(loc) = motor::Location::try_from(msg.location) {
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
                            })).await.unwrap();
                        }
                        message::message::Data::ToneDetectorStatus(msg) => {
                            tx.send(Telemetry::ToneDetector(microcat_msgs::msg::ToneDetector {
                                location: if let Ok(loc) = tone_detector::Location::try_from(msg.location) {
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
                                is_active: msg.is_high
                            })).await.unwrap();
                        }
                    }
                } else {
                    eprintln!("Failed to decode");
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to read from serial port: {e:?}");
        }
    }
}

pub enum MotorLocation {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

pub struct MotorPos {
    pub location: MotorLocation,
    pub amplitude: u32,
    pub target_position: i32,
    pub frequency: f32,
}

pub enum Command {
    MotorPosition(MotorPos),
}

pub fn write(serial: &mut Box<dyn SerialPort>, command: Command) {
    let message = match command {
        Command::MotorPosition(pos) => {
            let data = motor::MotorTarget {
                target_position: pos.target_position,
                amplitude: pos.amplitude,
                frequency: (pos.frequency * 1000.0) as u32,
                location: match pos.location {
                    MotorLocation::FrontLeft => {
                        motor::Location::FrontLeft.into()
                    }
                    MotorLocation::FrontRight => {
                        motor::Location::FrontRight.into()
                    }
                    MotorLocation::RearRight => {
                        motor::Location::BackRight.into()
                    }
                    MotorLocation::RearLeft => {
                        motor::Location::BackLeft.into()
                    }
                },
            };
            message::Message {
                data: Some(message::message::Data::MotorTarget(data)),
            }
        }
    };

    let mut buf = BytesMut::new();
    message.encode(&mut buf).unwrap();
    let mut dest = [0u8; 128];
    let len = cobs::encode(buf.iter().as_slice(), &mut dest);
    dest[len] = 0;
    let _ = serial.write(&dest[..=len]);
}