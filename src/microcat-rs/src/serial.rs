#![allow(unused_imports)]
use bytes::BytesMut;
use prost::Message;
use serialport::SerialPort;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[cfg(feature = "proto")]
#[allow(clippy::all, clippy::nursery, clippy::pedantic)]
mod message;
#[cfg(feature = "proto")]
mod tone_detector;
#[cfg(feature = "proto")]
mod imu;
#[cfg(feature = "proto")]
mod motor;
pub async fn read_step(
    // serial: &mut rppal::uart::Uart,
    serial: &mut tokio_serial::SerialStream, 
    message_buffer: &mut bytes::BytesMut,
    initialized: &mut bool,
) {
    // let mut buf = bytes::BytesMut::with_capacity(1024);
    let mut buf: [u8; 100] = [0; 100];
    match serial.read(&mut buf).await {
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
                        }
                        message::message::Data::MotorTarget(msg) => {

                        }
                        message::message::Data::MotorPosition(msg) => {}
                        message::message::Data::ToneDetectorStatus(msg) => {}
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