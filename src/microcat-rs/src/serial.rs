#![allow(unused_imports)]
use bytes::BytesMut;
use prost::Message;
use tokio_serial::SerialStream;
use anyhow::anyhow;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[cfg(feature = "proto")]
#[allow(clippy::all, clippy::nursery, clippy::pedantic)]
mod message;
#[cfg(feature = "proto")]
mod encoder;
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
            let bytes: BytesMut = buf[0..n].into_iter().collect::<BytesMut>().freeze().into();
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
                        message::message::Data::Encoder(msg) => {
                            if msg.position == encoder::Location::FrontRight.into() {
                                println!("{}", msg.position);
                            }
                        },
                        message::message::Data::Telemetry(msg) => {
                        }
                        message::message::Data::Motor(msg) => {

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

pub async fn send_motor_pos(serial: &mut SerialStream, target_position: f32, amplitude: f32, frequency: f32) {
    let motor = motor::MotorTarget {
        amplitude,
        frequency,
        target_position,
        location: motor::Location::FrontRight.into(),
    };
    let mut message = message::Message::default();
    message.data = Some(message::message::Data::Motor(motor));
    let mut buf = BytesMut::new();
    message.encode(&mut buf).unwrap();
    let mut dest = [0u8; 128];
    let len = cobs::encode(buf.iter().as_slice(), &mut dest);
    dest[len] = 0;
    let _ = serial.write(&dest[..=len]).await.unwrap();
}
