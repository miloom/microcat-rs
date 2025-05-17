#![allow(unused_imports)]

use crate::serial::message::message::Data;
use crate::Telemetry::BatteryVoltage;
use crate::{Telemetry, TimingFrame};
use bytes::{Buf, BytesMut};
use microcat_msgs::msg::{MotorStatus, ToneDetector};
use prost::Message;
use std::time::UNIX_EPOCH;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc::Sender;
use tokio_serial::SerialStream;
use tracing::{debug, error, info, trace};
use tracing_subscriber::fmt::time;

#[rustfmt::skip]
mod imu;
#[allow(clippy::all, clippy::nursery, clippy::pedantic)]
#[rustfmt::skip]
mod message;
#[rustfmt::skip]
mod motor;
#[rustfmt::skip]
mod pressure;
#[rustfmt::skip]
mod tone_detector;
#[rustfmt::skip]
mod sync;


#[tracing::instrument(level = "trace", skip(serial, tx))]
pub async fn read(
    serial: &mut SerialStream,
    message_buffer: &mut BytesMut,
    initialized: &mut bool,
    tx: &mut Sender<crate::Telemetry>,
    timing_tx: &mut Sender<crate::TimingFrame>,
    time_offset_tx: &mut tokio::sync::watch::Sender<i64>,
    time_offset_rx: &mut tokio::sync::watch::Receiver<i64>,
) -> Result<(), std::io::Error> {
    let mut buf: [u8; 256] = [0; 256];
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
                debug!("COBS decoding");
                if len.is_err() {
                    return Ok(());
                }
                let len = len.unwrap();

                #[allow(unused_variables)]
                let message = &dest[..len];

                debug!("Protobuf message decoding");
                let decoded = message::Message::decode(message).inspect_err(|e| error!("{e}"));

                #[allow(unused_variables)]
                if let Ok(msg) = decoded {
                    trace!("Protobuf message decoded successfully {msg:?}");
                    match msg.data {
                        Some(message::message::Data::MotorPosition(msg)) => {
                            if let Ok(loc) = motor::Location::try_from(msg.location) {
                                debug!("Motor position");
                                let _ = match loc {
                                    motor::Location::FrontLeft => {
                                        tx.send(Telemetry::FLMotorPosition(MotorStatus {
                                            position: msg.position,
                                        }))
                                    }
                                    motor::Location::FrontRight => {
                                        tx.send(Telemetry::FRMotorPosition(MotorStatus {
                                            position: msg.position,
                                        }))
                                    }
                                    motor::Location::BackLeft => {
                                        tx.send(Telemetry::RLMotorPosition(MotorStatus {
                                            position: msg.position,
                                        }))
                                    }
                                    motor::Location::BackRight => {
                                        tx.send(Telemetry::RRMotorPosition(MotorStatus {
                                            position: msg.position,
                                        }))
                                    }
                                }
                                .await;
                            }
                        }
                        Some(Data::MotorTarget(_)) | None => {
                            debug!("Motor target");
                            return Ok(());
                        }
                        Some(Data::InitSync(_)) => {
                            debug!("Init sync");
                            return Ok(());
                        }
                        Some(Data::ResponseSync(msg)) => {
                            debug!("Response sync: {:?}", msg);
                            let offset = (msg.delay_ms
                                - (msg.t3_ms
                                    - std::time::SystemTime::now()
                                        .duration_since(UNIX_EPOCH)
                                        .unwrap()
                                        .as_millis() as i64))
                                / 2;
                            let _ = time_offset_tx.send(offset);
                            info!("Offset: {}", offset);
                        }
                        Some(Data::Time(time)) => {
                            debug!("Time: {:?} Offset: {}", time, *time_offset_rx.borrow());
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
    MotorTarget(MotorPos),
    TimeSync,
}

#[tracing::instrument(level = "trace", skip(serial))]
pub async fn write(
    serial: &mut SerialStream,
    command: Command,
    timing_tx: &mut Sender<crate::TimingFrame>,
    count: &mut u32,
) {
    debug!("Writing to serial");
    let message = match command {
        Command::MotorTarget(pos) => {
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
        Command::TimeSync => message::Message {
            data: Some(message::message::Data::InitSync(sync::Initialize {
                t1_ms: std::time::SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as i64,
            })),
        },
    };

    let mut buf = BytesMut::new();
    if message.encode(&mut buf).is_err() {
        error!("Failed to encode protobuf message");
        return;
    }

    let mut dest = [0u8; 128];
    let len = cobs::encode(buf.iter().as_slice(), &mut dest);
    dest[len] = 0;
    let full_len = len + 1;

    debug!("Sending serial timing {}", count);
    let _ = timing_tx.send(TimingFrame {
        timestamp: std::time::SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis(),
        frame_number: *count,
    });
    for byte in dest[..full_len].iter() {
        match serial.write_all(&[*byte]).await {
            Ok(_) => {
                serial.flush().await.unwrap(); // Ensure data hits the wire
                tokio::time::sleep(tokio::time::Duration::from_micros(3000)).await;
            }
            Err(e) => {
                error!("Failed to write to serial: {}", e);
            }
        }
    }
}
