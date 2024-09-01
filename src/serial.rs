use tokio_serial::SerialStream;
use prost::Message;

mod encoder;

pub async fn read_step(serial: &mut SerialStream, message_buffer: &mut bytes::BytesMut, initialized: &mut bool) {
    let mut buf = bytes::BytesMut::with_capacity(1024);
    match serial.try_read(&mut buf) {
        Ok(n) if n == 0 => {
            println!("No data read, returning");
            return;
        }
        Ok(_) => {
            let bytes = buf.split().freeze();
            if !*initialized {
                if let Some(start) = bytes.iter().rposition(|&v| v == 0) {
                    message_buffer.extend(bytes[start + 1..].iter());
                    *initialized = true;
                }
            } else {
                message_buffer.extend(bytes);
            }

            while let Some(end) = message_buffer.iter().position(|&v| v == 0) {
                let message = message_buffer.split_to(end + 1);
                let mut dest = [0u8; 256];
                let len = cobs::decode(
                    message.iter().as_slice().try_into().expect("Expected to be able to convert to array"), &mut dest,
                ).expect("Expected to be able to decode COBS packet");

                let message = &dest[..len];

                let decoded = encoder::EncoderData::decode(message).inspect_err(|e| println!("{e}"));
                if let Ok(msg) = decoded {
                    if msg.location == encoder::Location::FrontRight.into() {
                        println!("{}", msg.position);
                    }
                } else {
                    println!("Failed to decode");
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to read from serial port: {:?}", e);
        }
    }
}