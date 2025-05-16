use crate::serial::write;
use crate::serial::Command::TimeSync;
use tokio_serial::SerialStream;

pub async fn time_sync(serial: &mut SerialStream) -> i128 {
    write(serial, TimeSync).await;
    0
}
