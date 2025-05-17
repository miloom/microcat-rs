use crate::serial::write;
use crate::serial::Command::TimeSync;
use tokio_serial::SerialStream;
use tracing::info;

pub async fn time_sync(serial: &mut SerialStream) {
    info!("Time sync");
    // write(serial, TimeSync).await;
}
