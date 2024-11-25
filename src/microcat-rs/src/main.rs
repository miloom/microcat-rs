#[cfg(feature = "rppal")]
use rppal::gpio::Gpio;
#[cfg(feature = "rppal")]
use rppal::i2c::I2c;
use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
#[cfg(feature = "ros")]
use std_msgs::msg::String as StringMsg;
use tokio::io::AsyncReadExt;
use tokio::signal;
use tokio::sync::Mutex;
use tokio_serial::{DataBits, FlowControl, Parity, SerialPortBuilderExt, SerialStream, StopBits};

#[allow(dead_code)]
mod consts;
#[cfg(feature = "rppal")]
mod imu;
// #[allow(dead_code), cfg(feature = "rppal")]
#[cfg(feature = "rppal")]
mod motors;
// #[allow(dead_code), cfg(feature="rppal")]
#[cfg(feature = "rppal")]
mod rgb;
#[cfg(feature = "proto")]
mod serial;

#[cfg(feature = "ros")]
struct MicrocatNode {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    publisher: Arc<rclrs::Publisher<StringMsg>>,
    data: Arc<std::sync::Mutex<Option<StringMsg>>>,
}

#[cfg(feature = "ros")]
impl MicrocatNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "microcat_node")?;
        let data = Arc::new(std::sync::Mutex::new(None));
        let data_cb = Arc::clone(&data);

        let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
        let _subscription = {
            node.create_subscription(
                "my_topic_in",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_cb.lock().unwrap() = Some(msg);
                },
            )?
        };
        Ok(Self {
            node,
            _subscription,
            publisher,
            data,
        })
    }

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }
        Ok(())
    }
}


#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());

    #[cfg(feature = "ros")]
    let context = rclrs::Context::new(std::env::args())?;
    #[cfg(feature = "ros")]
    let microcat_node = Arc::new(MicrocatNode::new(&context)?);
    #[cfg(feature = "ros")]
    let microcat_other_thread = Arc::clone(&microcat_node);
    #[cfg(feature = "ros")]
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            microcat_other_thread.republish()?;
        }
    });
    #[cfg(feature = "proto")]
    let mut serial = Arc::new(Mutex::new({
        println!("Using Proto");
        tokio_serial::new("/dev/ttyS0", 115_200)
            .timeout(std::time::Duration::from_secs(2))
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .open_native_async()?
    }));
    /*
    let mut rgb = rgb::init_leds()?;

    tokio::task::spawn(async move {
        rgb::test_leds(&mut rgb).await.unwrap();
    });*/


        let gpio = Gpio::new()?;
        let mut pin = gpio.get(19)?.into_output();
        pin.set_low();
        let mut new_pin = gpio.get(26)?.into_output();
        new_pin.set_low();
        println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

        let i2c = I2c::new()?;
        println!("I2c: {:?}", i2c.capabilities());

        println!("Motors test");
        let i2c = Arc::from(std::sync::Mutex::from(i2c));
        // motors::test(i2c.clone()).await?;

        println!("IMU");
        imu::setup(&mut i2c.lock().unwrap())?;

    let mut message_buffer = bytes::BytesMut::with_capacity(1024);
    let mut initialized = false;

    for _ in 0..100_000 {
        // let accel = imu::get_accel(&mut i2c.lock().unwrap())?;
        // println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        // serial::read_step(&mut serial, &mut message_buffer, &mut initialized).await;
        let mut buf = [0u8; 128];
        {
            let mut port_guard = serial.lock().await;
            if let Ok(size) = port_guard.try_read(&mut buf) {
                println!("{}", buf.iter().map(|&b| b as char).collect::<String>());
            } else {
                serial::send_motor_pos(&mut port_guard, 10.0, 20.0, 30.0).await;
                println!("Sending motor pos");
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    #[cfg(feature = "ros")]
    rclrs::spin(Arc::clone(&microcat_node.node))?;
    Ok(())
}
