use std::error::Error;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

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
#[cfg(feature = "rppal")]
mod serial;

struct MicrocatNode {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    publisher: Arc<rclrs::Publisher<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl MicrocatNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "microcat_node")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);

        let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
        let _subscription = {
            node.create_subscription(
                "my_topic_in",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_cb.lock().unwrap() = Some(msg);
                }
            )?
        };
        Ok(Self{
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


    let context = rclrs::Context::new(std::env::args())?;
    let microcat_node = Arc::new(MicrocatNode::new(&context)?);
    let microcat_other_thread = Arc::clone(&microcat_node);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            microcat_other_thread.republish()?;
        }
    });

    #[cfg(feature = "proto")]
    println!("Using Proto");

    /*
    let mut rgb = rgb::init_leds()?;
    
    tokio::task::spawn(async move {
        rgb::test_leds(&mut rgb).await.unwrap();
    });*/

    #[cfg(feature = "rppal")]
    {
    let gpio = Gpio::new()?;
    let mut pin = gpio.get(19)?.into_output();
    pin.set_low();
    let mut new_pin = gpio.get(26)?.into_output();
    new_pin.set_low();
    println!("{}, {}", pin.is_set_low(), new_pin.is_set_low());

    let i2c = I2c::new()?;
    println!("I2c: {:?}", i2c.capabilities());

    println!("Motors test");
    let i2c = Arc::from(Mutex::from(i2c));
    // motors::test(i2c.clone()).await?;

    println!("IMU");
    imu::setup(&mut i2c.lock().unwrap())?;

    let mut serial = tokio_serial::new("/dev/ttyS0", 115_200)
        .timeout(std::time::Duration::from_secs(2))
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .open_native_async()?;

    /*
    let mut serial = rppal::uart::Uart::with_path("/dev/serial0",115_200, rppal::uart::Parity::None, 8, 1)?;
    serial.set_read_mode(16, std::time::Duration::ZERO)?;
    dbg!(serial.input_len()?);
    dbg!(serial.is_read_blocking());
    */
    let mut message_buffer = bytes::BytesMut::with_capacity(1024);
    let mut initialized = false;

    for _ in 0..100_000 {
        let accel = imu::get_accel(&mut i2c.lock().unwrap())?;
        // println!("Acceleration x: {} y: {} z: {}", accel.x, accel.y, accel.z);
        serial::read_step(&mut serial, &mut message_buffer, &mut initialized).await;
        thread::sleep(Duration::from_millis(5));
    }

    }
    rclrs::spin(Arc::clone(&microcat_node.node))?;
    Ok(())

}
