use std::error::Error;

use rppal::system::DeviceInfo;

mod consts;
mod rgb;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Blinking an LED on a {}.", DeviceInfo::new()?.model());

    let mut rgb = rgb::init_leds()?;

    rgb::test_leds(&mut rgb)?;
    Ok(())
}
