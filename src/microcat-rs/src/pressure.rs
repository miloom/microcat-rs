use rppal::i2c::I2c;
use std::error::Error;

pub fn test_pressure(i2c:&mut I2c) -> Result<(), Box<dyn Error>> {
    i2c.set_slave_address(0x76)?;
    let in_buffer: [u8; 10]; 

    Ok(())
}
