use drv8830::{ReadRegister, WriteRegister};
use rppal::i2c::I2c;
use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

struct Motor {
    address: u16,
}
impl Motor {
    const REAR_RIGHT: Self = Self { address: 0x60 };
    const REAR_LEFT: Self = Self { address: 0x64 };
    const FRONT_RIGHT: Self = Self { address: 0x61 };
    const FRONT_LEFT: Self = Self { address: 0x63 };
    const MOTORS: [Self; 4] = [
        Self::FRONT_LEFT,
        Self::REAR_LEFT,
        Self::REAR_RIGHT,
        Self::FRONT_RIGHT,
    ];

    fn write<T: WriteRegister>(&self, data: &T, i2c: &mut I2c) -> Result<(), Box<dyn Error>> {
        i2c.set_slave_address(self.address)?;
        data.write(i2c)?;
        Ok(())
    }
    fn read<T: ReadRegister>(&self, i2c: &mut I2c) -> Result<T, Box<dyn Error>> {
        i2c.set_slave_address(self.address)?;
        Ok(T::new(i2c)?)
    }
}

pub async fn test(i2c: Arc<Mutex<I2c>>) -> Result<(), Box<dyn Error>> {
    let join = tokio::task::spawn(async move {
        loop {
            for motor in Motor::MOTORS {
                let mut data = motor
                    .read::<drv8830::Fault>(&mut i2c.lock().unwrap())
                    .unwrap();
                data.clear = true;
                motor.write(&data, &mut i2c.lock().unwrap()).unwrap();
                motor
                    .write(&drv8830::Control::BRAKE, &mut i2c.lock().unwrap())
                    .unwrap();
            }
        }
    });
    thread::sleep(Duration::from_millis(15000));
    join.await?;

    Ok(())
}
