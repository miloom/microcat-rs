use icm20608g::structs::{
    AccelConfig1, AccelConfig2, AccelMeasurements, Config, GyroConfig, GyroOffset,
    PowerManagement1, ReadRegister, WhoAmI, WriteRegister,
};
use rppal::i2c::I2c;
use std::error::Error;

pub fn setup(i2c: &mut I2c) -> Result<(), Box<dyn Error>> {
    i2c.set_slave_address(0x68)?;

    let who_am_i = WhoAmI::new(i2c)?;
    println!("IMU id: {}", who_am_i.device_id);

    let mut pwr_mgmt = PowerManagement1::new(i2c)?;
    pwr_mgmt.print_table(i2c)?;
    // Activate the IMU by bringing it out of sleep
    pwr_mgmt.sleep = false;
    pwr_mgmt.write(i2c)?;

    let mut config = Config::new(i2c)?;
    config.print_table(i2c)?;
    // Set the gyroscope and temp sensor to slowest mode highest res
    config.dlpf_cfg = 6;
    config.write(i2c)?;

    let mut gyro_config = GyroConfig::new(i2c)?;
    // Set the gyroscope range to 250dps
    gyro_config.full_scale_select = 0b0;
    // Make sure to use the gyro rate that is set above and not override it.
    gyro_config.fchoice_b = 0b0;
    gyro_config.write(i2c)?;
    gyro_config.print_table(i2c)?;

    let mut accel_config = AccelConfig1::new(i2c)?;
    // Set the accelerometer range to (+-4g)
    accel_config.full_scale_select = 0b01;
    accel_config.write(i2c)?;
    accel_config.print_table(i2c)?;

    let mut accel_config2 = AccelConfig2::new(i2c)?;
    // Set accelerometer to average 32 samples
    accel_config2.dec2_cfg = 0b11;
    // This and next line set low speed measurements and low pass filter to filter out most noise
    accel_config2.accel_fchoice_b = false;
    accel_config2.dlpf_cfg = 6;
    accel_config2.write(i2c)?;
    accel_config2.print_table(i2c)?;

    let mut gyro_offset = GyroOffset::new(i2c)?;
    gyro_offset.xg_offs = 0;
    gyro_offset.write(i2c)?;
    gyro_offset.print_table(i2c)?;

    Ok(())
}

pub fn get_accel(i2c: &mut I2c) -> Result<AccelMeasurements, Box<dyn Error>> {
    i2c.set_slave_address(0x68)?;
    Ok(AccelMeasurements::new(i2c)?)
}
