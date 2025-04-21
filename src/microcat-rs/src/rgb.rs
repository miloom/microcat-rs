use crate::consts::G_LED;
use crate::consts::LED_PWM_FREQUENCY;
use rppal::{gpio, pwm};
use std::error::Error;
use std::thread;
use std::time::Duration;
use tracing::{debug, error};

#[derive(Debug)]
enum Led {
    Gpio(gpio::OutputPin),
    Pwm(pwm::Pwm),
}

#[derive(Debug)]
pub struct Rgb {
    red: Led,
    green: Led,
    blue: Led,
}

impl Rgb {
    pub fn init_leds() -> Result<Rgb, Box<dyn Error>> {
        debug!("Initializing leds");
        let red_pwm = pwm::Pwm::with_frequency(
            pwm::Channel::Pwm0,
            LED_PWM_FREQUENCY,
            0.0,
            pwm::Polarity::Normal,
            false,
        )?;
        debug!("RED done");
        let blue_pwm = pwm::Pwm::with_frequency(
            pwm::Channel::Pwm1,
            LED_PWM_FREQUENCY,
            0.0,
            pwm::Polarity::Normal,
            false,
        )?;
        debug!("BLUE done");
        let mut green_pwm = gpio::Gpio::new()?.get(G_LED)?.into_output();
        green_pwm.set_pwm_frequency(LED_PWM_FREQUENCY, 0.0)?;
        debug!("GREEN done");
        Ok(Rgb {
            red: Led::Pwm(red_pwm),
            green: Led::Gpio(green_pwm),
            blue: Led::Pwm(blue_pwm),
        })
    }

    fn set_led(led: &mut Led, name: &str, value: f32) {
        match led {
            Led::Gpio(pin) => {
                if let Err(error) = pin.set_pwm_frequency(LED_PWM_FREQUENCY, value as f64) {
                    error!("Failed to set PWM for {name} LED: {error}");
                }
            }
            Led::Pwm(pwm) => {
                if let Err(error) = pwm.set_duty_cycle(value as f64) {
                    error!("Failed to set duty cycle for {name} LED: {error}");
                } else if let Err(error) = pwm.enable() {
                    error!("Failed to enable duty cycle for {name} LED: {error}");
                }
            }
        }
    }

    #[tracing::instrument(level = "trace")]
    pub fn set_color(&mut self, red: f32, green: f32, blue: f32) {
        Self::set_led(&mut self.red, "red", red);
        Self::set_led(&mut self.green, "green", green);
        Self::set_led(&mut self.blue, "blue", blue);
    }
}

#[allow(unused)]
pub async fn test_leds(leds: &mut Rgb) -> Result<(), Box<dyn Error>> {
    for r_duty in (0..=100).step_by(5) {
        let r_duty = f64::from(r_duty) / 100.0;
        match &mut leds.red {
            Led::Gpio(ref mut pin) => {
                pin.set_pwm_frequency(LED_PWM_FREQUENCY, r_duty)?;
            }
            Led::Pwm(ref mut channel) => {
                channel.set_duty_cycle(r_duty)?;
                channel.enable()?;
            }
        }
        for g_duty in (0..100).step_by(5) {
            let g_duty = f64::from(g_duty) / 100.0;
            match &mut leds.green {
                Led::Gpio(ref mut pin) => {
                    pin.set_pwm_frequency(LED_PWM_FREQUENCY, g_duty)?;
                }
                Led::Pwm(ref mut channel) => {
                    channel.set_duty_cycle(g_duty)?;
                    channel.enable()?;
                }
            }
            for b_duty in (0..100).step_by(5) {
                let b_duty = f64::from(b_duty) / 100.0;
                match &mut leds.blue {
                    Led::Gpio(ref mut pin) => {
                        pin.set_pwm_frequency(LED_PWM_FREQUENCY, b_duty)?;
                    }
                    Led::Pwm(ref mut channel) => {
                        channel.set_duty_cycle(b_duty)?;
                        channel.enable()?;
                    }
                }

                thread::sleep(Duration::from_millis(20));
            }
        }
    }
    Ok(())
}
