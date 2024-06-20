use std::error::Error;
use rppal::{pwm, gpio};
use std::thread;
use std::time::Duration;

use crate::consts::LED_PWM_FREQUENCY;
use crate::consts::G_LED;

enum Led {
    Gpio(gpio::OutputPin),
    Pwm(pwm::Pwm),
}

pub struct Rgb {
    red: Led,
    green: Led,
    blue: Led,
}

pub fn init_leds() -> Result<Rgb, Box<dyn Error>> {
    
    let red_pwm =
        pwm::Pwm::with_frequency(pwm::Channel::Pwm0, LED_PWM_FREQUENCY, 0.0, pwm::Polarity::Normal, false)?;
    let blue_pwm = pwm::Pwm::with_frequency(pwm::Channel::Pwm1, LED_PWM_FREQUENCY, 0.0, pwm::Polarity::Normal, false)?;
    let mut green_pwm = gpio::Gpio::new()?.get(G_LED)?.into_output();
    green_pwm.set_pwm_frequency(LED_PWM_FREQUENCY, 0.0)?;

    Ok(Rgb {
        red: Led::Pwm(red_pwm),
        green: Led::Gpio(green_pwm),
        blue: Led::Pwm(blue_pwm),
    })
}

pub fn test_leds(leds: &mut Rgb) -> Result<(), Box<dyn Error>> {
    for r_duty in (0..=100).step_by(5) {
        let r_duty = f64::from(r_duty) / 100.0;
        match &mut leds.red {
            Led::Gpio(ref mut pin) => {
                pin.set_pwm_frequency(LED_PWM_FREQUENCY, r_duty)?;
            },
            Led::Pwm(ref mut channel) => {
                channel.set_duty_cycle(r_duty)?;
                channel.enable()?;
            },
        }
        for g_duty in (0..100).step_by(5) {
            let g_duty = f64::from(g_duty) / 100.0;
            match &mut leds.green {
                Led::Gpio(ref mut pin) => {
                    pin.set_pwm_frequency(LED_PWM_FREQUENCY, g_duty)?;
                },
                Led::Pwm(ref mut channel) => {
                    channel.set_duty_cycle(g_duty)?;
                    channel.enable()?;
                },
            }
            for b_duty in (0..100).step_by(5) {
                let b_duty = f64::from(b_duty) / 100.0;
                match &mut leds.blue {
                    Led::Gpio(ref mut pin) => {
                        pin.set_pwm_frequency(LED_PWM_FREQUENCY, b_duty)?;
                    },
                    Led::Pwm(ref mut channel) => {
                        channel.set_duty_cycle(b_duty)?;
                        channel.enable()?;
                    },
                }

                println!("Red {r_duty} Green {g_duty} Blue {b_duty}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
    Ok(())
}
