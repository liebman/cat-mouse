use esp_idf_hal::gpio::Output;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::EspError;

use log::*;

pub struct Motor<'d, PH: OutputPin> {
    name: String,
    pin_ph: PinDriver<'d, PH, Output>,
    pwm: LedcDriver<'d>,
    invert: bool,
}

impl<'d, PH: OutputPin> Motor<'d, PH> {
    pub fn new<EN, C>(
        name: &str,
        channel: impl Peripheral<P = C> + 'd,
        timer_driver: &LedcTimerDriver<'d>,
        phase: PH,
        enable: EN,
        invert: bool,
    ) -> Result<Self, EspError>
    where
        EN: OutputPin + 'd,
        C: LedcChannel,
    {
        let pwm = LedcDriver::new(channel, timer_driver, enable)?;
        Ok(Motor {
            name: name.to_string(),
            pin_ph: PinDriver::output(phase)?,
            pwm,
            invert,
        })
    }

    pub fn set_percent(&mut self, percent: f32) -> Result<(), EspError> {
        let duty = (self.pwm.get_max_duty() as f32 * percent.abs() / (100f32)) as u32;
        let direction = percent.signum() as i8 * if self.invert { -1 } else { 1 };
        trace!("set_percent: percent={percent} duty={duty} direction={direction}");
        match direction {
            x if x > 0 => self.pin_ph.set_low()?,
            x if x < 0 => self.pin_ph.set_high()?,
            _ => {}
        };
        self.pwm.set_duty(duty)
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }
}
