use std::time::Duration;

use anyhow::Context;
use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::ledc::config;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::prelude::*;
use log::*;

use encoder::Encoder;
use motor::Motor;
use position_control::PositionControl;
use position_control::PositionControlCmd;
use speed_control::SpeedControl;

const TICKS_PER_REVOLUTION: u32 = 1440;

const SPEED_CONFIG: speed_control::Config = speed_control::Config {
    interval: Duration::from_millis(100),
    p: 0.5,
    i: 0.1,
    d: 0.2,
};

const POSITION_CONFIG: position_control::Config = position_control::Config {
    interval: Duration::from_millis(100),
    p: 0.5,
    i: 0.01,
    d: 0.2,
};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    // left motor pins
    let pin_m1_ph = peripherals.pins.gpio8;
    let pin_m1_en = peripherals.pins.gpio18;
    // right motor pins
    let pin_m2_ph = peripherals.pins.gpio17;
    let pin_m2_en = peripherals.pins.gpio16;
    // left encoder pins
    let pin_m1_enc1 = peripherals.pins.gpio5;
    let pin_m1_enc2 = peripherals.pins.gpio6;
    // right encoder pins
    let pin_m2_enc1 = peripherals.pins.gpio7;
    let pin_m2_enc2 = peripherals.pins.gpio15;

    let config = config::TimerConfig::new()
        .frequency(50.Hz())
        .resolution(Resolution::Bits10);
    let timer_driver =
        LedcTimerDriver::new(peripherals.ledc.timer0, &config).context("creating timer driver")?;

    let motor_left = Motor::new(
        "left",
        peripherals.ledc.channel0,
        &timer_driver,
        pin_m1_ph,
        pin_m1_en,
        false,
    )
    .context("creating left motor")?;

    let motor_right = Motor::new(
        "right",
        peripherals.ledc.channel1,
        &timer_driver,
        pin_m2_ph,
        pin_m2_en,
        true,
    )
    .context("creating right motor")?;

    let encoder_left = Encoder::new(
        peripherals.pcnt0,
        pin_m1_enc1,
        pin_m1_enc2,
        TICKS_PER_REVOLUTION,
        false,
    )
    .context("create left encoder")?;

    let encoder_right = Encoder::new(
        peripherals.pcnt1,
        pin_m2_enc1,
        pin_m2_enc2,
        TICKS_PER_REVOLUTION,
        true,
    )
    .context("create right encoder")?;

    let speed_left = SpeedControl::new(motor_left, encoder_left, SPEED_CONFIG)?;
    let speed_right = SpeedControl::new(motor_right, encoder_right, SPEED_CONFIG)?;

    let position_left = PositionControl::new(speed_left, POSITION_CONFIG, 720.0)?;
    let position_right = PositionControl::new(speed_right, POSITION_CONFIG, 720.0)?;

    let mut reporter = PositinReporter::new();

    info!("forward");
    position_left.send(PositionControlCmd::SetPosition(10000))?;
    position_right.send(PositionControlCmd::SetPosition(10000))?;
    while position_left.is_active() && position_right.is_active() {
        reporter.update(position_left.get_position(), position_right.get_position());
        delay.delay_ms(1000);
    }
    info!("done");
    delay.delay_ms(1000);
    info!("reverse");
    position_left.send(PositionControlCmd::SetPosition(0))?;
    position_right.send(PositionControlCmd::SetPosition(0))?;
    while position_left.is_active() && position_right.is_active() {
        reporter.update(position_left.get_position(), position_right.get_position());
        delay.delay_ms(1000);
    }
    info!("done");

    loop {
        delay.delay_ms(1000);
    }
}

#[derive(Debug)]
struct PositinReporter {
    left: i64,
    right: i64,
}

impl PositinReporter {
    fn new() -> Self {
        Self { left: 0, right: 0 }
    }

    fn update(&mut self, left: i64, right: i64) -> bool {
        if (left, right) != (self.left, self.right) {
            (self.left, self.right) = (left, right);
            info!("{self:+?}");
            return true;
        }
        false
    }
}
