use anyhow::Context;
use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::ledc::config;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::prelude::*;

use motor::Motor;

fn main() -> anyhow::Result<()> {
    println!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    // left motor pins
    let pin_m1_ph = peripherals.pins.gpio8;
    let pin_m1_en = peripherals.pins.gpio18;
    // right motor pins
    let pin_m2_ph = peripherals.pins.gpio17;
    let pin_m2_en = peripherals.pins.gpio16;

    let config = config::TimerConfig::new()
        .frequency(50.Hz())
        .resolution(Resolution::Bits10);
    let timer_driver =
        LedcTimerDriver::new(peripherals.ledc.timer0, &config).context("creating timer driver")?;

    let mut motor_left = Motor::new(
        "left",
        peripherals.ledc.channel0,
        &timer_driver,
        pin_m1_ph,
        pin_m1_en,
        false,
    )
    .context("creating left motor")?;

    let mut motor_right = Motor::new(
        "right",
        peripherals.ledc.channel1,
        &timer_driver,
        pin_m2_ph,
        pin_m2_en,
        true,
    )
    .context("creating right motor")?;

    motor_left
        .set_percent(75.0)
        .context("set left motor percent")?;
    motor_right
        .set_percent(75.0)
        .context("set right motor percent")?;
    delay.delay_ms(10000)?;
    motor_left
        .set_percent(0.0)
        .context("set left motor 0.0 percent")?;
    motor_right
        .set_percent(0.0)
        .context("set right motor 0.0 percent")?;

    loop {
        delay.delay_ms(1000)?;
    }
}
