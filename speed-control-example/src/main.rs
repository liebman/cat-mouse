use std::time::Duration;

use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::ledc::config;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::prelude::*;

use anyhow::Context;
use embedded_hal::delay::DelayUs;
use log::*;

use encoder::Encoder;
use motor::Motor;
use speed_control::SpeedControl;
use speed_control::SpeedControlCmd;

const TICKS_PER_REVOLUTION: u32 = 1440;

const LIDAR_CONFIG: speed_control::Config = speed_control::Config {
    interval: Duration::from_millis(100),
    p: 0.5,
    i: 0.1,
    d: 0.2,
};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    // lidar motor pins
    let pin_m3_en = peripherals.pins.gpio46;
    let pin_m3_ph = peripherals.pins.gpio3;
    // lidar encoder pins
    let pin_m3_enc1 = peripherals.pins.gpio11;
    let pin_m3_enc2 = peripherals.pins.gpio12;

    let config = config::TimerConfig::new()
        .frequency(50.Hz())
        .resolution(Resolution::Bits10);
    let timer_driver =
        LedcTimerDriver::new(peripherals.ledc.timer0, &config).context("creating timer driver")?;

    let motor_lidar = Motor::new(
        "lidar",
        peripherals.ledc.channel0,
        &timer_driver,
        pin_m3_ph,
        pin_m3_en,
        true,
    )
    .context("creating lidar motor")?;

    let encoder_lidar = Encoder::new(
        peripherals.pcnt0,
        pin_m3_enc1,
        pin_m3_enc2,
        TICKS_PER_REVOLUTION,
        true,
    )
    .context("create lidar encoder")?;

    let lidar = SpeedControl::new(motor_lidar, encoder_lidar, LIDAR_CONFIG)?;
    lidar.send(SpeedControlCmd::SetSpeed(360.0))?;

    loop {
        delay.delay_ms(1000);
    }
}
