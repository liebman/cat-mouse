use std::time::Duration;

use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::i2c;
use esp_idf_hal::prelude::*;

use anyhow::Context;
use embedded_hal::delay::DelayUs;
use esp_idf_hal::task;
use log::*;

use luna::Luna;
use luna::Mode;
use proximity::Proximity;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;

    let i2c0 = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio47;
    let pin_data_ready = peripherals.pins.gpio4;

    info!("Starting I2C unit 0");
    let config = i2c::config::Config::new().baudrate(400.kHz().into());
    let i2c: &'static _ =
        shared_bus::new_std!(i2c::I2cDriver = i2c::I2cDriver::new(i2c0, sda, scl, &config)?)
            .context("failed to create shared_bus for I2C0")?;

    info!("Starting Luna on I2C unit 0");
    let mut luna = Luna::new(i2c.acquire_i2c(), pin_data_ready)?;
    // set up luna the way we want
    info!("rebootting luna");
    luna.reboot()?;
    delay.delay_ms(500u32)?;
    info!("rebootting luna complete");
    luna.set_mode(Mode::Continous)?;
    luna.set_interval(Duration::from_millis(500))
        .map_err(|e| to_anyhow(e))?;
    luna.subscribe();

    loop {
        if task::wait_notification(Some(Duration::from_millis(2000))).is_none() {
            info!("timeout waiting for notification");
            continue;
        }
        let distance = luna.get_proximity().map_err(|e| to_anyhow(e))?;
        info!("distance={distance}");
    }
}

fn to_anyhow(e: Box<dyn std::error::Error>) -> anyhow::Error {
    anyhow::anyhow!("{e}")
}
