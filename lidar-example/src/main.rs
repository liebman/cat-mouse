use std::time::Duration;

use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::i2c;
use esp_idf_hal::ledc::config;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::prelude::*;

use anyhow::Context;
use embedded_hal::delay::DelayUs;
use lidar::LidarCmd;
use log::*;

use encoder::Encoder;
use lidar::Lidar;
use luna::Luna;
use motor::Motor;
use speed_control::SpeedControl;

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

    let lidar_speed = SpeedControl::new(motor_lidar, encoder_lidar, LIDAR_CONFIG)?;

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

    info!("create lidar");
    let mut lidar = Lidar::new(lidar_speed, luna);
    let rx = lidar.subscribe();
    info!("start lidar");
    lidar.sender().send(LidarCmd::State(true))?;
    loop {
        let frame = rx.recv()?;
        let left = frame.get_range_left();
        let front = frame.get_range_front();
        let right = frame.get_range_right();
        info!("l/f/r: {left:.2} / {front:.2} / {right:.2} {frame}");
    }
}
