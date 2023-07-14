use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::AnyOutputPin;
use esp_idf_hal::prelude::*;

use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;

use anyhow::Context;
use esp_idf_hal::uart;
use log::*;

use lidar::Lidar;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;

    let config = uart::config::Config::default()
        .baudrate(Hertz(230_400))
        .data_bits(uart::config::DataBits::DataBits8)
        .stop_bits(uart::config::StopBits::STOP1)
        .flow_control(uart::config::FlowControl::None);
    let uart = uart::UartRxDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio1,
        None::<AnyInputPin>,
        None::<AnyOutputPin>,
        &config,
    )?;

    info!("create lidar");
    let mut lidar = Lidar::new(uart, peripherals.pins.gpio2.into());
    loop {
        info!("power lidar on");
        lidar.set_power_on();

        info!("wait for lidar to get up to speed and sync");
        while !lidar.is_synced() {
            delay.delay_ms(100);
        }

        info!("wait for lidar to get a full sweep");
        delay.delay_ms(2000);

        // take readings once a second for a a bit
        for _ in 0..20 {
            let left = lidar.get_range_left();
            let front = lidar.get_range_front();
            let right = lidar.get_range_right();
            info!("l/f/r: {left:.2} / {front:.2} / {right:.2}");
            delay.delay_ms(1000);
        }

        // turn off lidar for a while
        info!("power lidar off");
        lidar.set_power_off();
        delay.delay_ms(60000);
    }
}
