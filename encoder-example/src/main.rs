use anyhow::Context;
use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::prelude::*;

use encoder::Encoder;

const TICKS_PER_REVOLUTION: u32 = 1440;

fn main() -> anyhow::Result<()> {
    println!("starting");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;

    // left encoder pins
    let pin_m1_enc1 = peripherals.pins.gpio5;
    let pin_m1_enc2 = peripherals.pins.gpio6;

    // right encoder pins
    let pin_m2_enc1 = peripherals.pins.gpio7;
    let pin_m2_enc2 = peripherals.pins.gpio15;

    let left_encoder = Encoder::new(
        peripherals.pcnt0,
        pin_m1_enc1,
        pin_m1_enc2,
        TICKS_PER_REVOLUTION,
        false,
    )
    .context("create left encoder")?;
    let right_encoder = Encoder::new(
        peripherals.pcnt1,
        pin_m2_enc1,
        pin_m2_enc2,
        TICKS_PER_REVOLUTION,
        true,
    )
    .context("create right encoder")?;

    let mut last_left = 0i64;
    let mut last_right = 0i64;
    loop {
        let left = left_encoder.get_position();
        let right = right_encoder.get_position();
        if left != last_left || right != last_right {
            println!("left={left} right={right}");
            (last_left, last_right) = (left, right);
        }
        delay.delay_ms(10);
    }
}
