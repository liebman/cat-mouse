use std::time::Duration;

use differential_drive::Drive;
use embedded_hal::delay::DelayUs;
use embedded_hal::i2c::I2c;
use encoder::Encoder;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::AnyOutputPin;
use esp_idf_hal::i2c::I2cError;
use esp_idf_hal::ledc;
use esp_idf_hal::ledc::LedcChannel;
use esp_idf_hal::ledc::LedcTimer;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::pcnt::Pcnt;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::units::FromValueType;
use esp_idf_sys::EspError;
use lidar::Lidar;
use log::info;
use luna::Luna;
use luna::LunaError;
use motor::Motor;
use position_control::PositionControl;
use position_control::PositionControlError;
use proximity::Proximity;
use speed_control::SpeedControl;
use speed_control::SpeedControlError;
use wheel::Wheel;

use crate::peripherals::EncoderPeripherals;
use crate::peripherals::LunaPeripherals;
use crate::peripherals::MotorPeripherals;

const WHEEL_DIAMETER: i32 = 60; // 60 mm wheel diameter
const WHEEL_DISTANCE: i32 = 100; // distance bewtween wheels

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

pub struct MotorFactory<'d> {
    timer_driver: LedcTimerDriver<'d>,
}

impl<'d> MotorFactory<'d> {
    pub fn new<T>(timer: impl Peripheral<P = T> + 'static) -> Result<Self, EspError>
    where
        T: LedcTimer,
    {
        let timer_config = ledc::config::TimerConfig::new()
            .frequency(50.Hz())
            .resolution(Resolution::Bits10);
        Ok(Self {
            timer_driver: LedcTimerDriver::new(timer, &timer_config)?,
        })
    }

    pub fn motor<C>(
        &self,
        name: &str,
        peripherals: MotorPeripherals<impl Peripheral<P = C> + 'd>,
        invert: bool,
    ) -> Result<Motor<'d, AnyOutputPin>, EspError>
    where
        C: LedcChannel,
    {
        Motor::new(
            name,
            peripherals.channel,
            &self.timer_driver,
            peripherals.ph,
            peripherals.en,
            invert,
        )
    }
}

pub fn encoder<PCNT>(
    peripherals: EncoderPeripherals<impl Peripheral<P = PCNT> + 'static>,
    invert: bool,
) -> Result<Encoder<'static>, EspError>
where
    PCNT: Pcnt,
{
    Encoder::new(
        peripherals.pcnt,
        peripherals.a,
        peripherals.b,
        TICKS_PER_REVOLUTION,
        invert,
    )
}

pub fn speed(
    motor: Motor<'static, AnyOutputPin>,
    encoder: Encoder<'static>,
) -> Result<SpeedControl<'static>, SpeedControlError> {
    SpeedControl::new(motor, encoder, SPEED_CONFIG)
}

pub fn positon(speed: SpeedControl<'static>) -> Result<PositionControl, PositionControlError> {
    PositionControl::new(speed, POSITION_CONFIG, 720.0)
}

pub fn drive(
    left: PositionControl<'static>,
    right: PositionControl<'static>,
) -> Result<Drive<'static>, std::io::Error> {
    let left_wheel = Wheel::new(left, WHEEL_DIAMETER);
    let right_wheel = Wheel::new(right, WHEEL_DIAMETER);
    Drive::new(left_wheel, right_wheel, WHEEL_DISTANCE)
}

pub fn luna<'d, I2C>(
    i2c: I2C,
    peripherals: LunaPeripherals,
) -> Result<Luna<'d, I2C, AnyInputPin>, LunaError>
where
    I2C: I2c<Error = I2cError> + Send,
{
    let mut luna = Luna::new(i2c, peripherals.ready)?;
    // set up luna the way we want
    info!("rebootting luna");
    luna.reboot()?;
    delay.delay_ms(500u32).expect("delay should not fail");
    info!("rebootting luna complete");
    luna.set_mode(luna::Mode::Continous)?;
    Ok(luna)
}

pub fn lidar(
    motor: SpeedControl<'static>,
    proximity: impl Proximity + Send + 'static,
) -> Result<Lidar, EspError> {
    Ok(Lidar::new(motor, proximity))
}
