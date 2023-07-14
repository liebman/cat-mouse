use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::AnyOutputPin;
use esp_idf_hal::i2c::I2C0;
use esp_idf_hal::ledc::CHANNEL0;
use esp_idf_hal::ledc::CHANNEL1;
use esp_idf_hal::ledc::TIMER0;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::pcnt::PCNT0;
use esp_idf_hal::pcnt::PCNT1;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::uart::UART1;

pub struct SystemPeripherals<C0, C1, T, I, P0, P1, U> {
    pub i2c_unit: I,
    pub i2c_pins: I2cPeripherals,
    pub ledc_timer: T,
    pub left_motor: MotorPeripherals<C0>,
    pub left_encoder: EncoderPeripherals<P0>,
    pub right_motor: MotorPeripherals<C1>,
    pub right_encoder: EncoderPeripherals<P1>,
    pub lidar: LidarPeripherals<U>,
    pub modem: Modem,
}

impl SystemPeripherals<CHANNEL0, CHANNEL1, TIMER0, I2C0, PCNT0, PCNT1, UART1> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        Self {
            i2c_unit: peripherals.i2c0,
            i2c_pins: I2cPeripherals {
                sda: peripherals.pins.gpio21.into(),
                scl: peripherals.pins.gpio47.into(),
            },
            ledc_timer: peripherals.ledc.timer0,
            left_motor: MotorPeripherals {
                channel: peripherals.ledc.channel0,
                ph: peripherals.pins.gpio8.into(),
                en: peripherals.pins.gpio18.into(),
            },
            left_encoder: EncoderPeripherals {
                pcnt: peripherals.pcnt0,
                a: peripherals.pins.gpio5.into(),
                b: peripherals.pins.gpio6.into(),
            },
            right_motor: MotorPeripherals {
                channel: peripherals.ledc.channel1,
                ph: peripherals.pins.gpio17.into(),
                en: peripherals.pins.gpio16.into(),
            },
            right_encoder: EncoderPeripherals {
                pcnt: peripherals.pcnt1,
                a: peripherals.pins.gpio7.into(),
                b: peripherals.pins.gpio15.into(),
            },
            lidar: LidarPeripherals {
                uart: peripherals.uart1,
                serial: peripherals.pins.gpio1.into(),
                power: peripherals.pins.gpio2.into(),
            },
            modem: peripherals.modem,
        }
    }
}

pub struct MotorPeripherals<C> {
    pub channel: C,
    pub ph: AnyOutputPin,
    pub en: AnyOutputPin,
}

pub struct EncoderPeripherals<P> {
    pub pcnt: P,
    pub a: AnyInputPin,
    pub b: AnyInputPin,
}

pub struct LidarPeripherals<U> {
    pub uart: U,
    pub serial: AnyInputPin,
    pub power: AnyOutputPin,
}

pub struct I2cPeripherals {
    pub sda: AnyIOPin,
    pub scl: AnyIOPin,
}
