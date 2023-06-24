use core::fmt::Debug;
use std::fmt;
use std::fmt::Display;
use std::sync::atomic::AtomicPtr;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;

use embedded_hal::i2c::I2c;
use esp_idf_hal::gpio::Input;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::I2cError;
use esp_idf_hal::task;
use esp_idf_sys::tskTaskControlBlock;
use esp_idf_sys::xTaskGetCurrentTaskHandle;
use esp_idf_sys::EspError;
use log::*;

use proximity::Proximity;

const ADDRESS: u8 = 0x10;

#[derive(Debug)]
#[allow(dead_code)]
#[repr(u8)]
enum Register {
    DistLow = 0x00,
    DistHigh = 0x01,
    AmpLow = 0x02,
    AmpHigh = 0x03,
    TempLow = 0x04,
    TempHigh = 0x05,
    TickLow = 0x06,
    TickHigh = 0x07,
    ErrorLow = 0x08,
    ErrorHigh = 0x09,
    VersionRevision = 0x0a,
    VersionMinor = 0x0b,
    VersionMajor = 0x0c,
    SerialNumber = 0x10, // 14 byte ascii, first byte is 0x10
    UltraLowPower = 0x1f,
    Save = 0x20,
    Reboot = 0x21,
    SlaveAddr = 0x22,
    Mode = 0x23,
    TrigOneShot = 0x24,
    Enable = 0x25,
    FpsLow = 0x26,
    FpsHigh = 0x27,
    LowPower = 0x28,
    RestoreFactory = 0x29,
    AmpThrLow = 0x2a,
    AmpThrHigh = 0x2b,
    DummyDistLow = 0x2c,
    DummyDistHigh = 0x2d,
    MinDistLow = 0x2e,
    MinDistHigh = 0x2f,
    MaxDistLow = 0x30,
    MaxDistHigh = 0x31,
    Signature = 0x3c, // 4 bytes 'L' 'U' 'N' 'A'
}

#[derive(Debug)]
#[repr(u8)]
enum Reboot {
    Reboot = 0x02,
}

#[derive(Debug)]
#[repr(u8)]
pub enum Mode {
    Continous = 0x00,
    Trigger = 0x01,
}

pub struct Signature(u8, u8, u8, u8);
impl Signature {
    pub fn is_valid(&self) -> bool {
        self.0 == b'L' && self.1 == b'U' && self.2 == b'N' && self.3 == b'A'
    }
}
impl fmt::Display for Signature {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}{}{}{}",
            self.0 as char, self.1 as char, self.2 as char, self.3 as char
        )
    }
}

pub struct Version {
    major: u8,
    minor: u8,
    revision: u8,
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.revision)
    }
}

/// Driver errors.
#[derive(Debug, PartialEq, Eq)]
pub enum LunaError {
    InvalidSignature,          // invalid device signature
    InvalidMode(u8),           // invalid device operating mode
    InvalidInterval(Duration), // invalid sampling interval
    InvalidDistance(u16),      // an out of range value was returned
    Esp(EspError),
    I2c(I2cError),
}

impl Display for LunaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl std::error::Error for LunaError {}

impl From<EspError> for LunaError {
    fn from(e: EspError) -> Self {
        LunaError::Esp(e)
    }
}

impl From<I2cError> for LunaError {
    fn from(e: I2cError) -> Self {
        LunaError::I2c(e)
    }
}

pub struct Luna<'d, I2C, IP: InputPin> {
    i2c: I2C,
    _pin: Mutex<PinDriver<'d, IP, Input>>,
    _task: Arc<AtomicPtr<tskTaskControlBlock>>,
}

impl<'d, I2C, IP> Luna<'d, I2C, IP>
where
    I2C: I2c<Error = I2cError> + Send,
    IP: InputPin,
{
    pub fn new(i2c: I2C, data_pin: IP) -> Result<Luna<'d, I2C, IP>, LunaError> {
        let mut pin = PinDriver::input(data_pin)?;
        let task: Arc<AtomicPtr<tskTaskControlBlock>> =
            Arc::new(AtomicPtr::new(std::ptr::null_mut::<tskTaskControlBlock>()));
        pin.set_interrupt_type(InterruptType::PosEdge)?;
        unsafe {
            let task = task.clone();
            pin.subscribe(move || {
                let handle = task.load(Ordering::Relaxed);
                #[allow(clippy::zero_ptr)]
                if handle != 0 as *mut tskTaskControlBlock {
                    task::notify(handle, 1);
                }
            })?;
        }
        let mut luna = Luna {
            i2c,
            _pin: Mutex::new(pin),
            _task: task,
        };
        let signature = luna.get_signature()?;
        info!("signature: {signature}");
        if !signature.is_valid() {
            return Err(LunaError::InvalidSignature);
        }
        let version = luna.get_version()?;
        info!("version: {version}");
        let serial_number = luna.get_serial_number()?;
        info!("serial number: {serial_number}");
        info!("raw distance: {}", luna.get_distance()?);
        Ok(luna)
    }

    /// distance in mm
    pub fn get_distance(&mut self) -> Result<u16, LunaError> {
        let mut buffer = [0u8, 2];
        self.read_bytes(Register::DistLow, &mut buffer)?;
        Ok(((buffer[1] as u16) << 8) | (buffer[0] as u16))
    }

    pub fn get_fps(&mut self) -> Result<u16, LunaError> {
        let mut buffer = [0u8; 2];
        self.read_bytes(Register::FpsLow, &mut buffer)?;
        Ok(((buffer[1] as u16) << 8) | (buffer[0] as u16))
    }

    pub fn set_fps(&mut self, fps: u16) -> Result<(), LunaError> {
        info!("set_fps: {fps}");
        self.write_u16(Register::FpsLow, fps)?;
        Ok(())
    }

    pub fn get_mode(&mut self) -> Result<Mode, LunaError> {
        match self.read_byte(Register::Mode)? {
            value if value == Mode::Continous as u8 => Ok(Mode::Continous),
            value if value == Mode::Trigger as u8 => Ok(Mode::Trigger),
            value => Err(LunaError::InvalidMode(value)),
        }
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), LunaError> {
        info!("set_mode: {:?}", mode);
        self.write_byte(Register::Mode, mode as u8)?;
        Ok(())
    }

    pub fn reboot(&mut self) -> Result<(), LunaError> {
        self.write_byte(Register::Reboot, Reboot::Reboot as u8)?;
        Ok(())
    }

    pub fn get_signature(&mut self) -> Result<Signature, LunaError> {
        let mut buffer = [0u8; 4];
        self.read_bytes(Register::Signature, &mut buffer)?;
        Ok(Signature(buffer[0], buffer[1], buffer[2], buffer[3]))
    }

    pub fn get_version(&mut self) -> Result<Version, LunaError> {
        let mut buffer = [0u8; 3];
        self.read_bytes(Register::VersionRevision, &mut buffer)?;
        Ok(Version {
            major: buffer[2],
            minor: buffer[1],
            revision: buffer[0],
        })
    }

    pub fn get_serial_number(&mut self) -> Result<String, LunaError> {
        let mut buffer = [0x20u8; 14];
        self.read_bytes(Register::SerialNumber, &mut buffer)?;
        Ok(std::str::from_utf8(&buffer).unwrap().to_string())
    }

    fn read_byte(&mut self, reg: Register) -> Result<u8, I2cError> {
        let cmd = [reg as u8];
        let mut buffer: [u8; 1] = [0];
        self.i2c.write_read(ADDRESS, &cmd, &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_bytes(&mut self, reg: Register, buffer: &mut [u8]) -> Result<(), I2cError> {
        let cmd = [reg as u8];
        self.i2c.write_read(ADDRESS, &cmd, buffer)
    }

    fn write_byte(&mut self, reg: Register, value: u8) -> Result<(), I2cError> {
        let cmd = [reg as u8, value];
        self.i2c.write(ADDRESS, &cmd)
    }

    fn write_u16(&mut self, reg: Register, value: u16) -> Result<(), I2cError> {
        let cmd = [reg as u8, (value & 0xff) as u8, (value >> 8) as u8];
        self.i2c.write(ADDRESS, &cmd)
    }
}

impl<'d, I2C, IP> Proximity for Luna<'d, I2C, IP>
where
    I2C: I2c<Error = I2cError> + Send,
    IP: InputPin,
{
    fn set_interval(&mut self, interval: Duration) -> Result<(), Box<dyn std::error::Error>> {
        let milliseconds = interval.as_millis();
        info!("set_interval: {milliseconds}");
        if !(10..=1000).contains(&milliseconds) {
            return Err(Box::new(LunaError::InvalidInterval(interval)));
        }
        self.set_fps(1000u16 / milliseconds as u16)?;
        // need to read once to clear the interrupt bit so it will
        // interrupt again
        self.get_distance()?;
        Ok(())
    }

    fn get_proximity(&mut self) -> Result<u16, Box<dyn std::error::Error>> {
        let cm = self.get_distance()?;
        if cm > 1000 {
            return Err(Box::new(LunaError::InvalidDistance(cm)));
        }
        Ok(cm * 10)
    }

    fn subscribe(&mut self) {
        let handle = unsafe { xTaskGetCurrentTaskHandle() };
        self._task.store(handle, Ordering::Relaxed);
    }
}
