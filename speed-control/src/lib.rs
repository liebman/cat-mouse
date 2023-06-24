use std::ptr;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::SendError;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use esp_idf_hal::gpio::OutputPin;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_sys::vTaskPrioritySet;
use esp_idf_sys::EspError;
use esp_idf_sys::TaskHandle_t;
use esp_idf_sys::ESP_TASK_PRIO_MAX;

use log::*;
use pid::Pid;

use encoder::Encoder;
use motor::Motor;

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub interval: Duration,
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum SpeedControlCmd {
    Tick(u64),
    SetSpeed(f32),
    SetPid((f32, f32, f32)),
}

#[derive(Debug, Clone)]
pub struct SpeedControl<'d> {
    name: Arc<str>,
    tx: Sender<SpeedControlCmd>,
    encoder: Encoder<'d>,
}

const MILLIS_PER_SEC: u64 = 1_000;

impl<'d> SpeedControl<'d> {
    pub fn new(
        mut motor: Motor<'static, impl OutputPin>,
        encoder: Encoder<'static>,
        config: Config,
    ) -> Result<Self, SpeedControlError> {
        let (tx, rx): (Sender<SpeedControlCmd>, Receiver<SpeedControlCmd>) = mpsc::channel();
        let mut pid = Pid::new(
            config.p, config.i, config.d, 100.0, 100.0, 100.0, 100.0, 0.0,
        );
        let interval = config.interval;
        let name: Arc<str> = motor.get_name().into();
        let mut active = false;
        {
            let tx = tx.clone();
            let name = name.clone();
            let encoder = encoder.clone();
            let timer = EspTaskTimerService::new()?
                .timer(move || tx.send(SpeedControlCmd::Tick(0)).unwrap())?;
            thread::Builder::new().stack_size(4096).spawn(move || {
                unsafe { vTaskPrioritySet(ptr::null_mut() as TaskHandle_t, ESP_TASK_PRIO_MAX - 2) }
                let mut last_position = 0i64;
                loop {
                    match rx.recv().unwrap() {
                        SpeedControlCmd::Tick(_) => {
                            if !active {
                                continue;
                            }

                            let position = encoder.get_position();
                            let delta = (position - last_position) as i32;
                            last_position = position;
                            trace!("{name}: delta={delta}");
                            let output = pid.next_control_output(delta as f32);
                            trace!("{name}: output={output:+?}");
                            motor.set_percent(output.output).unwrap();
                        }
                        SpeedControlCmd::SetPid((p, i, d)) => {
                            pid.kp = p;
                            pid.ki = i;
                            pid.kd = d;
                        }
                        SpeedControlCmd::SetSpeed(speed) => {
                            let speed =
                                speed / (MILLIS_PER_SEC as f32 / interval.as_millis() as f32);
                            pid.setpoint = speed;
                            if speed == 0.0 {
                                active = false;
                                info!("{name}: SetSpeed: 0 - canceling timer");
                                timer.cancel().unwrap();
                                pid.reset_integral_term();
                                motor.set_percent(0.0).unwrap();
                            } else {
                                active = true;
                                if !timer.is_scheduled().unwrap() {
                                    info!("{name}: SetSpeed: not 0 - starting timer");
                                    timer.every(interval).unwrap();
                                }
                            }
                        }
                    }
                }
            })?;
        }

        Ok(Self { name, tx, encoder })
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    pub fn send(&self, cmd: SpeedControlCmd) -> Result<(), SendError<SpeedControlCmd>> {
        self.tx.send(cmd)
    }

    pub fn get_position(&self) -> i64 {
        self.encoder.get_position()
    }
}

#[derive(Debug)]
pub enum SpeedControlError {
    EspError(EspError),
    IOError(std::io::Error),
}

impl std::fmt::Display for SpeedControlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))
    }
}

impl std::error::Error for SpeedControlError {}

impl From<EspError> for SpeedControlError {
    fn from(e: EspError) -> Self {
        SpeedControlError::EspError(e)
    }
}

impl From<std::io::Error> for SpeedControlError {
    fn from(e: std::io::Error) -> Self {
        SpeedControlError::IOError(e)
    }
}
