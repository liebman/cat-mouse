use std::ptr;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::SendError;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_sys::vTaskPrioritySet;
use esp_idf_sys::EspError;
use esp_idf_sys::TaskHandle_t;
use esp_idf_sys::ESP_TASK_PRIO_MAX;
use log::*;
use pid::Pid;

use speed_control::SpeedControl;
use speed_control::SpeedControlCmd;

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub interval: Duration,
    pub p: f64,
    pub i: f64,
    pub d: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum PositionControlCmd {
    Tick(u64), // tick from timer.
    SetPosition(i64),
    SetPid((f64, f64, f64)),
    Stop,
    SpeedCmd(SpeedControlCmd),
}

#[derive(Debug, Clone)]
pub struct PositionControl<'d> {
    tx: Sender<PositionControlCmd>,
    speed: SpeedControl<'d>,
    active: Arc<AtomicBool>,
}

impl<'d> PositionControl<'d> {
    pub fn new(
        speed: SpeedControl<'static>,
        config: Config,
        max_speed: f32, // degrees per second
    ) -> Result<Self, PositionControlError> {
        let (tx, rx): (Sender<PositionControlCmd>, Receiver<PositionControlCmd>) = mpsc::channel();

        let mut pid = Pid::new(
            config.p,
            config.i,
            config.d,
            1000.0,
            10.0,
            1000.0,
            max_speed as f64,
            0.0,
        );
        let interval = config.interval;
        let active = Arc::new(AtomicBool::new(false));
        {
            let mut stop = false;
            let speed = speed.clone();
            let active = active.clone();
            let tx = tx.clone();
            let timer = EspTaskTimerService::new()?
                .timer(move || tx.send(PositionControlCmd::Tick(0)).unwrap())?;

            thread::Builder::new().stack_size(4096).spawn(move || {
                let name = speed.get_name();
                unsafe { vTaskPrioritySet(ptr::null_mut() as TaskHandle_t, ESP_TASK_PRIO_MAX - 2) }

                loop {
                    match rx.recv().unwrap() {
                        PositionControlCmd::Tick(_) => {
                            if !active.load(Ordering::Relaxed) {
                                continue;
                            }
                            let position: f64 = speed.get_position() as f64;
                            trace!("{name}: position:{position}");

                            if stop || (position - pid.setpoint).abs() <= 2.0 {
                                info!(
                                    "{name}: stopping at {position} setpoint={} stop={stop}",
                                    pid.setpoint
                                );
                                pid.reset_integral_term();
                                active.store(false, Ordering::Relaxed);
                                stop = false;
                                speed.send(SpeedControlCmd::SetSpeed(0.0)).unwrap();
                                continue;
                            }
                            let output = pid.next_control_output(position);

                            trace!("{name}: PID update {:?}", output);
                            speed
                                .send(SpeedControlCmd::SetSpeed(output.output as f32))
                                .unwrap();
                        }
                        PositionControlCmd::Stop => {
                            info!("{name}: setting stop=true");
                            stop = true
                        }
                        PositionControlCmd::SetPosition(position) => {
                            if !timer.is_scheduled().unwrap() {
                                timer.every(interval).unwrap();
                            }
                            pid.setpoint = position as f64;
                            info!("{name}: setpoint={}", &pid.setpoint);
                            stop = false;
                            active.store(true, Ordering::Relaxed);
                        }
                        PositionControlCmd::SetPid((p, i, d)) => {
                            pid.kp = p;
                            pid.ki = i;
                            pid.kd = d;
                        }
                        PositionControlCmd::SpeedCmd(cmd) => {
                            speed.send(cmd).unwrap();
                        }
                    }
                }
            })?;
        }
        Ok(Self { tx, speed, active })
    }

    pub fn get_position(&self) -> i64 {
        self.speed.get_position()
    }

    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    pub fn send(&self, cmd: PositionControlCmd) -> Result<(), SendError<PositionControlCmd>> {
        self.tx.send(cmd)
    }
}

#[derive(Debug)]
pub enum PositionControlError {
    EspError(EspError),
    IOError(std::io::Error),
    SendError(SendError<PositionControlCmd>),
}

impl std::fmt::Display for PositionControlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))
    }
}

impl std::error::Error for PositionControlError {}

impl From<EspError> for PositionControlError {
    fn from(e: EspError) -> Self {
        PositionControlError::EspError(e)
    }
}

impl From<std::io::Error> for PositionControlError {
    fn from(e: std::io::Error) -> Self {
        PositionControlError::IOError(e)
    }
}

impl From<SendError<PositionControlCmd>> for PositionControlError {
    fn from(e: SendError<PositionControlCmd>) -> Self {
        PositionControlError::SendError(e)
    }
}
