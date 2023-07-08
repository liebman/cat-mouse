use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::SendError;
use std::sync::mpsc::Sender;
use std::time::Duration;

use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_svc::timer::EspTimer;
use esp_idf_svc::timer::EspTimerService;
use esp_idf_sys::EspError;

use differential_drive::Drive;
use differential_drive::DriveCmd;
use lidar::Frame;
use log::*;

use super::BrainCmd;

#[derive(Debug)]
pub(super) enum Event {
    Start,
    Frame(Frame),
    Stop,
    Timeout,
}

#[derive(Debug)]
enum State {
    Idle,
    Warmup,
    SearchChoice,
    Searching,
    Moving,
}

#[allow(dead_code)]
pub(super) struct Simple<'d> {
    brain: Sender<BrainCmd>,
    drive: Drive<'d>,
    state: State,
    timer: EspTimer,
    tx: Sender<Event>,
    rx: Receiver<Event>,
}

impl<'d> Simple<'d> {
    pub fn start(
        brain: Sender<BrainCmd>,
        drive: Drive<'static>,
    ) -> Result<Sender<Event>, BrainError> {
        let (tx, rx) = channel();
        let timer_tx = tx.clone();
        let mut simple = Simple {
            brain,
            drive,
            state: State::Idle,
            timer: EspTimerService::new()?.timer(move || timer_tx.send(Event::Timeout).unwrap())?,
            tx: tx.clone(),
            rx,
        };
        std::thread::Builder::new()
            .stack_size(8192)
            .name("simpleton".into())
            .spawn(move || {
                info!("starting loop");
                loop {
                    let event = simple.rx.recv().unwrap();
                    simple.event(event).unwrap();
                    simple.run().unwrap();
                }
            })?;
        Ok(tx)
    }

    pub fn event(&mut self, event: Event) -> Result<(), BrainError> {
        match self.state {
            #[allow(clippy::single_match)]
            State::Idle => match event {
                Event::Start => {
                    info!("Idle received Start");
                    self.brain.send(BrainCmd::LidarOnOff(true))?;
                    self.timer.after(Duration::from_secs(5))?;
                    self.state = State::Warmup;
                }
                _ => {}
            },
            State::Warmup => match event {
                Event::Stop => {
                    info!("Warmup received Stop");
                    self.drive.send(DriveCmd::Stop)?;
                    self.state = State::Idle;
                }
                Event::Timeout => {
                    info!("Warmup received Timeout");
                    self.state = State::SearchChoice;
                }
                _ => {}
            },
            State::SearchChoice => match event {
                Event::Stop => {
                    info!("SearchChoice received Stop");
                    self.brain.send(BrainCmd::LidarOnOff(false))?;
                    self.drive.send(DriveCmd::Stop)?;
                    self.state = State::Idle;
                }
                Event::Frame(frame) => {
                    let front = frame.get_range_front();
                    let left = frame.get_range_left();
                    let right = frame.get_range_right();
                    if front > 2000 {
                        self.display_ranges(&frame);
                        info!("SearchChoice: no search, just go!");
                        self.drive.send(DriveCmd::Move(front as i64))?;
                        self.state = State::Moving;
                    } else if left > right {
                        self.display_ranges(&frame);
                        info!("SearchChoice: search left");
                        self.drive.send(DriveCmd::Rotate(-270))?;
                        self.state = State::Searching;
                    } else {
                        self.display_ranges(&frame);
                        info!("SearchChoice: search right");
                        self.drive.send(DriveCmd::Rotate(270))?;
                        self.state = State::Searching;
                    }
                }
                _ => {}
            },
            State::Searching => match event {
                Event::Stop => {
                    info!("Searching received Stop");
                    self.brain.send(BrainCmd::LidarOnOff(false))?;
                    self.drive.send(DriveCmd::Stop)?;
                    self.state = State::Idle;
                }
                Event::Frame(frame) => {
                    let front = frame.get_range_front();
                    let left = frame.get_range_left();
                    let right = frame.get_range_right();
                    if left < 100 || right < 100 {
                        self.display_ranges(&frame);
                        info!("Searching: too close, back to SearchChoice");
                        self.drive.send(DriveCmd::Stop)?;
                        self.state = State::SearchChoice;
                    } else if front > 1000 {
                        self.display_ranges(&frame);
                        info!("Searching: found a path, go go go!");
                        self.drive.send(DriveCmd::Stop)?;
                        self.drive.send(DriveCmd::Move(front as i64))?;
                        self.state = State::Moving;
                    } else if !self.drive.is_active() {
                        info!("Searching: rotate complete, back to SearchChoice");
                        self.drive.send(DriveCmd::Stop)?;
                        self.state = State::SearchChoice;
                    }
                }
                _ => {}
            },
            State::Moving => match event {
                Event::Stop => {
                    info!("Moving received Stop");
                    self.brain.send(BrainCmd::LidarOnOff(false))?;
                    self.drive.send(DriveCmd::Stop)?;
                    self.state = State::Idle;
                }
                Event::Frame(frame) => {
                    let front = frame.get_range_front();
                    let left = frame.get_range_left();
                    let right = frame.get_range_right();
                    if front < 500 || left < 250 || right < 250 {
                        self.display_ranges(&frame);
                        info!("Moving: too close, back to SearchChoice");
                        self.drive.send(DriveCmd::Stop)?;
                        self.state = State::SearchChoice;
                    } else if !self.drive.is_active() {
                        info!("Moving: move complete, back to SearchChoice");
                        self.drive.send(DriveCmd::Stop)?;
                        self.state = State::SearchChoice;
                    }
                }
                _ => {}
            },
        }
        Ok(())
    }

    fn display_ranges(&self, frame: &Frame) {
        let left = frame.get_range_left();
        let front = frame.get_range_front();
        let right = frame.get_range_right();
        info!("l/f/r: {left:.2} / {front:.2} / {right:.2}");
    }

    pub fn run(&self) -> Result<(), BrainError> {
        let _ = delay.delay_ms(100);
        Ok(())
    }
}

#[derive(Debug)]
pub enum BrainError {
    Esp(EspError),
    Drive(SendError<DriveCmd>),
    IO(std::io::Error),
    Brain(SendError<BrainCmd>),
}

impl From<EspError> for BrainError {
    fn from(e: EspError) -> Self {
        BrainError::Esp(e)
    }
}

impl From<SendError<DriveCmd>> for BrainError {
    fn from(e: SendError<DriveCmd>) -> Self {
        BrainError::Drive(e)
    }
}

impl From<std::io::Error> for BrainError {
    fn from(e: std::io::Error) -> Self {
        BrainError::IO(e)
    }
}

impl From<SendError<BrainCmd>> for BrainError {
    fn from(e: SendError<BrainCmd>) -> Self {
        BrainError::Brain(e)
    }
}
