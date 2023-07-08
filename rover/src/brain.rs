use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::mpsc::channel;
use std::sync::mpsc::SendError;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use differential_drive::DriveCmd;

use log::*;

use differential_drive::Drive;
use lidar::Lidar;

use crate::brain::simple::Simple;

mod simple;

#[derive(Debug, Clone, Copy)]
pub enum BrainCmd {
    State(bool), // on/off
    Move(i64),
    Rotate(i64),
    LidarOnOff(bool),
}

#[derive(Debug, Clone)]
pub struct Brain {
    tx: Sender<BrainCmd>,
}

impl Brain {
    pub fn new(mut lidar: Lidar<'static>, drive: Drive<'static>) -> Result<Brain, std::io::Error> {
        let active = Arc::new(AtomicBool::new(false));
        let (tx, cmd_rx) = channel();
        {
            let brain_tx = tx.clone();
            thread::Builder::new()
                .stack_size(6144)
                .name("brain".into())
                .spawn(move || {
                    let simpleton = Simple::start(brain_tx, drive.clone()).unwrap();
                    loop {
                        if let Ok(cmd) = cmd_rx.recv_timeout(Duration::from_millis(250)) {
                            match cmd {
                                BrainCmd::State(value) => {
                                    info!("brain({value})");
                                    active.store(value, Ordering::Relaxed);
                                    match value {
                                        true => simpleton.send(simple::Event::Start),
                                        false => simpleton.send(simple::Event::Stop),
                                    }
                                    .unwrap();
                                }
                                BrainCmd::Move(distance) => {
                                    let drive_cmd = match distance {
                                        0 => DriveCmd::Stop,
                                        _ => DriveCmd::Move(distance),
                                    };
                                    info!("move({drive_cmd:?})");
                                    if let Err(err) = drive.send(drive_cmd) {
                                        error!("failed to send move command: {err}");
                                    }
                                }
                                BrainCmd::Rotate(degrees) => {
                                    let drive_cmd = match degrees {
                                        0 => DriveCmd::Stop,
                                        _ => DriveCmd::Rotate(degrees),
                                    };
                                    info!("rotate({drive_cmd:?})");
                                    if let Err(err) = drive.send(drive_cmd) {
                                        error!("failed to send rotate command: {err}");
                                    }
                                }
                                BrainCmd::LidarOnOff(value) => {
                                    info!("lidar on/off({value:?})");
                                    lidar.set_power(value)
                                }
                            }
                        }
                        simpleton
                            .send(simple::Event::Frame(lidar.get_frame()))
                            .unwrap();
                        // if !active.load(Ordering::Relaxed) {
                        //     continue;
                        // }
                        // let left = frame.get_range_left();
                        // let front = frame.get_range_front();
                        // let right = frame.get_range_right();
                        // info!("l/f/r: {left:.2} / {front:.2} / {right:.2} {frame}");
                    }
                })?;
        }
        Ok(Brain { tx })
    }

    pub fn send(&self, cmd: BrainCmd) -> Result<(), SendError<BrainCmd>> {
        self.tx.send(cmd)
    }
}
