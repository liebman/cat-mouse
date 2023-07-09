use std::f64::consts::PI;
use std::ptr;
use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::SendError;
use std::sync::mpsc::Sender;
use std::thread;

use esp_idf_sys::{vTaskPrioritySet, TaskHandle_t, ESP_TASK_PRIO_MAX};
use log::*;

use wheel::Wheel;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum DriveCmd {
    Drive((f32, f32, f32)),
    Rotate(i64),
    Move(i64),
    Stop,
}

#[allow(dead_code)]
#[derive(Clone)]
pub struct Drive<'d> {
    tx: Sender<DriveCmd>,
    left_wheel: Wheel<'d>,
    right_wheel: Wheel<'d>,
}

#[allow(dead_code)]
impl<'d> Drive<'d> {
    pub fn new(
        left_wheel: Wheel<'static>,
        right_wheel: Wheel<'static>,
        wheel_dist: i32,
    ) -> Result<Self, std::io::Error> {
        let (tx, rx): (Sender<DriveCmd>, Receiver<DriveCmd>) = channel();
        {
            let left_wheel = left_wheel.clone();
            let right_wheel = right_wheel.clone();
            thread::Builder::new().stack_size(4096).spawn(move || {
                unsafe { vTaskPrioritySet(ptr::null_mut() as TaskHandle_t, ESP_TASK_PRIO_MAX / 2) }

                loop {
                    match rx.recv().unwrap() {
                        DriveCmd::Move(distance) => {
                            info!("Move {distance}");
                            let pos_left = left_wheel.get_position() + distance;
                            let pos_right = right_wheel.get_position() + distance;
                            left_wheel.set_position(pos_left).unwrap();
                            right_wheel.set_position(pos_right).unwrap();
                        }
                        DriveCmd::Rotate(degrees) => {
                            info!("Rotate {degrees}");
                            // turn by using only one wheel (yes - it throws us out of position slightly)
                            let degrees_per_mm = 360.0 / (wheel_dist as f64 * PI);
                            let distance = (degrees as f64 / degrees_per_mm) as i64;
                            if degrees > 0 {
                                let pos_left = left_wheel.get_position() + distance;
                                left_wheel.set_position(pos_left).unwrap();
                            } else {
                                let pos_right = right_wheel.get_position() + distance.abs();
                                right_wheel.set_position(pos_right).unwrap();
                            }
                        }
                        DriveCmd::Drive((_translational, _angular, _distance)) => {
                            warn!("Drive not implemented!");
                            //  let speed_diff  = angular * (wheel_dist / 2.0);
                            //  let speed_left  = (translational - speed_diff) * degrees_per_mm;
                            //  let speed_right = (translational + speed_diff) * degrees_per_mm;
                            // let mut dist_diff = 0f32;
                            // if angular != 0.0 {
                            //     if translational != 0.0 {
                            //         let radius = translational / angular;
                            //         let phi    = distance / radius;
                            //         dist_diff    = phi * (wheel_dist / 2.0);
                            //     } else {
                            //         dist_diff = angular/2.0 * (wheel_dist / 2.0);
                            //     }
                            // }
                            // let dist_left  = distance - dist_diff;
                            // let dist_right = distance + dist_diff;
                            // info!("left={dist_left} right={dist_right}");
                            // let deg_left  = Degrees::from(dist_left  * degrees_per_mm);
                            // let deg_right = Degrees::from(dist_right * degrees_per_mm);

                            // let pos_left = _left_encoder.get_position() + deg_left;
                            // let pos_right = _right_encoder.get_position() + deg_right;
                            // left_wheel.send(PositionControlCmd::SetPosition(pos_left)).unwrap();
                            // _right_sender.send(PositionControlCmd::SetPosition(pos_right)).unwrap();
                        }
                        DriveCmd::Stop => {
                            left_wheel.stop().unwrap();
                            right_wheel.stop().unwrap();
                        }
                    }
                }
            })?;
        }
        Ok(Drive {
            tx,
            left_wheel,
            right_wheel,
        })
    }

    pub fn is_active(&self) -> bool {
        self.left_wheel.is_active() || self.right_wheel.is_active()
    }

    pub fn send(&self, cmd: DriveCmd) -> Result<(), SendError<DriveCmd>> {
        self.tx.send(cmd)
    }
}
