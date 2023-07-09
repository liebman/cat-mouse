use std::f64::consts::PI;

use log::*;

use position_control::PositionControl;
use position_control::PositionControlCmd;
use position_control::PositionControlError;

#[derive(Clone)]
pub struct Wheel<'d> {
    degrees_per_mm: f64,
    position: PositionControl<'d>,
}

#[allow(dead_code)]
impl<'d> Wheel<'d> {
    pub fn new(
        position: PositionControl,
        diameter: i32, // diameter in mm
    ) -> Wheel {
        let degrees_per_mm = 360.0 / (diameter as f64 * PI);
        info!("degrees_per_mm: {degrees_per_mm}");
        Wheel {
            degrees_per_mm,
            position,
        }
    }

    pub fn set_pid(&self, p: f64, i: f64, d: f64) -> Result<(), PositionControlError> {
        self.position.send(PositionControlCmd::SetPid((p, i, d)))?;
        Ok(())
    }

    pub fn get_position(&self) -> i64 {
        (self.position.get_position() as f64 / self.degrees_per_mm) as i64
    }

    pub fn is_active(&self) -> bool {
        self.position.is_active()
    }

    pub fn stop(&self) -> Result<(), PositionControlError> {
        self.position.send(PositionControlCmd::Stop)?;
        Ok(())
    }

    pub fn set_position(&self, position: i64) -> Result<(), PositionControlError> {
        let pos = (position as f64 * self.degrees_per_mm) as i64;
        info!("set position: {pos} ({position})",);
        self.position.send(PositionControlCmd::SetPosition(pos))?;
        Ok(())
    }
}
