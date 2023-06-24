use std::time::Duration;

pub trait Proximity {
    // set the expected interval between measurements
    fn set_interval(&mut self, interval: Duration) -> Result<(), Box<dyn std::error::Error>>;
    // get a proximity measurement in millimeters
    fn get_proximity(&mut self) -> Result<u16, Box<dyn std::error::Error>>;
    // subscribe calling thread for Task notifications when data is available
    fn subscribe(&mut self);
}
