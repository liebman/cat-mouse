use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;

use embedded_hal::delay::DelayUs;

use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::AnyOutputPin;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::uart::UartRxDriver;
use lidar_ld19::LidarFrame;
use log::warn;

const SAMPLE_COUNT: usize = 360;

pub struct Lidar<'a> {
    power: PinDriver<'a, AnyOutputPin, Output>,
    data: Arc<Mutex<Data>>,
    running: Arc<AtomicBool>,
    synced: Arc<AtomicBool>,
}

#[derive(Debug)]
struct Data {
    samples: [u16; SAMPLE_COUNT],
}

impl Data {
    fn new() -> Self {
        Self {
            samples: [0u16; SAMPLE_COUNT],
        }
    }

    fn update(&mut self, frame: LidarFrame) {
        for point in frame.points {
            let index = match point.angle as usize / 100 {
                x if x >= self.samples.len() => self.samples.len() - 1,
                x => x,
            };
            self.samples[index] = point.distance;
        }
    }

    pub fn reset(&mut self) {
        for i in &mut self.samples {
            *i = 0;
        }
    }

    pub fn get_range_left(&self) -> u16 {
        let mut value = 12000u16;
        for dist in &self.samples[224..315] {
            if *dist < value {
                value = *dist;
            }
        }
        value
    }

    pub fn get_range_front(&self) -> u16 {
        let mut value = 12000u16;
        for dist in &self.samples[315..=359] {
            if *dist < value {
                value = *dist;
            }
        }
        for dist in &self.samples[0..45] {
            if *dist < value {
                value = *dist;
            }
        }
        value
    }

    pub fn get_range_right(&self) -> u16 {
        let mut value = 12000u16;
        for dist in &self.samples[45..135] {
            if *dist < value {
                value = *dist;
            }
        }
        value
    }

    pub fn get_frame(&self) -> Frame {
        Frame {
            range_left: self.get_range_left(),
            range_front: self.get_range_front(),
            range_right: self.get_range_right(),
        }
    }
}

#[derive(Debug)]
pub struct Frame {
    range_left: u16,
    range_front: u16,
    range_right: u16,
}

impl Frame {
    pub fn get_range_left(&self) -> u32 {
        self.range_left as u32
    }

    pub fn get_range_front(&self) -> u32 {
        self.range_front as u32
    }

    pub fn get_range_right(&self) -> u32 {
        self.range_right as u32
    }
}

impl<'a> Lidar<'a> {
    pub fn new(serial: UartRxDriver<'static>, power: AnyOutputPin) -> Self {
        let data = Arc::new(Mutex::new(Data::new()));
        let running = Arc::new(AtomicBool::new(false));
        let synced = Arc::new(AtomicBool::new(false));

        {
            let data = data.clone();
            let running = running.clone();
            let synced = synced.clone();
            _ = thread::Builder::new()
                .stack_size(8192)
                .name("lidar".into())
                .spawn(move || {
                    let mut ld19 = lidar_ld19::LidarLD19::new();
                    loop {
                        if !running.load(Ordering::SeqCst) {
                            synced.store(false, Ordering::SeqCst);
                            data.lock().unwrap().reset();
                            // wait for lidar to be enabled
                            while !running.load(Ordering::SeqCst) {
                                delay.delay_ms(100).unwrap();
                            }
                            // delay for lidar to get up to speed
                            delay.delay_ms(6000).unwrap();
                        }
                        let mut buffer = [0u8];
                        serial
                            .read(&mut buffer, TickType::from(Duration::from_millis(100)).0)
                            .expect("uart read failed!");
                        match ld19.add_byte(buffer[0]) {
                            Err(err) => warn!("lidar error: {err:?}"),
                            Ok(Some(frame)) => {
                                if !synced.load(Ordering::SeqCst) {
                                    synced.store(true, Ordering::SeqCst);
                                }
                                data.lock().unwrap().update(frame);
                            }
                            Ok(None) => {}
                        }
                    }
                })
                .unwrap();
        }

        // TODO: how can I set this high before making it an output.
        let mut power = PinDriver::output_od(power).unwrap();
        power.set_high().unwrap();

        Lidar {
            power,
            data,
            running,
            synced,
        }
    }

    pub fn set_power_on(&mut self) {
        self.power.set_low().unwrap();
        self.running.store(true, Ordering::SeqCst)
    }

    pub fn set_power_off(&mut self) {
        self.power.set_high().unwrap();
        self.running.store(false, Ordering::SeqCst)
    }

    pub fn set_power(&mut self, value: bool) {
        if value {
            self.set_power_on()
        } else {
            self.set_power_off()
        }
    }

    pub fn is_synced(&self) -> bool {
        self.synced.load(Ordering::SeqCst)
    }

    pub fn get_range_left(&self) -> u16 {
        self.data.lock().unwrap().get_range_left()
    }

    pub fn get_range_front(&self) -> u16 {
        self.data.lock().unwrap().get_range_front()
    }

    pub fn get_range_right(&self) -> u16 {
        self.data.lock().unwrap().get_range_right()
    }

    pub fn get_frame(&self) -> Frame {
        self.data.lock().unwrap().get_frame()
    }
}
