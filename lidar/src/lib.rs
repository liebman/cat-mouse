use std::fmt;
use std::fmt::Display;
use std::ptr;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;

use esp_idf_hal::task;
use esp_idf_sys::vTaskPrioritySet;
use esp_idf_sys::TaskHandle_t;
use esp_idf_sys::ESP_TASK_PRIO_MAX;
use log::*;

use proximity::Proximity;
use speed_control::SpeedControl;
use speed_control::SpeedControlCmd;

const DEGREES_PER_SECOND: f32 = 720.0; // two revolutions per second
const STEP_INTERVAL: f32 = 0.025;
const STEP_INTERVAL_MS: u64 = (STEP_INTERVAL * 1000.0) as u64;
const STEP_COUNT: usize = (360.0 / DEGREES_PER_SECOND / STEP_INTERVAL) as f32 as usize;
const STEP_DEGREES: f32 = 360.0 / STEP_COUNT as f32;

const BACK_THRESHOLD: u16 = 40; // less than this dist (mm) and we may be looking at back
const BACK_STEP_COUNT: usize = STEP_COUNT / 4 - 1;
const LEFT_START: usize = BACK_STEP_COUNT / 2 + 2;
const LEFT_END: usize = LEFT_START + BACK_STEP_COUNT;
const FRONT_START: usize = LEFT_END - 1;
const FRONT_END: usize = FRONT_START + BACK_STEP_COUNT;
const RIGHT_START: usize = FRONT_END - 1;
const RIGHT_END: usize = RIGHT_START + BACK_STEP_COUNT;

#[derive(Debug, Clone, Copy)]
pub enum LidarCmd {
    State(bool),
}

pub type LidarSender = Sender<LidarCmd>;

pub struct Lidar {
    data: Arc<Data>,
    tx: LidarSender,
}

#[derive(Debug, Clone, Copy)]
pub struct Frame {
    samples: [u16; STEP_COUNT],
}

impl Frame {
    fn get_range_for_slice(&self, samples: &[u16]) -> u32 {
        let mut range: u32 = u32::MAX;
        for r in samples {
            if (*r as u32) < range && *r > BACK_THRESHOLD {
                range = *r as u32;
            }
        }
        range
    }

    pub fn get_range_left(&self) -> u32 {
        self.get_range_for_slice(&self.samples[LEFT_START..=LEFT_END])
    }

    pub fn get_range_front(&self) -> u32 {
        self.get_range_for_slice(&self.samples[FRONT_START..=FRONT_END])
    }

    pub fn get_range_right(&self) -> u32 {
        self.get_range_for_slice(&self.samples[RIGHT_START..=RIGHT_END])
    }
}

impl Display for Frame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[")?;
        let mut prefix = "";
        for sample in self.samples {
            write!(f, "{prefix}{sample:4}")?;
            if prefix.is_empty() {
                prefix = " ";
            }
        }
        write!(f, "]")
    }
}

struct Data {
    active: AtomicBool,
    raw_values: Mutex<[u16; STEP_COUNT]>,
    values: Mutex<[u16; STEP_COUNT]>,
    tx_list: Arc<Mutex<Vec<Sender<Frame>>>>,
}

#[allow(dead_code)]
impl Lidar {
    pub fn new(motor: SpeedControl<'static>, proximity: impl Proximity + Send + 'static) -> Lidar {
        info!("step interval: {STEP_INTERVAL} interval_ms: {STEP_INTERVAL_MS} count: {STEP_COUNT} degrees: {STEP_DEGREES}");
        let data = Arc::new(Data {
            active: AtomicBool::new(false),
            raw_values: Mutex::new([0; STEP_COUNT]),
            values: Mutex::new([0; STEP_COUNT]),
            tx_list: Arc::new(Mutex::new(Vec::new())),
        });

        let (tx, rx) = mpsc::channel();

        {
            let data = data.clone();
            _ = thread::Builder::new()
                .stack_size(4096)
                .name("lidar".into())
                .spawn(move || {
                    unsafe {
                        vTaskPrioritySet(ptr::null_mut() as TaskHandle_t, ESP_TASK_PRIO_MAX - 3);
                    }
                    Self::process(motor, proximity, data, rx);
                })
                .unwrap();
        }

        Lidar { data, tx }
    }

    #[allow(dead_code)]
    fn process(
        motor: SpeedControl,
        mut proximity: impl Proximity + Send,
        data: Arc<Data>,
        rx: Receiver<LidarCmd>,
    ) {
        proximity.subscribe();
        let mut position = 0;
        loop {
            let nv = task::wait_notification(Some(Duration::from_millis(100)));
            if let Ok(cmd) = rx.try_recv() {
                info!("recieved command: {cmd:?}");
                match cmd {
                    LidarCmd::State(value) => {
                        if value {
                            // start the motor at one revolution per second
                            motor
                                .send(SpeedControlCmd::SetSpeed(DEGREES_PER_SECOND))
                                .unwrap();
                            // set the sampling interval
                            retry::retry(retry::delay::Fixed::from_millis(10).take(2), || {
                                proximity.set_interval(Duration::from_millis(STEP_INTERVAL_MS))
                            })
                            .unwrap();
                        } else {
                            motor.send(SpeedControlCmd::SetSpeed(0.0)).unwrap();
                        }
                        data.active.store(value, Ordering::Relaxed);
                    }
                }
            }
            if nv.is_none() || !data.active.load(Ordering::Relaxed) {
                continue;
            }
            let distance =
                match retry::retry(
                    retry::delay::Fixed::from_millis(5).take(10),
                    || match proximity.get_proximity() {
                        Ok(x) if x > 0 => Ok(x),
                        Ok(x) => Err(LidarError::InvalidDistance(x)),
                        Err(e) => Err(e.into()),
                    },
                ) {
                    Ok(x) => x,
                    Err(e) => {
                        match e.error {
                            LidarError::InvalidDistance(x) if x == 0 => {}
                            _ => error!("failed to fetch proximity value '{e}' using 0"),
                        }
                        0
                    }
                };
            data.raw_values.lock().unwrap()[position] = distance;
            position += 1;
            if position >= STEP_COUNT {
                position = 0;
                let mut values = [0u16; STEP_COUNT];
                values.copy_from_slice(data.raw_values.lock().unwrap().as_slice());
                if Self::fix_range(&mut values) {
                    data.values
                        .lock()
                        .unwrap()
                        .copy_from_slice(values.as_slice());
                } else {
                    warn!("failed to find back: {:?}", values);
                }

                for tx in data.tx_list.lock().unwrap().as_slice() {
                    tx.send(Frame { samples: values }).unwrap();
                }
            }
        }
    }

    pub fn get_values(&self) -> Vec<u16> {
        let mut values = vec![0u16; STEP_COUNT];
        values.copy_from_slice(self.data.values.lock().unwrap().as_slice());
        values
    }

    pub fn get_raw_values(&self) -> Vec<u16> {
        let mut values = vec![0u16; STEP_COUNT];
        values.copy_from_slice(self.data.raw_values.lock().unwrap().as_slice());
        values
    }

    // find the length of the run of values less than the back threshold
    fn find_length(data: &[u16], start: usize) -> usize {
        let mut pos = start;
        let mut len: usize = 0;
        while data[pos] <= BACK_THRESHOLD {
            pos += 1;
            len += 1;
            if pos >= STEP_COUNT {
                pos = 0;
            }
            if len > BACK_STEP_COUNT + 2 {
                break;
            }
        }
        len
    }

    // find the start of the longest run of values less than the back threshold
    fn find_start(data: &[u16]) -> (usize, usize) {
        let mut start: usize = 0;
        let mut length: usize = 0;
        for step in 0..STEP_COUNT - 1 {
            let l = Self::find_length(data, step);
            if l > length {
                start = step;
                length = l;
            }
        }
        (start, length)
    }

    fn fix_range(data: &mut [u16]) -> bool {
        let (start, length) = Self::find_start(data);
        if length < 2 {
            return false;
        }
        let mut center = start + length / 2;

        if center >= STEP_COUNT {
            center -= STEP_COUNT;
        }

        data.rotate_left(center);
        true
    }

    pub fn is_active(&self) -> bool {
        self.data.active.load(Ordering::Relaxed)
    }

    pub fn subscribe(&mut self) -> Receiver<Frame> {
        let (tx, rx): (Sender<Frame>, Receiver<Frame>) = mpsc::channel();
        self.data.tx_list.lock().unwrap().push(tx);
        rx
    }

    pub fn sender(&self) -> LidarSender {
        self.tx.clone()
    }
}

#[derive(Debug)]
pub enum LidarError {
    InvalidDistance(u16),
    Other(Box<dyn std::error::Error>),
}
impl Display for LidarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}
impl std::error::Error for LidarError {}

impl From<Box<dyn std::error::Error>> for LidarError {
    fn from(e: Box<dyn std::error::Error>) -> Self {
        LidarError::Other(e)
    }
}
