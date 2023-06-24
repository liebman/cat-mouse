use std::io::stdout;
use std::io::Write;
use std::sync::Mutex;

use embedded_svc::mqtt::client::QoS;
use log::set_logger;
use log::set_max_level;
use log::Level;
use log::LevelFilter;
use log::Log;
use log::Record;

use esp_idf_sys::esp_log_timestamp;

use crate::mqtt::Mqtt;

static LOGGER: Logger = Logger {
    mqtt: Mutex::new(None),
};

pub struct Logger {
    mqtt: Mutex<Option<Mqtt>>,
}

unsafe impl Sync for Logger {}
unsafe impl Send for Logger {}

#[allow(dead_code)]
impl Logger {
    pub fn initialize() {
        set_max_level(LevelFilter::Debug);
        set_logger(&LOGGER).unwrap();
    }

    pub fn set_mqtt(mqtt: Mqtt) {
        let mut guard = LOGGER.mqtt.lock().unwrap();
        *guard = Some(mqtt);
    }

    fn get_color(level: Level) -> Option<u8> {
        match level {
            Level::Error => Some(31), // LOG_COLOR_RED
            Level::Warn => Some(33),  // LOG_COLOR_BROWN
            Level::Info => Some(32),  // LOG_COLOR_GREEN,
            _ => None,
        }
    }

    fn add_color_for_level(s: &str, level: Level) -> String {
        if let Some(color) = Self::get_color(level) {
            format!("\x1b[0;{color}m{s}\x1b[0m")
        } else {
            s.to_string()
        }
    }

    fn fmt_record(record: &Record) -> String {
        format!(
            "{} ({}) {} {}",
            record.level(),
            unsafe { esp_log_timestamp() },
            record.metadata().target(),
            record.args()
        )
    }
}

impl Log for Logger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        if let Some(level) = ::log::max_level().to_level() {
            metadata.level() <= level
        } else {
            false
        }
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let msg = Self::fmt_record(record);
            let msg_color = Self::add_color_for_level(&msg, record.level());
            {
                let mut stdout = stdout().lock();
                stdout.write_all(msg_color.as_bytes()).unwrap();
                stdout.write_all("\n".as_bytes()).unwrap();
                stdout.flush().unwrap();
            }
            match self.mqtt.lock() {
                Ok(mut maybe_mqtt) => {
                    if let Some(mqtt) = maybe_mqtt.as_mut() {
                        if let Err(e) =
                            mqtt.publish("cat-mouse/log", msg.as_bytes(), QoS::AtMostOnce)
                        {
                            stdout()
                                .write_all(
                                    format!("LOGGING ERROR: failed to send message via MQTT: {e}")
                                        .as_bytes(),
                                )
                                .unwrap();
                        }
                    }
                }
                Err(_) => {
                    stdout()
                        .write_all("LOGGING ERROR: failed to lock MQTT".as_bytes())
                        .unwrap();
                }
            };
        }
    }

    fn flush(&self) {
        stdout().flush().unwrap();
    }
}
