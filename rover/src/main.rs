use std::ffi::CStr;

use embedded_hal::delay::DelayUs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::netif::IpEvent;
use esp_idf_svc::wifi::WifiEvent;
use esp_idf_sys::esp_vfs_spiffs_conf_t;
use esp_idf_sys::esp_vfs_spiffs_register;
use esp_idf_sys::{esp, EspError};

use log::*;

use crate::brain::Brain;
use crate::config::Config;
use crate::factory::MotorFactory;
use crate::http_controller::HttpController;
use crate::logger::Logger;
use crate::mqtt::Mqtt;
use crate::network::Network;
use crate::peripherals::SystemPeripherals;

mod brain;
mod config;
mod factory;
mod http_controller;
mod logger;
mod mqtt;
mod network;
mod peripherals;

fn log_compile_info() {
    esp_idf_sys::esp_app_desc!();

    // WARNING: Bug workaround: time & date are flipped!
    let date = unsafe { CStr::from_ptr(esp_app_desc.time.as_ptr()) }.to_string_lossy();
    let time = unsafe { CStr::from_ptr(esp_app_desc.date.as_ptr()) }.to_string_lossy();

    let name = unsafe { CStr::from_ptr(esp_app_desc.project_name.as_ptr()) }.to_string_lossy();
    let version = unsafe { CStr::from_ptr(esp_app_desc.version.as_ptr()) }.to_string_lossy();
    let idf = unsafe { CStr::from_ptr(esp_app_desc.idf_ver.as_ptr()) }.to_string_lossy();

    info!("Running {name} v{version} @ IDF v{idf} - compiled {date} {time}");
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    Logger::initialize();
    log_compile_info();
    init_fs()?;
    let config = Config::new("/spiffs/config.json")?;

    info!("taking system peripherals");
    let peripherals = SystemPeripherals::take();

    info!("take the system event loop");
    let sysloop = EspSystemEventLoop::take()?;

    info!("setup network");
    let network = Network::new(
        peripherals.modem,
        sysloop.clone(),
        &config.wifi_ssid,
        &config.wifi_pass,
        &config.hostname,
    )?;

    // if wifi gets disconnected then reconnect.
    let _wifi_subs = {
        let network = network.clone();
        sysloop.subscribe(move |event: &WifiEvent| {
            info!("WIFI EVENT: {:+?}", event);
            #[allow(clippy::single_match)]
            match event {
                WifiEvent::StaDisconnected => {
                    network.connect().expect("start connection failed");
                }
                _ => {}
            }
        })?
    };

    let _ip_subs = sysloop.subscribe(|event: &IpEvent| {
        info!("IP EVENT: {:+?}", event);
        match event {
            IpEvent::ApStaIpAssigned(_) => {}
            IpEvent::DhcpIpAssigned(evt) => {
                info!("IP assigned: {}", evt.ip_settings.ip);
            }
            IpEvent::DhcpIp6Assigned(_) => {}
            IpEvent::DhcpIpDeassigned(_) => {}
        }
    })?;

    info!("setup MQTT");
    let mqtt = Mqtt::new(&config.mqtt_url, &config.mqtt_user, &config.mqtt_pass)?;
    Logger::set_mqtt(mqtt);

    // info!("setup i2c shared bus");
    // let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
    // let i2c = shared_bus::new_std!(
    //     I2cDriver = I2cDriver::new(
    //         peripherals.i2c_unit,
    //         peripherals.i2c_pins.sda,
    //         peripherals.i2c_pins.scl,
    //         &i2c_config
    //     )?
    // )
    // .expect("failed to create i2c shared bus");

    info!("setup motor factory");
    let motor_factory = MotorFactory::new(peripherals.ledc_timer)?;

    info!("setup lidar");
    let lidar = factory::lidar(peripherals.lidar)?;

    info!("setup motor/encoder/speed/position: left");
    let motor = motor_factory.motor("left", peripherals.left_motor, false)?;
    let encoder = factory::encoder(peripherals.left_encoder, false)?;
    let speed = factory::speed(motor, encoder)?;
    let left = factory::positon(speed)?;

    info!("setup motor/encoder/speed/position: right");
    let motor = motor_factory.motor("right", peripherals.right_motor, true)?;
    let encoder = factory::encoder(peripherals.right_encoder, true)?;
    let speed = factory::speed(motor, encoder)?;
    let right = factory::positon(speed)?;

    info!("setup the differential drive");
    let drive = factory::drive(left, right)?;

    info!("setup brain");
    let brain = Brain::new(lidar, drive)?;

    let _http = HttpController::new(brain)?;
    info!("server is up!");

    loop {
        delay.delay_ms(1000)?;
    }
}

fn init_fs() -> Result<(), EspError> {
    info!("configuring SPIFFS");
    let spiffs_config = esp_vfs_spiffs_conf_t {
        base_path: "/spiffs\0".as_ptr() as *const i8,
        partition_label: std::ptr::null(),
        max_files: 5,
        format_if_mount_failed: false,
    };
    esp!(unsafe { esp_vfs_spiffs_register(&spiffs_config) })
}
