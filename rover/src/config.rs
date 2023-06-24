use std::fs::File;

use log::*;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde()]
#[allow(dead_code)]
pub struct Config {
    #[serde(default = "default_hostname")]
    pub hostname: String,
    #[serde(default)]
    pub wifi_ssid: String,
    #[serde(default)]
    pub wifi_pass: String,
    #[serde(default)]
    pub mqtt_url: String,
    #[serde(default)]
    pub mqtt_user: String,
    #[serde(default)]
    pub mqtt_pass: String,
}

fn default_hostname() -> String {
    "cat-mouse-unknown".to_string()
}

impl Config {
    pub fn new(file_name: &str) -> anyhow::Result<Self> {
        info!("opening {file_name}");
        let file = File::open(file_name)?;
        let config: Config = serde_json::from_reader(file)?;

        info!("config: {:?}", config);

        Ok(config)
    }
}
