use std::sync::Arc;
use std::sync::Mutex;

use embedded_svc::wifi::AuthMethod;
use embedded_svc::wifi::ClientConfiguration;
use embedded_svc::wifi::Configuration;
use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::mdns::EspMdns;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::BlockingWifi;
use esp_idf_svc::wifi::EspWifi;
use esp_idf_sys::EspError;
use log::*;

#[derive(Clone)]
pub struct Network<'d> {
    wifi: Arc<Mutex<BlockingWifi<EspWifi<'d>>>>,
    _mdns: Arc<Mutex<EspMdns>>,
}

impl<'d> Network<'d> {
    pub fn new(
        modem: impl Peripheral<P = impl WifiModemPeripheral + 'd> + 'd,
        sysloop: EspSystemEventLoop,
        ssid: &str,
        psk: &str,
        hostname: &str,
    ) -> Result<Self, NetworkError> {
        let mut auth_method = AuthMethod::WPA2Personal;
        if ssid.is_empty() {
            return Err(NetworkError::Config("No SSID specified!".to_string()));
        }
        if psk.is_empty() {
            auth_method = AuthMethod::None;
            warn!("Wifi password is empty");
        }
        let nvs = EspDefaultNvsPartition::take()?;
        let mut wifi = BlockingWifi::wrap(
            EspWifi::new(modem, sysloop.clone(), Some(nvs))?,
            sysloop,
        )?;
        info!("setting Wifi configuration");
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: ssid.into(),
            password: psk.into(),
            auth_method,
            ..Default::default()
        }))?;

        info!("starting Wifi");
        wifi.start()?;

        info!("connecting Wifi");
        wifi.connect()?;
        info!("Wifi connected!");

        info!("wait for ip address");
        wifi.wait_netif_up()?;

        // add mdns
        let mut mdns = EspMdns::take()?;
        mdns.set_hostname(hostname)?;
        mdns.set_instance_name(hostname)?;
        mdns.add_service(None, "_http", "_tcp", 80, &[])?;

        Ok(Self {
            wifi: Arc::new(Mutex::new(wifi)),
            _mdns: Arc::new(Mutex::new(mdns)),
        })
    }

    pub fn connect(&self) -> Result<(), EspError> {
        self.wifi.lock().unwrap().connect()
    }
}

#[derive(Debug)]
pub enum NetworkError {
    EspError(EspError),
    Config(String),
}

impl std::fmt::Display for NetworkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))
    }
}

impl std::error::Error for NetworkError {}

impl From<EspError> for NetworkError {
    fn from(e: EspError) -> Self {
        NetworkError::EspError(e)
    }
}
