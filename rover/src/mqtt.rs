use std::io::stdout;
use std::io::Write;
use std::time::Duration;

use embedded_svc::mqtt::client::Event;
use embedded_svc::mqtt::client::MessageId;
use embedded_svc::mqtt::client::QoS;
use esp_idf_svc::mqtt::client::*;
use esp_idf_sys::EspError;

#[allow(dead_code)]
pub struct Mqtt {
    client: EspMqttClient<()>,
}

#[allow(dead_code)]
impl Mqtt {
    pub fn new(url: &str, user: &str, pass: &str) -> Result<Mqtt, EspError> {
        let conf = MqttClientConfiguration {
            username: Some(user),
            password: Some(pass),
            reconnect_timeout: Some(Duration::from_millis(3000)),
            task_stack: 4096,
            ..Default::default()
        };
        let client = EspMqttClient::new(
            url,
            &conf,
            |data: &'_ Result<Event<EspMqttMessage<'_>>, EspError>| {
                match data {
                    Ok(msg) => {
                        stdout()
                            .write_all(format!("MQTT MESSAGE: {msg:?}\n").as_bytes())
                            .unwrap();
                        stdout().flush()
                    }
                    Err(_) => Ok(()),
                }
                .unwrap();
            },
        )?;

        Ok(Mqtt { client })
    }

    pub fn publish(&mut self, topic: &str, bytes: &[u8], qos: QoS) -> Result<MessageId, EspError> {
        self.client.publish(topic, qos, false, bytes)
    }
}
