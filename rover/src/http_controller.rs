use std::borrow::Borrow;

use embedded_svc::http::server::HandlerResult;
use embedded_svc::http::server::Request;
use embedded_svc::http::Method;
use esp_idf_svc::errors::EspIOError;
use esp_idf_svc::http::server::Configuration;
use esp_idf_svc::http::server::EspHttpConnection;
use esp_idf_svc::http::server::EspHttpServer;

use log::*;
use url::ParseError;
use url::Url;

use crate::brain::Brain;
use crate::brain::BrainCmd;

const HTML: &str = r#"<!DOCTYPE html>
    <html>
    <head>
    <meta name="viewport" content="width=device-width; initial-scale=3.0; maximum-scale=3.0; user-scalable=0;">
    <style>
    h1 {text-align: center;}
    p {text-align: center;}
    div {text-align: center;}
    </style>
    </head>
    <body>
    <form action="/" method="get">
      <br><br>
      <p><input type="submit" name="state" value="on" /></p>
      <br>
      <p><input type="submit" name="state" value="off" /></p>
    </form>
    </body>
    </html>
"#;

pub struct HttpController {
    _server: EspHttpServer,
}

impl HttpController {
    pub fn new(brain: Brain) -> Result<HttpController, EspIOError> {
        let config = Configuration::default();
        let mut server = EspHttpServer::new(&config)?;

        let b = brain.clone();
        server.fn_handler("/", Method::Get, move |request| {
            Self::handle_root(&b, request)
        })?;

        let b = brain.clone();
        server.fn_handler("/brain", Method::Get, move |request| {
            Self::handle_brain(&b, request)
        })?;

        server.fn_handler("/drive", Method::Get, move |request| {
            Self::handle_drive(&brain, request)
        })?;

        Ok(HttpController { _server: server })
    }

    fn parse_uri(uri: &str) -> Result<Url, ParseError> {
        Url::options()
            .base_url(Some(&Url::parse("http://localhost")?))
            .parse(uri)
    }
    fn value_to_bool(value: &str) -> bool {
        match value {
            "on" => true,
            "true" => true,
            "run" => true,
            "running" => true,
            "off" => false,
            "false" => false,
            "stop" => false,
            "stopped" => false,
            _ => false,
        }
    }

    fn handle_root(
        brain: &Brain,
        mut request: Request<&mut EspHttpConnection<'_>>,
    ) -> HandlerResult {
        let mut state: Option<bool> = None;
        let url = Self::parse_uri(request.connection().uri())?;
        for (n, v) in url.query_pairs() {
            info!("name={} value={}", n, v);
            if n == "state" {
                state = Some(Self::value_to_bool(v.borrow()));
            }
        }
        if let Some(value) = state {
            brain.send(crate::brain::BrainCmd::State(value))?;
            info!("handle_root: sending redirect");
            request.into_response(302, Some("Redirect"), &[("Location", "/")])?;
        } else {
            info!("handle_root: sending page");
            let mut response = request.into_ok_response()?;
            response.connection().write(HTML.as_bytes())?;
        }
        Ok(())
    }

    fn handle_brain(
        brain: &Brain,
        mut request: Request<&mut EspHttpConnection<'_>>,
    ) -> HandlerResult {
        let mut state = false;
        let url = Self::parse_uri(request.connection().uri())?;
        for (n, v) in url.query_pairs() {
            info!("name={} value={}", n, v);
            if n == "state" {
                state = Self::value_to_bool(v.borrow());
            }
        }
        brain.send(crate::brain::BrainCmd::State(state))?;
        request.into_ok_response()?;
        Ok(())
    }

    fn handle_drive(
        brain: &Brain,
        mut request: Request<&mut EspHttpConnection<'_>>,
    ) -> HandlerResult {
        let mut move_value: Option<i64> = None;
        let mut rotate_value: Option<i64> = None;
        let mut left: bool = false;
        let mut right: bool = false;
        let mut stop = false;
        let mut result: String = "OK".to_string();
        let url = Self::parse_uri(request.connection().uri())?;
        for (n, v) in url.query_pairs() {
            info!("name={} value={}", n, v);
            if n == "move" {
                if v == "stop" {
                    stop = true;
                } else {
                    move_value = Some(v.parse()?);
                }
            } else if n == "rotate" {
                if v == "stop" {
                    stop = true;
                } else {
                    rotate_value = Some(v.parse()?);
                }
            } else if n == "left" {
                left = true;
            } else if n == "right" {
                right = true;
            }
        }
        if stop {
            brain.send(BrainCmd::Move(0))?;
        } else if move_value.is_some() && rotate_value.is_some() {
            result = "Can't move and rotate at the same time!".to_string();
        } else if let Some(value) = move_value {
            if left {
                brain.send(BrainCmd::Left(value))?;
            }
            if right {
                brain.send(BrainCmd::Right(value))?;
            }
            if !left && !right {
                brain.send(BrainCmd::Move(value))?;
            }
        } else if let Some(value) = rotate_value {
            brain.send(BrainCmd::Rotate(value))?;
        } else if let Some(value) = rotate_value {
            brain.send(BrainCmd::Rotate(value))?;
        } else if let Some(value) = rotate_value {
            brain.send(BrainCmd::Rotate(value))?;
        }
        request
            .into_ok_response()?
            .connection()
            .write(format!("{result}\n").as_bytes())?;
        Ok(())
    }
}
