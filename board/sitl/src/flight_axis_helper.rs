use std::str;

use anyhow::{bail, Context};
use quick_xml::name::QName;

const FLIGHT_AXIS_PORT: &str = "http://127.0.0.1:18083";

macro_rules! soap_payload {
    ($body:expr) => {
        format!(
            r#"<?xml version='1.0' encoding='UTF-8'?>
            <soap:Envelope
                xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/"
                xmlns:xsd="http://www.w3.org/2001/XMLSchema"
                xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
                    <soap:Body>{}</soap:Body>
            </soap:Envelope>"#,
            $body
        )
    };
}

pub struct FlightAxis {
    client: reqwest::Client,
    output: [i32; 12],

    controller_started: bool,

    pub state: State,
}

pub struct State {
    pub input: [i32; 12],
    pub flight_axis_controller_is_active: bool,
    pub reset_button_has_been_pressed: bool,
}

impl FlightAxis {
    pub fn new() -> Self {
        let default_input = [1500; 12];
        let mut default_output = [1500; 12];
        default_output[2] = 1000;
        Self {
            client: reqwest::Client::new(),
            output: default_output,

            controller_started: false,
            state: State {
                input: default_input,
                flight_axis_controller_is_active: false,
                reset_button_has_been_pressed: false,
            },
        }
    }

    pub fn set_output(&mut self, output: [i32; 12]) {
        self.output = output;
    }

    pub async fn update(&mut self) -> anyhow::Result<()> {
        if !self.controller_started
            || self.state.flight_axis_controller_is_active
            || self.state.reset_button_has_been_pressed
        {
            // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
            self.soap_restore_originial_controller_device().await?;

            // call a reset to ensure the aircraft is in a known state
            self.soap_reset_aircraft().await?;

            // call a inject to start the controller
            self.soap_inject_uav_control().await?;

            self.controller_started = true;
        }

        let scaled_ouput = self
            .output
            .iter()
            .map(|&x| (x as f64 - 1000.0) / 1000.0)
            .collect();

        let result = self.soap_exchange_data(&scaled_ouput).await?;

        let mut reader = quick_xml::Reader::from_str(&result);

        loop {
            match reader.read_event() {
                Ok(quick_xml::events::Event::Start(e)) => match e.name() {
                    QName(b"m-orientationQuaternion-W") => {
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            let text = e_text.unescape()?;
                        }
                    }
                    QName(b"m-channelValues-0to1") => {
                        for i in 0..12 {
                            match reader.read_event() {
                                Ok(quick_xml::events::Event::Start(e)) => {
                                    if e.name() == QName(b"item") {
                                        if let Ok(quick_xml::events::Event::Text(e_text)) =
                                            reader.read_event()
                                        {
                                            let text = e_text.unescape()?;
                                            let input =
                                                text.parse::<f64>().context("Failed to parse")?;

                                            self.state.input[i] = (input * 1000.0) as i32 + 1000;
                                        } else {
                                            bail!("Unexpected event {:?}", e);
                                        }

                                        if let Ok(quick_xml::events::Event::End(e)) =
                                            reader.read_event()
                                        {
                                            if e.name() != QName(b"item") {
                                                bail!("Unexpected event {:?}", e);
                                            }
                                        } else {
                                            bail!("Unexpected event");
                                        }
                                    } else {
                                        bail!("Unexpected event {:?}", e);
                                    }
                                }
                                Ok(quick_xml::events::Event::Eof) => {
                                    bail!("Unexpected EOF");
                                }
                                Err(e) => {
                                    bail!(
                                        "Error at position {}: {:?}",
                                        reader.buffer_position(),
                                        e
                                    );
                                }
                                _ => {
                                    bail!("Unexpected event");
                                }
                            }
                        }
                    }
                    name => {
                        // println!("Unknown element: {:?}", name);
                    }
                },
                Ok(quick_xml::events::Event::Eof) => break,
                Err(e) => bail!("Error at position {}: {:?}", reader.buffer_position(), e),
                _ => {}
            }
        }
        Ok(())
    }

    async fn soap_restore_originial_controller_device(&self) -> anyhow::Result<()> {
        let _res = self
            .send_soap_action("RestoreOriginalControllerDevice", 
            soap_payload!("<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>")
        )
            .await?;
        Ok(())
    }

    async fn soap_reset_aircraft(&self) -> anyhow::Result<()> {
        let _res = self
            .send_soap_action(
                "ResetAircraft",
                soap_payload!("<ResetAircraft><a>1</a><b>2</b></ResetAircraft>"),
            )
            .await?;
        Ok(())
    }

    async fn soap_inject_uav_control(&self) -> anyhow::Result<()> {
        let _res = self
            .send_soap_action(
                "InjectUAVControllerInterface",
                soap_payload!(
                    "<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>"
                ),
            )
            .await?;
        Ok(())
    }

    async fn soap_exchange_data(&self, data: &Vec<f64>) -> anyhow::Result<String> {
        let payload = soap_payload!(format!(
            r#"<ExchangeData>
                        <pControlInputs>
                            <m-selectedChannels>4095</m-selectedChannels>
                            <m-channelValues-0to1>
                            {}
                            </m-channelValues-0to1>
                        </pControlInputs>
                    </ExchangeData>"#,
            data.iter()
                .map(|x| format!("<item>{}</item>", x.to_string()))
                .collect::<Vec<String>>()
                .join("")
        ));
        self.send_soap_action("ExchangeData", payload).await
    }

    async fn send_soap_action(&self, soap_action: &str, payload: String) -> anyhow::Result<String> {
        let res = self
            .client
            .post(FLIGHT_AXIS_PORT)
            .header("content-type", "text/xml;charset='UTF-8'")
            .header("soapaction", soap_action)
            .header("connection", "Keep-Alive")
            .body(payload)
            .send()
            .await?;

        if res.status().is_success() {
            Ok(res.text().await?)
        } else {
            Ok(res.text().await?)
        }
    }
}
