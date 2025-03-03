use std::{f64, str};

use sitl::util::quaternion_to_euler;

pub struct FlightAxis {
    client: reqwest::Client,
    input: [f32; 12],
}

const FLIGHT_AXIS_PORT: &str = "http://127.0.0.1:18083";

impl FlightAxis {
    pub fn set_throttle(&mut self, throttle: f32) {
        self.input[2] = throttle;
    }

    pub async fn connect() -> anyhow::Result<Self> {
        let mut input = [0.5; 12];
        input[2] = 0.0;
        let f = Self {
            client: reqwest::Client::new(),
            input,
        };

        match f.enable_rc().await {
            Ok(_) => {}
            Err(err) => {
                eprintln!("Failed to enable RC: {:?}", err);
            }
        }

        f.disable_rc().await?;

        Ok(f)
    }

    // Set Spektrum as the RC input
    // Return True if the response was OK
    async fn enable_rc(&self) -> anyhow::Result<bool> {
        let soap_body = r#"<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>"#;
        let _res = self
            .send_soap_action("RestoreOriginalControllerDevice", soap_body)
            .await?;
        Ok(true)
    }

    // Disable Spektrum as the RC input, and use FlightAxis instead
    // Return True if the response was OK
    async fn disable_rc(&self) -> anyhow::Result<bool> {
        let soap_body =
            r#"<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>"#;
        let _res = self
            .send_soap_action("InjectUAVControllerInterface", soap_body)
            .await?;
        Ok(true)
    }

    // Set the control inputs, and get the states
    // Return True if the response was OK
    pub async fn get_state(&self) -> anyhow::Result<(f64, f64, f64)> {
        let soap_body = create_soap_body_exchange_data_from_rc_channels(&self.input);
        let xml = self.send_soap_action("ExchangeData", &soap_body).await?;

        let mut reader = quick_xml::Reader::from_str(&xml);
        reader.trim_text(true);
        let mut w: Option<f64> = None;
        let mut x: Option<f64> = None;
        let mut y: Option<f64> = None;
        let mut z: Option<f64> = None;

        loop {
            match reader.read_event() {
                Ok(quick_xml::events::Event::Start(ref e)) => match e.name() {
                    quick_xml::name::QName(b"m-orientationQuaternion-W") => {
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            w = Some(str::from_utf8(&e_text.into_inner())?.parse()?);
                        }
                    }
                    quick_xml::name::QName(b"m-orientationQuaternion-X") => {
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            x = Some(str::from_utf8(&e_text.into_inner())?.parse()?);
                        }
                    }
                    quick_xml::name::QName(b"m-orientationQuaternion-Y") => {
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            y = Some(str::from_utf8(&e_text.into_inner())?.parse()?);
                        }
                    }
                    quick_xml::name::QName(b"m-orientationQuaternion-Z") => {
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            z = Some(str::from_utf8(&e_text.into_inner())?.parse()?);
                        }
                    }
                    quick_xml::name::QName(name) => {
                        let name = str::from_utf8(name)?;
                        if let Ok(quick_xml::events::Event::Text(e_text)) = reader.read_event() {
                            println!("{}: {}", name, str::from_utf8(&e_text.into_inner())?);
                        } else {
                            println!("{}: None", name);
                        }
                    }
                },
                Ok(quick_xml::events::Event::Eof) => break,
                Err(e) => {
                    return Err(e.into());
                }
                _ => {}
            }
        }
        if let (Some(w), Some(x), Some(y), Some(z)) = (w, x, y, z) {
            let (roll, pitch, yaw) = quaternion_to_euler((w, x, y, z));
            Ok((roll, pitch, yaw))
        } else {
            Err(anyhow::anyhow!("Failed to get state"))
        }
    }

    async fn send_soap_action(&self, soap_action: &str, soap_body: &str) -> anyhow::Result<String> {
        let full_soap_body = format!(
            r#"<?xml version='1.0' encoding='UTF-8'?>
    <soap:Envelope xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/"
                   xmlns:xsd="http://www.w3.org/2001/XMLSchema"
                   xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <soap:Body>
        {}
      </soap:Body>
    </soap:Envelope>"#,
            soap_body
        );
        let res = self
            .client
            .post(FLIGHT_AXIS_PORT)
            .header("content-type", "text/xml;charset='UTF-8'")
            .header("soapaction", soap_action)
            .header("connection", "Keep-Alive")
            .body(full_soap_body)
            .send()
            .await?;

        if res.status().is_success() {
            Ok(res.text().await?)
        } else {
            Ok(res.text().await?)
        }
    }
}

fn create_soap_body_exchange_data_from_rc_channels(rc_channels: &[f32; 12]) -> String {
    format!(
        r#"
        <ExchangeData>
            <pControlInputs>
                <m-selectedChannels>4095</m-selectedChannels>
                <m-channelValues-0to1>
                    <item>{0}</item>
                    <item>{1}</item>
                    <item>{2}</item>
                    <item>{3}</item>
                    <item>{4}</item>
                    <item>{5}</item>
                    <item>{6}</item>
                    <item>{7}</item>
                    <item>{8}</item>
                    <item>{9}</item>
                    <item>{10}</item>
                    <item>{11}</item>
                </m-channelValues-0to1>
            </pControlInputs>
        </ExchangeData>"#,
        rc_channels[0],
        rc_channels[1],
        rc_channels[2],
        rc_channels[3],
        rc_channels[4],
        rc_channels[5],
        rc_channels[6],
        rc_channels[7],
        rc_channels[8],
        rc_channels[9],
        rc_channels[10],
        rc_channels[11]
    )
}
