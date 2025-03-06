use std::{
    str,
    sync::{mpsc, Arc, RwLock},
    thread::JoinHandle,
    time::Duration,
};

use anyhow::{bail, Context};
use quick_xml::{events::Event, name::QName};

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

#[derive(Debug, Clone)]
pub struct FlightAxisState {
    pub output: [i32; 12],
    pub input: [i32; 12],
    pub airspeed: f64,
    pub groundspeed: f64,
    pub altitude: f64,
    pub pitch_rate: f64,
    pub roll_rate: f64,
    pub yaw_rate: f64,
    pub acceleration_x: f64,
    pub acceleration_y: f64,
    pub acceleration_z: f64,
    pub battery_voltage: f64,
}

impl FlightAxisState {
    pub fn new() -> Self {
        let mut default_input = [1500; 12];
        default_input[2] = 1000;
        let mut default_output = [1500; 12];
        default_output[2] = 1000;
        Self {
            output: default_output,
            input: default_input,
            airspeed: 0.0,
            groundspeed: 0.0,
            altitude: 0.0,
            pitch_rate: 0.0,
            roll_rate: 0.0,
            yaw_rate: 0.0,
            acceleration_x: 0.0,
            acceleration_y: 0.0,
            acceleration_z: 0.0,
            battery_voltage: 0.0,
        }
    }
}

pub enum FlightAxisCommand {
    Stop,
}

pub struct FlightAxisWorker {
    client: reqwest::blocking::Client,
    controller_started: bool,
    flight_axis_controller_is_active: bool,
    reset_button_has_been_pressed: bool,
}

impl FlightAxisWorker {
    pub fn new() -> Self {
        Self {
            client: reqwest::blocking::Client::new(),
            controller_started: false,
            flight_axis_controller_is_active: false,
            reset_button_has_been_pressed: false,
        }
    }

    pub fn run(
        state: &mut Arc<RwLock<FlightAxisState>>,
        to_worker_rx: mpsc::Receiver<FlightAxisCommand>,
    ) -> JoinHandle<()> {
        let mut worker = Self::new();
        let state = state.clone();
        std::thread::spawn(move || {
            let mut last_loop_time = std::time::Instant::now();
            let minimum_elapsed_duration = std::time::Duration::from_millis(1000 / 200); // 200 Hz
            loop {
                match to_worker_rx.try_recv() {
                    Ok(FlightAxisCommand::Stop) => {
                        return;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        return;
                    }
                    _ => {}
                }
                let now = std::time::Instant::now();
                let dt = now.duration_since(last_loop_time);
                if dt < minimum_elapsed_duration {
                    std::thread::sleep(minimum_elapsed_duration - dt);
                    continue;
                }
                last_loop_time = now;

                let mut flight_axis_state = state.read().unwrap().clone();
                match worker.update(&mut flight_axis_state) {
                    Ok(_) => {
                        *state.write().unwrap() = flight_axis_state;
                    }
                    Err(e) => {
                        println!("Error: {:?}", e);
                        std::thread::sleep(Duration::from_secs(3));
                    }
                }
            }
        })
    }

    fn update(&mut self, state: &mut FlightAxisState) -> anyhow::Result<()> {
        if !self.controller_started
            || !self.flight_axis_controller_is_active
            || self.reset_button_has_been_pressed
        {
            println!("Restoring original controller device, resetting aircraft, and injecting UAV control");

            // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
            self.soap_restore_originial_controller_device()?;

            // call a reset to ensure the aircraft is in a known state
            self.soap_reset_aircraft()?;

            // call a inject to start the controller
            self.soap_inject_uav_control()?;

            self.controller_started = true;
        }

        let scaled_ouput = state
            .output
            .iter()
            .map(|&x| (x as f64 - 1000.0) / 1000.0)
            .collect();

        self.soap_exchange_data(&scaled_ouput, state)
    }

    fn soap_restore_originial_controller_device(&self) -> anyhow::Result<()> {
        let _res = self
            .send_soap_action("RestoreOriginalControllerDevice", 
            soap_payload!("<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>")
        )
            ?;
        Ok(())
    }

    fn soap_reset_aircraft(&self) -> anyhow::Result<()> {
        let _res = self.send_soap_action(
            "ResetAircraft",
            soap_payload!("<ResetAircraft><a>1</a><b>2</b></ResetAircraft>"),
        )?;
        Ok(())
    }

    fn soap_inject_uav_control(&self) -> anyhow::Result<()> {
        let _res = self.send_soap_action(
            "InjectUAVControllerInterface",
            soap_payload!(
                "<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>"
            ),
        )?;
        Ok(())
    }

    fn soap_exchange_data(
        &mut self,
        data: &Vec<f64>,
        state: &mut FlightAxisState,
    ) -> anyhow::Result<()> {
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

        let response = self.send_soap_action("ExchangeData", payload)?;

        self.parse_response(&response, state)
    }

    fn parse_response(
        &mut self,
        response: &str,
        state: &mut FlightAxisState,
    ) -> anyhow::Result<()> {
        let mut parser = quick_xml::Reader::from_str(response);
        let mut parsed_input = false;
        let mut parsed_aircraft_state = false;
        loop {
            match parser.read_event() {
                Ok(Event::Start(e)) => match e.name() {
                    QName(b"m-channelValues-0to1") => {
                        parsed_input = self.parse_rc_input(&mut parser, state)?;
                    }
                    QName(b"m-aircraftState") => {
                        parsed_aircraft_state = self.parse_aircraft_state(&mut parser, state)?;
                    }
                    QName(b"m-notifications") => match parser.read_event() {
                        Ok(Event::Start(_e)) => {
                            let value = parser.read_text(QName(b"m-resetButtonHasBeenPressed"))?;
                            if value == "true" {
                                println!("Reset button has been pressed: {}", value);
                            }
                            self.reset_button_has_been_pressed = value == "true";
                        }
                        _ => {}
                    },
                    _ => {}
                },
                Ok(Event::Eof) => {
                    if !parsed_input || !parsed_aircraft_state {
                        bail!("Failed to parse input or aircraft state");
                    }
                    break;
                }
                Err(e) => bail!("Error while parsing response: {:?}", e),
                _ => {}
            }
        }
        Ok(())
    }

    fn parse_rc_input(
        &mut self,
        parser: &mut quick_xml::Reader<&[u8]>,
        state: &mut FlightAxisState,
    ) -> anyhow::Result<bool> {
        let mut input = [0; 12];
        for i in 0..12 {
            match parser.read_event() {
                Ok(Event::Start(e)) => match e.name() {
                    QName(b"item") => {
                        if let Ok(Event::Text(e)) = parser.read_event() {
                            let value = str::from_utf8(&e.to_ascii_lowercase())?
                                .parse::<f64>()
                                .context("Failed to parse value")?;
                            input[i] = (value * 1000.0) as i32 + 1000;
                        }
                        // discard close tag
                        parser.read_event()?;
                    }
                    _ => {
                        bail!("Unexpected tag {:?}", e.name());
                    }
                },
                Err(e) => {
                    bail!("Error while parsing input: {:?}", e);
                }
                Ok(e) => {
                    bail!("Unexpected event {:?}", e);
                }
            }
        }
        state.input = input;
        Ok(true)
    }

    fn parse_aircraft_state(
        &mut self,
        parser: &mut quick_xml::Reader<&[u8]>,
        state: &mut FlightAxisState,
    ) -> anyhow::Result<bool> {
        let mut parsed_properties_count = 0;

        loop {
            match parser.read_event() {
                Ok(Event::Start(e)) => {
                    let name = e.name();
                    let value = parser.read_text(name)?;
                    match name {
                        QName(b"m-flightAxisControllerIsActive") => {
                            self.flight_axis_controller_is_active = value == "true";
                            parsed_properties_count += 1;
                        }
                        QName(b"m-airspeed-MPS") => {
                            state.airspeed = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-groundspeed-MPS") => {
                            state.groundspeed = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-altitudeAGL-MTR") => {
                            state.altitude = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-pitchRate-DEGpSEC") => {
                            state.pitch_rate = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-rollRate-DEGpSEC") => {
                            state.roll_rate = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-yawRate-DEGpSEC") => {
                            state.yaw_rate = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-accelerationBodyAX-MPS2") => {
                            state.acceleration_x = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-accelerationBodyAY-MPS2") => {
                            state.acceleration_y = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-accelerationBodyAZ-MPS2") => {
                            state.acceleration_z = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        QName(b"m-batteryVoltage-VOLTS") => {
                            state.battery_voltage = value.parse::<f64>()?;
                            parsed_properties_count += 1;
                        }
                        _ => {}
                    }
                }
                Err(e) => {
                    bail!("Error while parsing aircraft state: {:?}", e);
                }
                Ok(Event::End(e)) => match e.name() {
                    QName(b"m-aircraftState") => {
                        break;
                    }
                    _ => {}
                },
                _ => {}
            }
        }

        if parsed_properties_count == 11 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn send_soap_action(&self, soap_action: &str, payload: String) -> anyhow::Result<String> {
        let res = self
            .client
            .post(FLIGHT_AXIS_PORT)
            .header("content-type", "text/xml;charset='UTF-8'")
            .header("soapaction", soap_action)
            .header("connection", "Keep-Alive")
            .body(payload)
            .send()?;

        if res.status().is_success() {
            Ok(res.text()?)
        } else {
            Err(anyhow::anyhow!(
                "Failed to send SOAP action: {:?}",
                res.status()
            ))
        }
    }
}

pub struct FlightAxis {
    state: Arc<RwLock<FlightAxisState>>,
    to_worker_tx: Option<mpsc::Sender<FlightAxisCommand>>,
    worker_thread: Option<JoinHandle<()>>,
}

impl FlightAxis {
    pub fn new() -> Self {
        let state = Arc::new(RwLock::new(FlightAxisState::new()));
        Self {
            state,
            to_worker_tx: None,
            worker_thread: None,
        }
    }

    pub fn state(&self) -> FlightAxisState {
        self.state.read().unwrap().clone()
    }

    pub fn set_output(&mut self, output: [i32; 12]) {
        self.state.write().unwrap().output = output;
    }

    pub fn start(&mut self) {
        let (to_worker_tx, to_worker_rx) = mpsc::channel();
        self.worker_thread = Some(FlightAxisWorker::run(&mut self.state, to_worker_rx));
        self.to_worker_tx = Some(to_worker_tx);
    }

    pub fn stop(&mut self) {
        if let Some(tx) = &self.to_worker_tx {
            let _ = tx.send(FlightAxisCommand::Stop);
            self.to_worker_tx = None;
        }
        if let Some(handle) = self.worker_thread.take() {
            handle.join().unwrap();
        }
    }
}
