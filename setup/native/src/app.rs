use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail, ensure};
use hebbian_so101::commissioning::{
    ARM_IDS, ArmFiles, HOME_GRIPPER, HOME_JOINTS, MOTOR_NAMES, Session, SessionView, Stage,
    program_motor_id, restore_servos,
};
use hebbian_so101::discovery::{ControllerMatch, ControllerSerial, UsbController, discover};
use hebbian_so101::driver::SerialBus;
use hebbian_so101::setup::{ModelVariant, MotorIds};
use hebbian_so101::telemetry::CommunicationFailure;
use serde::{Deserialize, Serialize};
use serialport::SerialPortType;

#[derive(Deserialize, Serialize)]
#[serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)]
pub enum Action {
    Connect {
        serial: String,
    },
    Begin {
        serial: String,
        skip_reset: bool,
    },
    Disconnect,
    ResetServos,
    SkipServoReset,
    Restart,
    Reopen,
    StartSweep {
        joint_index: Option<usize>,
    },
    FinishSweep,
    MapMotorIds {
        motor_ids: MotorIds,
    },
    CheckMotorOrder,
    ApplyDetectedMotorOrder,
    ReverseJoint {
        index: usize,
    },
    AdjustJoint {
        index: usize,
        delta_radians: f64,
    },
    Save {
        model_verified: bool,
        review_revision: u64,
    },
}

#[derive(Deserialize, Serialize)]
pub struct ProgramMotor {
    pub port: String,
    pub motor_id: u8,
    pub isolated_motor: bool,
}

#[derive(Serialize)]
pub struct ControllerChoice {
    #[serde(flatten)]
    controller: UsbController,
    registered: bool,
}

#[derive(Serialize)]
pub struct View {
    #[serde(flatten)]
    session: SessionView,
    serial: String,
    controllers: Vec<ControllerChoice>,
    model_variant: ModelVariant,
    motor_ids: MotorIds,
    offer_servo_reset: bool,
    servo_reset_incomplete: bool,
    checking_motor_order: bool,
    detected_base_motor: Option<u8>,
    review_revision: Option<u64>,
    home_joints_radians: [f64; 5],
    home_gripper_radians: f64,
}

#[derive(Debug, Serialize)]
pub struct ProgrammingPort {
    port: String,
    description: String,
}

#[derive(Default)]
enum ResetDecision {
    #[default]
    Pending,
    Decided,
}

pub struct Application {
    config: PathBuf,
    calibration_root: PathBuf,
    model: ModelVariant,
    auto_connect: bool,
    session: Option<Session<SerialBus>>,
    revision: u64,
    reset: ResetDecision,
    sample_order: Option<[i64; 6]>,
    detected_base: Option<u8>,
    retry_at: Option<Instant>,
    notice: String,
}

impl Application {
    pub fn new(
        config: PathBuf,
        root: PathBuf,
        model: ModelVariant,
        auto_connect: bool,
    ) -> Result<Self> {
        std::fs::create_dir_all(config.join("arms"))?;
        Ok(Self {
            config,
            calibration_root: root,
            model,
            auto_connect,
            session: None,
            revision: uuid::Uuid::new_v4().as_u64_pair().1 & ((1 << 52) - 1),
            reset: ResetDecision::Pending,
            sample_order: None,
            detected_base: None,
            retry_at: None,
            notice: String::new(),
        })
    }

    pub fn controllers(&self) -> Result<Vec<ControllerChoice>> {
        let mut choices: BTreeMap<_, Vec<_>> = BTreeMap::new();
        for item in discover(&self.config)? {
            let controller = match item {
                ControllerMatch::Registered { controller, .. }
                | ControllerMatch::Unregistered { controller } => controller,
                ControllerMatch::MissingSerial { .. }
                | ControllerMatch::InvalidSerial { .. }
                | ControllerMatch::AmbiguousSerial { .. } => continue,
            };
            choices
                .entry(controller.serial.clone())
                .or_default()
                .push(controller);
        }
        Ok(choices
            .into_values()
            .filter_map(|mut ports| {
                if ports.len() != 1 {
                    return None;
                }
                let controller = ports.pop()?;
                let registered = self
                    .config
                    .join("arms")
                    .join(controller.serial.as_str())
                    .join("setup.toml")
                    .is_file();
                Some(ControllerChoice {
                    controller,
                    registered,
                })
            })
            .collect())
    }

    fn controller(&self, serial: &str) -> Result<UsbController> {
        let serial = ControllerSerial::parse(serial)?;
        self.controllers()?
            .into_iter()
            .find_map(|entry| (entry.controller.serial == serial).then_some(entry.controller))
            .with_context(|| format!("USB controller {} is absent or ambiguous", serial.as_str()))
    }

    fn open_session(&self, serial: &str) -> Result<Session<SerialBus>> {
        let controller = self.controller(serial)?;
        let arm_directory = self.config.join("arms").join(controller.serial.as_str());
        let files = ArmFiles::new(
            arm_directory,
            &self.calibration_root,
            controller.serial,
            self.model,
        );
        Session::connect(SerialBus::open(&controller.port, &ARM_IDS)?, files)
    }

    fn connect(&mut self, serial: &str) -> Result<()> {
        ensure!(self.session.is_none(), "Disconnect the current arm first");
        self.session = Some(self.open_session(serial)?);
        self.reset = ResetDecision::Pending;
        self.sample_order = None;
        self.detected_base = None;
        self.retry_at = None;
        Ok(())
    }

    pub fn poll(&mut self) {
        if self.retry_at.is_some_and(|time| Instant::now() < time) {
            return;
        }
        if let Err(error) = self.poll_arm() {
            self.notice = format!("{error:#}");
            self.retry_at = Some(Instant::now() + Duration::from_secs(1));
        }
    }

    fn poll_arm(&mut self) -> Result<()> {
        let Some(session) = self.session.as_ref() else {
            if self.auto_connect {
                let controllers = self.controllers()?;
                if let [choice] = controllers.as_slice() {
                    self.connect(choice.controller.serial.as_str())?;
                }
            }
            self.notice.clear();
            return Ok(());
        };
        if matches!(session.stage(), Stage::Saved | Stage::Error) {
            return Ok(());
        }
        if !session.has_bus() {
            let controller = self
                .controller(&session.setup.serial)
                .context("Calibration paused. Waiting for the original controller")?;
            self.session
                .as_mut()
                .context("No arm")?
                .reconnect(SerialBus::open(&controller.port, &ARM_IDS)?)?;
            self.revision += 1;
        }
        let session = self.session.as_mut().context("No arm")?;
        if let Err(error) = session.sample() {
            if error.downcast_ref::<CommunicationFailure>().is_some() {
                session.pause();
                self.sample_order = None;
                self.detected_base = None;
                self.revision += 1;
                return Err(error).context("Calibration paused. Reconnecting to the same board");
            }
            return Err(error);
        }
        self.notice.clear();
        self.retry_at = None;
        Ok(())
    }

    fn reset_selected(&mut self, serial: &str, motor_id: Option<u8>) -> Result<Vec<PathBuf>> {
        let controller = self.controller(serial)?;
        let directory = self
            .config
            .join("arms")
            .join(controller.serial.as_str())
            .join("backups")
            .join("servo-restore");
        let mut ids = self
            .session
            .as_ref()
            .map_or(ARM_IDS, |session| session.setup.motor_ids.ordered());
        let count = if let Some(motor) = motor_id {
            ensure!(ARM_IDS.contains(&motor), "Motor ID must be 1–6");
            ids[0] = motor;
            1
        } else {
            ids.len()
        };
        self.auto_connect = false;
        let mut session = self.session.take();
        if let Some(session) = &mut session {
            session.pause();
        }
        let result = restore_servos(&controller.port, &ids[..count], &directory);
        if session.is_some() {
            match self.open_session(serial) {
                Ok(mut reopened) => {
                    reopened.restart()?;
                    self.session = Some(reopened);
                }
                Err(error) if result.is_ok() => {
                    return Err(error).context("Reset finished, but the arm could not be reopened");
                }
                Err(_) => {}
            }
        }
        if result.is_ok() {
            self.reset = ResetDecision::Decided;
        }
        self.sample_order = None;
        self.detected_base = None;
        self.revision += 1;
        result
    }

    pub fn standalone_reset(
        &mut self,
        serial: &str,
        motor: Option<u8>,
        confirmed: bool,
    ) -> Result<Vec<PathBuf>> {
        ensure!(
            confirmed,
            "Support and disable the arm, then pass --confirm"
        );
        ensure!(
            self.session
                .as_ref()
                .is_none_or(|session| session.setup.serial == serial && motor.is_none()),
            "Disconnect the current calibration session first"
        );
        self.reset_selected(serial, motor)
    }

    pub fn act(&mut self, action: Action) -> Result<View> {
        match action {
            Action::Connect { serial } => self.connect(&serial)?,
            Action::Begin { serial, skip_reset } => {
                if self
                    .session
                    .as_ref()
                    .is_none_or(|session| session.setup.serial != serial)
                {
                    self.connect(&serial)?;
                }
                if skip_reset
                    && self
                        .session
                        .as_ref()
                        .is_some_and(|session| session.stage() == Stage::Reference)
                    && matches!(self.reset, ResetDecision::Pending)
                {
                    self.act_on_session(&Action::SkipServoReset)?;
                }
            }
            Action::Disconnect => {
                self.session = None;
                self.auto_connect = false;
                self.retry_at = None;
            }
            Action::Reopen => {
                let session = self.session.as_ref().context("Connect an arm first")?;
                ensure!(
                    matches!(session.stage(), Stage::Saved),
                    "Reopen is available after saving"
                );
                self.session = Some(self.open_session(&session.setup.serial)?);
            }
            Action::ResetServos => {
                let session = self.session.as_ref().context("Connect an arm first")?;
                ensure!(
                    session.stage() == Stage::Reference
                        && matches!(self.reset, ResetDecision::Pending),
                    "Choose reset before beginning a full sweep"
                );
                let serial = session.setup.serial.clone();
                self.reset_selected(&serial, None)?;
            }
            action => self.act_on_session(&action)?,
        }
        self.revision += 1;
        self.notice.clear();
        self.view()
    }

    fn act_on_session(&mut self, action: &Action) -> Result<()> {
        let session = self.session.as_mut().context("Connect an arm first")?;
        ensure!(
            session.has_bus(),
            "Reconnect the arm before changing calibration"
        );
        match action {
            Action::Restart => {
                session.restart()?;
                self.reset = ResetDecision::Pending;
                self.sample_order = None;
                self.detected_base = None;
                Ok(())
            }
            Action::SkipServoReset => {
                ensure!(
                    session.stage() == Stage::Reference
                        && matches!(self.reset, ResetDecision::Pending),
                    "Choose reset before beginning a full sweep"
                );
                ensure!(
                    !session.files.reset_incomplete()?,
                    "An incomplete reset must be retried first"
                );
                self.reset = ResetDecision::Decided;
                Ok(())
            }
            Action::StartSweep { joint_index } => {
                ensure!(
                    joint_index.is_some() || matches!(self.reset, ResetDecision::Decided),
                    "Choose whether to reset servos first"
                );
                session.start_sweep(*joint_index)
            }
            Action::FinishSweep => session.finish_sweep(),
            Action::MapMotorIds { motor_ids } => {
                session.remap(*motor_ids)?;
                self.sample_order = None;
                self.detected_base = None;
                Ok(())
            }
            Action::CheckMotorOrder => {
                Self::check_motor_order(session, &mut self.sample_order, &mut self.detected_base)
            }
            Action::ApplyDetectedMotorOrder => {
                let motor = self
                    .detected_base
                    .context("Check which motor moves the base first")?;
                let order = if motor == 1 {
                    ARM_IDS
                } else {
                    [6, 5, 4, 3, 2, 1]
                };
                session.remap(MotorIds::from_joint_order(order)?)?;
                self.detected_base = None;
                Ok(())
            }
            Action::ReverseJoint { index } => session.edit(*index, None),
            Action::AdjustJoint {
                index,
                delta_radians,
            } => session.edit(*index, Some(*delta_radians)),
            Action::Save {
                model_verified,
                review_revision,
            } => {
                ensure!(
                    *model_verified,
                    "Review and confirm live calibration before saving"
                );
                ensure!(
                    *review_revision == self.revision,
                    "Calibration changed. Review the current draft before saving."
                );
                session.save()
            }
            Action::Connect { .. }
            | Action::Begin { .. }
            | Action::Disconnect
            | Action::ResetServos
            | Action::Reopen => {
                bail!("Invalid session action")
            }
        }
    }

    fn check_motor_order(
        session: &mut Session<SerialBus>,
        sample_order: &mut Option<[i64; 6]>,
        detected_base: &mut Option<u8>,
    ) -> Result<()> {
        ensure!(
            session.is_full_sweep(),
            "Check motor order during a full sweep"
        );
        session.sample()?;
        let positions = session.positions();
        *detected_base = None;
        if let Some(before) = sample_order.take() {
            let ids = session.setup.motor_ids.ordered();
            let moved: Vec<_> = (0..6)
                .filter(|&index| {
                    let delta = (positions[index] - before[index]).abs();
                    delta.min(4096 - delta) >= 16
                })
                .map(|index| ids[index])
                .collect();
            match moved.as_slice() {
                [motor @ (1 | 6)] => *detected_base = Some(*motor),
                _ => bail!("Motor order is unclear. Try again and move only the base."),
            }
        } else {
            *sample_order = Some(positions);
        }
        Ok(())
    }

    pub fn view(&self) -> Result<View> {
        let controllers = if self.session.is_some() {
            Vec::new()
        } else {
            self.controllers()?
        };
        let mut view = self
            .session
            .as_ref()
            .map_or_else(SessionView::default, Session::view);
        if !self.notice.is_empty() {
            view.message.clone_from(&self.notice);
            if self.session.is_some() {
                view.stage = Stage::Error;
            }
        } else if self.session.is_none() {
            view.message = if controllers.is_empty() {
                "No USB controller detected."
            } else {
                "Select a USB controller."
            }
            .into();
        }
        let serial = self
            .session
            .as_ref()
            .map(|session| session.setup.serial.clone())
            .unwrap_or_default();
        Ok(View {
            review_revision: (view.stage == Stage::Review).then_some(self.revision),
            offer_servo_reset: view.stage == Stage::Reference
                && matches!(self.reset, ResetDecision::Pending),
            servo_reset_incomplete: self
                .session
                .as_ref()
                .map(|session| session.files.reset_incomplete())
                .transpose()?
                .unwrap_or_default(),
            checking_motor_order: self.sample_order.is_some(),
            detected_base_motor: self.detected_base,
            model_variant: self
                .session
                .as_ref()
                .map_or(self.model, |session| session.setup.model_variant),
            motor_ids: self
                .session
                .as_ref()
                .map_or_else(MotorIds::default, |session| session.setup.motor_ids),
            home_joints_radians: HOME_JOINTS,
            home_gripper_radians: HOME_GRIPPER,
            session: view,
            serial,
            controllers,
        })
    }

    pub fn program(&mut self, request: &ProgramMotor) -> Result<u8> {
        ensure!(
            request.isolated_motor,
            "Confirm the adapter is connected to one isolated servo"
        );
        ensure!(
            self.session.is_none(),
            "Disconnect calibration before programming a motor"
        );
        ensure!(
            programming_ports()?
                .iter()
                .any(|port| port.port == request.port),
            "The selected USB adapter is unavailable"
        );
        self.auto_connect = false;
        program_motor_id(&request.port, request.motor_id)
    }
}

pub fn programming_ports() -> Result<Vec<ProgrammingPort>> {
    let mut ports: Vec<_> = serialport::available_ports()?
        .into_iter()
        .filter_map(|port| {
            let SerialPortType::UsbPort(usb) = port.port_type else {
                return None;
            };
            let description = usb
                .product
                .or(usb.manufacturer)
                .unwrap_or_else(|| "USB serial adapter".into());
            Some(ProgrammingPort {
                port: port.port_name,
                description,
            })
        })
        .collect();
    ports.sort_by(|first, second| first.port.cmp(&second.port));
    Ok(ports)
}

pub fn follower_motors() -> impl Serialize {
    ARM_IDS
        .into_iter()
        .zip(MOTOR_NAMES)
        .map(|(id, joint)| serde_json::json!({ "id": id, "joint": joint }))
        .collect::<Vec<_>>()
}
