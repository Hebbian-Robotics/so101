mod app;
mod service;
mod update;

use std::io::{self, IsTerminal, Write};
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Parser, Subcommand, ValueEnum};
use hebbian_so101::calibration::lerobot_calibration_root;
use hebbian_so101::commissioning::MOTOR_NAMES;
use hebbian_so101::setup::{ModelVariant, MotorIds};
use serde::Deserialize;
use serde_json::{Value, json};

use crate::app::{Action, Application, ProgramMotor, programming_ports};
use crate::service::{Client, ResetRequest, ServiceDirectory, resolve_directory};

#[derive(Parser)]
#[command(version, about = "SO-101 arm setup, calibration and servo maintenance")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Start a local setup service and open the bundled browser UI.
    Setup(SetupOptions),
    /// List USB controllers and adapters.
    List {
        #[arg(long)]
        adapters: bool,
        #[arg(long)]
        setup_directory: Option<PathBuf>,
    },
    /// Run the guided calibration steps from the terminal.
    Calibrate {
        #[command(subcommand)]
        command: CalibrationCommand,
    },
    /// Update a directly installed so101 executable from GitHub Releases.
    Update {
        /// Report whether a compatible release is available without installing it.
        #[arg(long)]
        check: bool,
    },
    /// Program servo IDs and reset servo parameters.
    Servo {
        #[command(subcommand)]
        command: ServoCommand,
    },
}

#[derive(Subcommand)]
enum ServoCommand {
    /// Program one isolated servo and verify its physical ID.
    Id {
        /// Serial port of the USB servo adapter.
        #[arg(long)]
        port: String,
        /// New physical ID to assign to the servo.
        #[arg(long)]
        motor_id: u8,
        /// Confirm that only the loose servo is connected.
        #[arg(long)]
        isolated: bool,
    },
    /// Reset factory parameters on one servo or all six servos.
    Reset(ResetOptions),
}

#[derive(Clone, Copy, ValueEnum)]
enum NewModel {
    Stock,
    SoftGripperCamera,
}

#[derive(Args)]
struct SetupOptions {
    #[arg(long)]
    no_browser: bool,
    #[arg(long)]
    no_auto_connect: bool,
    #[arg(long, default_value_t = 0)]
    port: u16,
    #[arg(long)]
    setup_directory: Option<PathBuf>,
    #[arg(long, value_enum, default_value = "soft-gripper-camera")]
    model_variant: NewModel,
}

#[derive(Args)]
struct ResetOptions {
    /// USB serial number of the assembled arm's controller.
    #[arg(long)]
    serial: String,
    /// Current physical ID of the servo to reset.
    #[arg(long, conflicts_with = "all", required_unless_present = "all")]
    motor_id: Option<u8>,
    /// Reset all six servos on the selected arm.
    #[arg(long)]
    all: bool,
    #[arg(long)]
    confirm: bool,
    #[arg(long)]
    setup_directory: Option<PathBuf>,
}

#[derive(Subcommand)]
enum CalibrationCommand {
    /// Connect to an identified controller.
    Begin {
        #[arg(long)]
        serial: String,
        #[arg(long)]
        skip_reset: bool,
    },
    /// Show live status and joint readings.
    Status {
        #[arg(long)]
        watch: bool,
        #[arg(long)]
        json: bool,
    },
    /// Skip the optional factory reset before a full sweep.
    SkipReset,
    /// Start or finish a hand-moved sweep.
    Sweep {
        #[command(subcommand)]
        operation: SweepCommand,
    },
    /// Change the mapping of the six physical IDs in joint order during a full sweep.
    Map {
        #[arg(value_delimiter = ',', num_args = 1..=6, required = true, value_name = "MOTOR_ID")]
        motor_ids: Vec<u8>,
    },
    /// Sample the base motor before and after moving the base by hand.
    CheckOrder,
    /// Apply the motor order detected by check-order.
    ApplyOrder,
    /// Review the draft, including its revision, encoder ranges and alignments.
    Review {
        #[arg(long)]
        watch: bool,
        #[arg(long)]
        json: bool,
    },
    /// Reverse the direction of one joint while retaining its reference.
    Reverse { joint: Joint },
    /// Adjust one joint's alignment by up to 0.1 radians.
    Adjust {
        joint: Joint,
        #[arg(allow_hyphen_values = true)]
        delta_radians: f64,
    },
    /// Save a verified calibration draft.
    Save {
        /// Confirm the alignment and direction on the physical arm.
        #[arg(long)]
        confirm: bool,
        /// Revision shown by the review command. Required for non-interactive saves.
        #[arg(long, requires = "confirm")]
        revision: Option<u64>,
    },
    /// Discard the draft, or start a fresh full calibration from a saved setup.
    Restart,
    /// Reopen a saved calibration for inspection or single-joint recalibration.
    Reopen,
    /// Release the serial connection and discard the in-memory draft.
    Disconnect,
}

#[derive(Subcommand)]
enum SweepCommand {
    /// Capture the reference pose and start recording, optionally for one joint.
    Start {
        #[arg(long, value_enum)]
        joint: Option<Joint>,
    },
    /// End recording and estimate the calibration.
    Finish,
}

#[derive(Clone, Copy, ValueEnum)]
enum Joint {
    ShoulderPan,
    ShoulderLift,
    ElbowFlex,
    WristFlex,
    WristRoll,
    Gripper,
}

impl Joint {
    fn index(self) -> usize {
        match self {
            Self::ShoulderPan => 0,
            Self::ShoulderLift => 1,
            Self::ElbowFlex => 2,
            Self::WristFlex => 3,
            Self::WristRoll => 4,
            Self::Gripper => 5,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let directory = ServiceDirectory::locate()?;
    match cli.command {
        Command::Setup(options) => run_setup(&directory, &options),
        Command::Update { check } => update::run(&directory, check),
        Command::List {
            adapters,
            setup_directory,
        } => {
            let payload = if adapters {
                serde_json::to_value(programming_ports()?)?
            } else if let Some(client) = directory.running_client()? {
                client.get("/api/controllers")?
            } else {
                serde_json::to_value(
                    application(setup_directory.as_deref(), ModelVariant::Stock, false)?
                        .controllers()?,
                )?
            };
            print_json(&payload)
        }
        Command::Calibrate { command } => calibration(&Client::connect(&directory)?, command),
        Command::Servo { command } => run_servo_command(&directory, command),
    }
}

fn run_servo_command(directory: &ServiceDirectory, command: ServoCommand) -> Result<()> {
    match command {
        ServoCommand::Id {
            port,
            motor_id,
            isolated,
        } => {
            ensure!(
                isolated,
                "Connect only the loose motor with torque off and pass --isolated"
            );
            let request = ProgramMotor {
                port,
                motor_id,
                isolated_motor: isolated,
            };
            let output = if let Some(client) = directory.running_client()? {
                client.post("/api/motor-ids", &request)?
            } else {
                let _lock = directory.lock()?;
                json!({ "motor_id": application(None, ModelVariant::Stock, false)?.program(&request)? })
            };
            print_json(&output)
        }
        ServoCommand::Reset(options) => reset_servo(directory, options),
    }
}

fn application(
    path: Option<&std::path::Path>,
    model: ModelVariant,
    auto_connect: bool,
) -> Result<Application> {
    Application::new(
        resolve_directory(path)?,
        lerobot_calibration_root(),
        model,
        auto_connect,
    )
}

fn run_setup(directory: &ServiceDirectory, options: &SetupOptions) -> Result<()> {
    let _lock = directory.lock()?;
    let model = match options.model_variant {
        NewModel::Stock => ModelVariant::Stock,
        NewModel::SoftGripperCamera => ModelVariant::SoftGripperCamera,
    };
    let auto_connect = !(options.no_auto_connect || options.no_browser);
    let app = application(options.setup_directory.as_deref(), model, auto_connect)?;
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(service::serve(
            app,
            directory,
            options.port,
            options.no_browser,
        ))
}

fn reset_servo(directory: &ServiceDirectory, options: ResetOptions) -> Result<()> {
    ensure!(
        options.confirm,
        "Support and disable all six servos, then pass --confirm"
    );
    let request = ResetRequest {
        serial: options.serial,
        motor_id: options.motor_id,
        confirmed: options.confirm,
    };
    let output = if let Some(client) = directory.running_client()? {
        ensure!(
            options.setup_directory.is_none(),
            "Use the running setup service's directory or stop it first"
        );
        client.post("/api/reset", &request)?
    } else {
        let _lock = directory.lock()?;
        let mut app = application(
            options.setup_directory.as_deref(),
            ModelVariant::Stock,
            false,
        )?;
        json!({ "backups": app.standalone_reset(&request.serial, request.motor_id, request.confirmed)? })
    };
    print_json(&output)?;
    println!("Verify calibration before enabling the arm.");
    Ok(())
}

fn calibration(client: &Client, command: CalibrationCommand) -> Result<()> {
    let action = match command {
        CalibrationCommand::Begin { serial, skip_reset } => {
            return show_view(&client.act(&Action::Begin { serial, skip_reset })?);
        }
        CalibrationCommand::Status { watch, json } => {
            return show_status(client, watch, json, false);
        }
        CalibrationCommand::Review { watch, json } => {
            return show_status(client, watch, json, true);
        }
        CalibrationCommand::SkipReset => Action::SkipServoReset,
        CalibrationCommand::Sweep { operation } => match operation {
            SweepCommand::Start { joint } => Action::StartSweep {
                joint_index: joint.map(Joint::index),
            },
            SweepCommand::Finish => Action::FinishSweep,
        },
        CalibrationCommand::Map { motor_ids } => Action::MapMotorIds {
            motor_ids: MotorIds::from_joint_order(
                motor_ids
                    .try_into()
                    .map_err(|_| anyhow::anyhow!("Specify six motor IDs"))?,
            )?,
        },
        CalibrationCommand::CheckOrder => Action::CheckMotorOrder,
        CalibrationCommand::ApplyOrder => Action::ApplyDetectedMotorOrder,
        CalibrationCommand::Reverse { joint } => Action::ReverseJoint {
            index: joint.index(),
        },
        CalibrationCommand::Adjust {
            joint,
            delta_radians,
        } => Action::AdjustJoint {
            index: joint.index(),
            delta_radians,
        },
        CalibrationCommand::Save { confirm, revision } => {
            let review_revision = verify_save(client, confirm, revision)?;
            Action::Save {
                model_verified: true,
                review_revision,
            }
        }
        CalibrationCommand::Restart => Action::Restart,
        CalibrationCommand::Reopen => Action::Reopen,
        CalibrationCommand::Disconnect => Action::Disconnect,
    };
    show_view(&client.act(&action)?)
}

fn verify_save(client: &Client, confirm: bool, revision: Option<u64>) -> Result<u64> {
    if confirm {
        return revision.context("Pass the --revision shown by so101 calibrate review");
    }
    ensure!(
        io::stdin().is_terminal(),
        "Review the live arm and pass --confirm --revision <revision>"
    );
    let payload = client.get("/api/state")?;
    let revision = payload["review_revision"]
        .as_u64()
        .context("Review a completed calibration draft before saving")?;
    show_view(&payload)?;
    print!(
        "Have you checked the live angles, ranges and directions against the physical arm? [y/N] "
    );
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().read_line(&mut answer)?;
    ensure!(matches!(answer.trim(), "y" | "Y" | "yes"), "Save cancelled");
    Ok(revision)
}

fn show_status(client: &Client, watch: bool, as_json: bool, review_only: bool) -> Result<()> {
    loop {
        let view = client.get("/api/state")?;
        if review_only && view["review_revision"].is_null() {
            bail!("Calibration is not ready for review. Run so101 calibrate status.");
        }
        if as_json {
            println!("{}", serde_json::to_string(&view)?);
        } else {
            show_view(&view)?;
        }
        if !watch {
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(400));
    }
}

#[derive(Deserialize)]
struct TerminalView {
    stage: String,
    serial: String,
    message: String,
    review_revision: Option<u64>,
    offer_servo_reset: bool,
    checking_motor_order: bool,
    detected_base_motor: Option<u8>,
    joints: Vec<TerminalJoint>,
    motor_ids: MotorIds,
}

#[derive(Deserialize)]
struct TerminalJoint {
    name: String,
    position_radians: f64,
    range_min: i64,
    range_max: i64,
    offset_radians: f64,
    sign: i32,
    moving: bool,
}

fn show_view(value: &Value) -> Result<()> {
    let view: TerminalView = TerminalView::deserialize(value)?;
    println!("{} {}", view.serial, view.stage);
    if let Some(revision) = view.review_revision {
        println!("Review revision {revision}");
    }
    if view.offer_servo_reset {
        println!(
            "Choose so101 servo reset --serial {} --all --confirm or so101 calibrate skip-reset.",
            view.serial
        );
    }
    if view.checking_motor_order {
        println!("Move only the base, then run so101 calibrate check-order again.");
    }
    if let Some(motor) = view.detected_base_motor {
        println!(
            "Detected base motor {motor}. Run so101 calibrate apply-order to use that mapping."
        );
    }
    if !view.message.is_empty() {
        println!("{}", view.message);
    }
    for joint in view.joints {
        let id = MOTOR_NAMES
            .iter()
            .zip(view.motor_ids.ordered())
            .find_map(|(name, id)| (*name == joint.name).then_some(id))
            .context("Unknown SO-101 joint")?;
        let movement = if joint.moving { " moving" } else { "" };
        println!(
            "{:<14} ID {id}  {:>7.3} rad  steps {:>5}..{:<5}  dir {:+}  offset {:+.3}{movement}",
            joint.name,
            joint.position_radians,
            joint.range_min,
            joint.range_max,
            joint.sign,
            joint.offset_radians
        );
    }
    Ok(())
}

fn print_json(value: &Value) -> Result<()> {
    println!("{}", serde_json::to_string_pretty(value)?);
    Ok(())
}
