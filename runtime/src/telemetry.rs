//! Hardware observations and communication/reconnection outcomes.

use serde::Serialize;

/// The servo bus could not exchange a command or reading with its hardware.
#[derive(Debug)]
pub struct CommunicationFailure(pub String);

impl std::fmt::Display for CommunicationFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for CommunicationFailure {}

/// An independent follow-up to a severe environmental reading.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct EnvironmentConfirmation {
    pub motor_id: u8,
    pub temperature_celsius: f64,
    pub voltage_volts: f64,
    pub status: i64,
    pub packet_status: u8,
}

#[derive(Clone, Default)]
pub struct Feedback {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub efforts: Vec<f64>,
    /// The last temperature reading of each driven motor, the gripper's last;
    /// empty for a plant without thermometers.
    pub temperatures_celsius: Vec<f64>,
    /// Supply voltage for each driven motor in the same order as temperatures.
    pub voltages_volts: Vec<f64>,
    /// Follow-up readings; initial values remain in the regular arrays.
    pub environment_confirmations: Vec<EnvironmentConfirmation>,
    /// The jaw's angle in the URDF's gripper joint, when the arm has a gripper.
    pub gripper_angle: Option<f64>,
    /// A fault the hardware reports (an alarm, overheating): the loop trips on it.
    pub fault: Option<String>,
}

/// A fresh read after a communication interruption, before issuing a new goal.
pub struct RecoveryObservation {
    pub feedback: Feedback,
    /// Motor IDs and their observed torque-enable bits.
    pub motor_torque: Vec<(u8, bool)>,
    pub health_issue: Option<String>,
}

/// How getting a lost plant back stands.
pub enum Reconnection {
    /// No attempt finished since the last poll.
    Waiting,
    /// An attempt failed; another follows.
    Failed(anyhow::Error),
    /// The plant answers again, set up as at startup, with every motor heard.
    Restored,
}
