//! Per-arm SO-101 wiring, model alignment and servo settings.

use std::collections::BTreeMap;

use anyhow::ensure;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(try_from = "i64", into = "i32")]
pub enum Sign {
    Negative,
    Positive,
}

impl From<Sign> for i32 {
    fn from(sign: Sign) -> Self {
        sign.as_integer()
    }
}

impl TryFrom<i64> for Sign {
    type Error = String;

    fn try_from(value: i64) -> Result<Self, Self::Error> {
        match value {
            -1 => Ok(Self::Negative),
            1 => Ok(Self::Positive),
            other => Err(format!("A sign is 1 or -1, not {other}")),
        }
    }
}

impl Sign {
    #[must_use]
    pub fn as_integer(self) -> i32 {
        match self {
            Self::Negative => -1,
            Self::Positive => 1,
        }
    }
}

fn positive_sign() -> Sign {
    Sign::Positive
}

/// SO-101 bus, calibration and servo facts, checked in preflight.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeetechSettings {
    /// Serial port per arm; use stable udev names, not /dev/ttyACM*.
    pub ports: BTreeMap<String, String>,
    /// `LeRobot` calibration id per arm, the file name under its calibration folder.
    pub calibration_ids: BTreeMap<String, String>,
    pub motor_ids: Vec<u8>,
    /// Motors on the bus that stay torque-free.
    pub passive_motor_ids: Vec<u8>,
    #[serde(default)]
    pub gripper_motor_id: Option<u8>,
    #[serde(default = "positive_sign")]
    pub gripper_sign: Sign,
    #[serde(default)]
    pub gripper_offset_radians: f64,
    pub expected_position_gains: [i64; 3],
    /// Keyed by motor id, as TOML keys are strings.
    #[serde(default)]
    pub expected_position_gains_by_motor: BTreeMap<String, [i64; 3]>,
    /// `Return_Delay_Time` every driven servo must already hold; unset skips the check.
    #[serde(default)]
    pub expected_return_delay_time: Option<u8>,
    pub max_temperature_celsius: f64,
    pub min_voltage_volts: f64,
    pub max_voltage_volts: f64,
    /// Joint angle = sign * (steps - homing steps) * 2 pi / resolution + offset.
    pub joint_signs: Vec<Sign>,
    pub joint_offsets_radians: Vec<f64>,
}

impl FeetechSettings {
    pub(crate) fn validate_joint_alignment(&self, joint_count: usize) -> anyhow::Result<()> {
        ensure!(
            !self.motor_ids.is_empty()
                && self.motor_ids.len() == joint_count
                && self.joint_signs.len() == joint_count
                && self.joint_offsets_radians.len() == joint_count,
            "Feetech motor ids, signs and offsets are per joint"
        );
        ensure!(
            self.joint_offsets_radians
                .iter()
                .all(|offset| offset.is_finite()),
            "Feetech joint offsets must be finite"
        );
        Ok(())
    }

    /// The arm's joint motors, then the gripper's if there is one.
    #[must_use]
    pub fn driven_motor_ids(&self) -> Vec<u8> {
        self.motor_ids
            .iter()
            .copied()
            .chain(self.gripper_motor_id)
            .collect()
    }

    /// The gains a motor is expected to hold: its override, or the shared ones.
    #[must_use]
    pub fn expected_gains(&self, motor_id: u8) -> [i64; 3] {
        self.expected_position_gains_by_motor
            .get(&motor_id.to_string())
            .copied()
            .unwrap_or(self.expected_position_gains)
    }
}
impl FeetechSettings {
    /// Validate the SO-101 wiring and model mapping against the enclosing profile.
    ///
    /// # Errors
    /// Ports, motors, model coordinates, gripper or gain overrides are inconsistent.
    pub fn validate_profile_mapping(
        &self,
        arms: &[String],
        joint_count: usize,
        has_gripper: bool,
    ) -> anyhow::Result<()> {
        let arms: Vec<&String> = arms.iter().collect();
        let mut sorted_arms = arms.clone();
        sorted_arms.sort();
        sorted_arms.dedup();
        ensure!(
            sorted_arms == self.ports.keys().collect::<Vec<_>>()
                && sorted_arms == self.calibration_ids.keys().collect::<Vec<_>>(),
            "Every arm needs a port and a calibration id"
        );
        self.validate_joint_alignment(joint_count)?;
        ensure!(
            self.gripper_offset_radians.is_finite(),
            "Feetech gripper offset must be finite"
        );
        ensure!(
            !self
                .motor_ids
                .iter()
                .any(|motor| self.passive_motor_ids.contains(motor)),
            "A motor cannot be both driven and passive"
        );
        ensure!(
            self.gripper_motor_id.is_some() == has_gripper,
            "A Feetech gripper needs both a motor and settings"
        );
        if let Some(gripper) = self.gripper_motor_id {
            ensure!(
                !self.motor_ids.contains(&gripper) && !self.passive_motor_ids.contains(&gripper),
                "The gripper motor cannot also be a joint or passive"
            );
        }
        let driven = self.driven_motor_ids();
        for motor in self.expected_position_gains_by_motor.keys() {
            ensure!(
                motor.parse::<u8>().is_ok_and(|id| driven.contains(&id)),
                "Gain overrides must name a driven motor, not {motor}"
            );
        }
        ensure!(
            [
                self.max_temperature_celsius,
                self.min_voltage_volts,
                self.max_voltage_volts
            ]
            .iter()
            .all(|value| value.is_finite() && *value > 0.0),
            "Feetech temperature and voltage limits must be positive and finite"
        );
        Ok(())
    }
}
