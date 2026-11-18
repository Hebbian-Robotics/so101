//! Native SO-101 setup file types shared by control applications.

use crate::SO101_JOINT_NAMES;
use crate::calibration::LerobotMotorCalibration;
use crate::settings::Sign;
use anyhow::{Context, ensure};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

#[derive(Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ModelVariant {
    #[default]
    Stock,
    SoftGripperCamera,
}

#[derive(Clone, Copy, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MotorIdOrder {
    #[default]
    Ascending,
    Descending,
}

impl MotorIdOrder {
    #[must_use]
    pub fn motor_ids_in_joint_order(self) -> ([u8; 5], u8) {
        match self {
            Self::Ascending => ([1, 2, 3, 4, 5], 6),
            Self::Descending => ([6, 5, 4, 3, 2], 1),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArmSetup {
    pub serial: String,
    #[serde(default)]
    pub model_variant: ModelVariant,
    #[serde(default)]
    pub motor_id_order: MotorIdOrder,
    #[serde(default)]
    pub alignment: Alignment,
    #[serde(default)]
    pub gripper: GripperEndpoints,
    #[serde(default)]
    pub servos: ServoSettings,
}

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Alignment {
    pub joint_signs: [Sign; 5],
    pub joint_offsets_radians: [f64; 5],
    pub gripper_sign: Sign,
    pub gripper_offset_radians: f64,
}

impl Default for Alignment {
    fn default() -> Self {
        Self {
            joint_signs: [Sign::Positive; 5],
            joint_offsets_radians: [0.0; 5],
            gripper_sign: Sign::Positive,
            gripper_offset_radians: 0.0,
        }
    }
}

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GripperEndpoints {
    pub closed_position_radians: f64,
    pub open_position_radians: f64,
}

impl Default for GripperEndpoints {
    fn default() -> Self {
        Self {
            closed_position_radians: 0.0,
            open_position_radians: 1.5,
        }
    }
}

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ServoSettings {
    #[serde(rename = "expected_position_gains")]
    pub position_gains: [i64; 3],
    #[serde(default, rename = "expected_position_gains_by_motor")]
    pub position_gains_by_motor: BTreeMap<String, [i64; 3]>,
    #[serde(rename = "expected_return_delay_time")]
    pub return_delay_time: Option<u8>,
}

impl Default for ServoSettings {
    fn default() -> Self {
        Self {
            position_gains: [16, 0, 32],
            position_gains_by_motor: BTreeMap::new(),
            return_delay_time: Some(0),
        }
    }
}

impl ArmSetup {
    /// Read and validate a setup for the selected USB controller.
    pub fn load(path: &Path, serial: &str) -> anyhow::Result<Self> {
        let setup: Self = toml::from_str(&std::fs::read_to_string(path)?)
            .with_context(|| format!("Invalid SO-101 setup {}", path.display()))?;
        ensure!(
            setup.serial == serial,
            "Setup serial does not match USB controller"
        );
        ensure!(
            setup
                .alignment
                .joint_offsets_radians
                .iter()
                .chain([&setup.alignment.gripper_offset_radians])
                .all(|value| value.is_finite()),
            "Setup offsets must be finite"
        );
        ensure!(
            setup.gripper.closed_position_radians.is_finite()
                && setup.gripper.open_position_radians.is_finite()
                && (setup.gripper.closed_position_radians - setup.gripper.open_position_radians)
                    .abs()
                    > 0.0,
            "Invalid gripper endpoints"
        );
        ensure!(
            setup
                .servos
                .position_gains_by_motor
                .keys()
                .all(|id| id.parse::<u8>().is_ok_and(|id| (1..=6).contains(&id))),
            "Gain overrides must name an SO-101 motor (IDs 1-6)"
        );
        Ok(setup)
    }
}

impl MotorIdOrder {
    /// Check that the named calibration entries agree with the physical joint order.
    ///
    /// # Errors
    /// The file cannot be read or the named entries do not match this order.
    pub fn validate_calibration_mapping(self, path: &Path) -> anyhow::Result<()> {
        let entries: BTreeMap<String, LerobotMotorCalibration> =
            serde_json::from_slice(&std::fs::read(path)?)
                .with_context(|| format!("Invalid SO-101 calibration {}", path.display()))?;
        let joint_names = SO101_JOINT_NAMES.into_iter().chain(["gripper"]);
        // Older ascending setups could use calibration keys with only bus-level meaning.
        if matches!(self, Self::Ascending)
            && !joint_names.clone().any(|name| entries.contains_key(name))
        {
            return Ok(());
        }
        let (joint_ids, gripper_id) = self.motor_ids_in_joint_order();
        ensure!(
            entries.len() == 6
                && joint_names
                    .zip(joint_ids.into_iter().chain([gripper_id]))
                    .all(|(name, id)| entries.get(name).is_some_and(|entry| entry.id == id)),
            "Calibration {} does not match the setup's motor ID order; recalibrate the arm",
            path.display()
        );
        Ok(())
    }
}
