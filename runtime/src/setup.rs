//! Native SO-101 setup file types shared by control applications.

use crate::SO101_JOINT_NAMES;
use crate::calibration::{calibration_by_motor_id, parse_named_calibration};
use crate::settings::Sign;
use anyhow::{Context, ensure};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

pub(crate) const INCOMPLETE_CALIBRATION_FILE: &str = ".calibration-pending";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum ModelVariant {
    #[default]
    Stock,
    SoftGripperCamera,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorIds {
    shoulder_pan: u8,
    shoulder_lift: u8,
    elbow_flex: u8,
    wrist_flex: u8,
    wrist_roll: u8,
    gripper: u8,
}

impl Default for MotorIds {
    fn default() -> Self {
        Self {
            shoulder_pan: 1,
            shoulder_lift: 2,
            elbow_flex: 3,
            wrist_flex: 4,
            wrist_roll: 5,
            gripper: 6,
        }
    }
}

impl MotorIds {
    #[must_use]
    pub fn motor_ids_in_joint_order(self) -> ([u8; 5], u8) {
        (
            [
                self.shoulder_pan,
                self.shoulder_lift,
                self.elbow_flex,
                self.wrist_flex,
                self.wrist_roll,
            ],
            self.gripper,
        )
    }

    pub fn from_joint_order(ids: [u8; 6]) -> anyhow::Result<Self> {
        let [
            shoulder_pan,
            shoulder_lift,
            elbow_flex,
            wrist_flex,
            wrist_roll,
            gripper,
        ] = ids;
        let mapping = Self {
            shoulder_pan,
            shoulder_lift,
            elbow_flex,
            wrist_flex,
            wrist_roll,
            gripper,
        };
        mapping.validate()?;
        Ok(mapping)
    }

    #[must_use]
    pub fn ordered(self) -> [u8; 6] {
        let (joints, gripper) = self.motor_ids_in_joint_order();
        let [pan, lift, elbow, flex, roll] = joints;
        [pan, lift, elbow, flex, roll, gripper]
    }

    pub fn validate(self) -> anyhow::Result<()> {
        let mut ids = self.ordered();
        ids.sort_unstable();
        ensure!(
            ids == [1, 2, 3, 4, 5, 6],
            "Motor mapping must use IDs 1-6 exactly once"
        );
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ArmSetup {
    pub serial: String,
    #[serde(default)]
    pub model_variant: ModelVariant,
    #[serde(default)]
    pub motor_ids: MotorIds,
    #[serde(default)]
    pub alignment: Alignment,
    #[serde(default)]
    pub gripper: GripperEndpoints,
    #[serde(default)]
    pub servos: ServoSettings,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
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

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
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

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
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
        let pending = path.with_file_name(INCOMPLETE_CALIBRATION_FILE);
        ensure!(
            !pending.try_exists()?,
            "SO-101 calibration save did not finish. Inspect the backups named in {} and recalibrate.",
            pending.display()
        );
        let setup: Self = toml::from_str(&std::fs::read_to_string(path)?)
            .with_context(|| format!("Invalid SO-101 setup {}", path.display()))?;
        setup.validate(serial)?;
        Ok(setup)
    }

    pub(crate) fn validate(&self, serial: &str) -> anyhow::Result<()> {
        ensure!(
            self.serial == serial,
            "Setup serial does not match USB controller"
        );
        ensure!(
            self.alignment
                .joint_offsets_radians
                .iter()
                .chain([&self.alignment.gripper_offset_radians])
                .all(|value| value.is_finite()),
            "Setup offsets must be finite"
        );
        ensure!(
            self.gripper.closed_position_radians.is_finite()
                && self.gripper.open_position_radians.is_finite()
                && (self.gripper.closed_position_radians - self.gripper.open_position_radians)
                    .abs()
                    > 0.0,
            "Invalid gripper endpoints"
        );
        ensure!(
            self.servos
                .position_gains_by_motor
                .keys()
                .all(|id| id.parse::<u8>().is_ok_and(|id| (1..=6).contains(&id))),
            "Gain overrides must name an SO-101 motor (IDs 1-6)"
        );
        self.motor_ids.validate()?;
        Ok(())
    }
}

impl MotorIds {
    /// Check that the named calibration entries agree with the physical joint order.
    ///
    /// # Errors
    /// The file cannot be read or the named entries do not match this order.
    pub fn validate_calibration_mapping(self, path: &Path) -> anyhow::Result<()> {
        self.load_named_calibration(path).map(|_| ())
    }

    pub fn load_named_calibration(
        self,
        path: &Path,
    ) -> anyhow::Result<BTreeMap<String, crate::calibration::LerobotMotorCalibration>> {
        let entries = parse_named_calibration(path, &std::fs::read(path)?)?;
        let joint_names = SO101_JOINT_NAMES.into_iter().chain(["gripper"]);
        let (joint_ids, gripper_id) = self.motor_ids_in_joint_order();
        let motor_ids = joint_ids.into_iter().chain([gripper_id]);
        ensure!(
            entries.len() == 6
                && joint_names
                    .zip(motor_ids.clone())
                    .all(|(name, id)| entries.get(name).is_some_and(|entry| entry.id == id)),
            "Calibration {} does not match the setup's motor ID mapping; recalibrate the arm",
            path.display()
        );
        calibration_by_motor_id(entries.values().cloned(), motor_ids, path)?;
        Ok(entries)
    }
}
