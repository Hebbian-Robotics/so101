//! Per-arm hardware configuration, independent of station policy.

use crate::calibration::{LerobotMotorCalibration, load_calibration, validate_encoder_calibration};
use crate::settings::{FeetechSettings, Sign};
use anyhow::Context;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// One motor's entry from the arm's `LeRobot` calibration file.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct MotorCalibration {
    pub homing_offset: i64,
    pub range_min: i64,
    pub range_max: i64,
}

/// Position-loop gains the servo must already hold: P, I, D.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Gains {
    pub p: i64,
    pub i: i64,
    pub d: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct FeetechMotor {
    pub id: u8,
    /// Joint angle = sign * (steps - 2047) * 2 pi / resolution + offset.
    pub sign: i32,
    pub offset_radians: f64,
    pub calibration: MotorCalibration,
    pub expected_gains: Gains,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct FeetechGripper {
    pub motor: FeetechMotor,
    /// Share of the servo's torque the jaw may use, held while gripping.
    pub force_fraction: f64,
}

/// The SO-101's Feetech bus: where it is and what its servos must look like
/// before torque goes on. The station reads the profile and the `LeRobot`
/// calibration file once and writes the result here.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct FeetechConfig {
    pub port: String,
    /// The arm's joints in order.
    pub joints: Vec<FeetechMotor>,
    pub gripper: Option<FeetechGripper>,
    /// Motors on the bus that stay torque-free.
    pub passive_motor_ids: Vec<u8>,
    /// `Return_Delay_Time` every driven servo must already hold; unset skips the
    /// check. Checked in preflight, never written. Each unit adds 2 us per motor
    /// to every bus read; `LeRobot` sets 0, the factory value is 250.
    #[serde(default)]
    pub expected_return_delay_time: Option<i64>,
    pub max_temperature_celsius: f64,
    pub min_voltage_volts: f64,
    pub max_voltage_volts: f64,
}

fn feetech_motor(
    settings: &FeetechSettings,
    calibration: &std::collections::BTreeMap<u8, LerobotMotorCalibration>,
    motor_id: u8,
    sign: Sign,
    offset_radians: f64,
) -> FeetechMotor {
    let entry = &calibration[&motor_id];
    let [p, i, d] = settings.expected_gains(motor_id);
    FeetechMotor {
        id: motor_id,
        sign: sign.as_integer(),
        offset_radians,
        calibration: MotorCalibration {
            homing_offset: entry.homing_offset,
            range_min: entry.range_min,
            range_max: entry.range_max,
        },
        expected_gains: Gains { p, i, d },
    }
}

/// Build one arm's runtime config from per-arm wiring and its calibration file.
///
/// # Errors
/// Wiring, alignment or calibration is invalid, or the arm has no configured port.
pub fn load_arm_config(
    gripper_force_fraction: Option<f64>,
    settings: &FeetechSettings,
    arm: &str,
    calibration_root: &Path,
) -> anyhow::Result<FeetechConfig> {
    settings.validate_profile_mapping(
        &settings.ports.keys().cloned().collect::<Vec<_>>(),
        settings.motor_ids.len(),
        gripper_force_fraction.is_some(),
    )?;
    let calibration = load_calibration(settings, arm, calibration_root)?;
    let joints = settings
        .motor_ids
        .iter()
        .zip(&settings.joint_signs)
        .zip(&settings.joint_offsets_radians)
        .map(|((motor_id, sign), offset)| {
            feetech_motor(settings, &calibration, *motor_id, *sign, *offset)
        })
        .collect();
    let gripper = match (settings.gripper_motor_id, gripper_force_fraction) {
        (Some(motor_id), Some(force_fraction)) => Some(FeetechGripper {
            motor: feetech_motor(
                settings,
                &calibration,
                motor_id,
                settings.gripper_sign,
                settings.gripper_offset_radians,
            ),
            force_fraction,
        }),
        _ => None,
    };
    let config = FeetechConfig {
        port: settings
            .ports
            .get(arm)
            .with_context(|| format!("No port for arm {arm}"))?
            .clone(),
        joints,
        gripper,
        passive_motor_ids: settings.passive_motor_ids.clone(),
        expected_return_delay_time: settings.expected_return_delay_time.map(i64::from),
        max_temperature_celsius: settings.max_temperature_celsius,
        min_voltage_volts: settings.min_voltage_volts,
        max_voltage_volts: settings.max_voltage_volts,
    };
    config.validate()?;
    Ok(config)
}

impl FeetechConfig {
    /// Validate the motor signs, calibrated ranges and voltage interval.
    ///
    /// # Errors
    /// A sign, range or supply interval cannot describe a usable motor mapping.
    pub fn validate(&self) -> anyhow::Result<()> {
        anyhow::ensure!(!self.port.trim().is_empty(), "The Feetech port is empty");
        anyhow::ensure!(!self.joints.is_empty(), "The arm has no joint motors");
        anyhow::ensure!(
            [
                self.min_voltage_volts,
                self.max_voltage_volts,
                self.max_temperature_celsius
            ]
            .iter()
            .all(|value| value.is_finite() && *value > 0.0),
            "Feetech temperature and voltage limits must be positive and finite"
        );
        anyhow::ensure!(
            self.min_voltage_volts < self.max_voltage_volts,
            "The Feetech voltage range is empty"
        );
        anyhow::ensure!(
            self.expected_return_delay_time
                .is_none_or(|delay| (0..=255).contains(&delay)),
            "The return delay must fit the servo register"
        );
        if let Some(gripper) = &self.gripper {
            anyhow::ensure!(
                gripper.force_fraction.is_finite()
                    && gripper.force_fraction > 0.0
                    && gripper.force_fraction <= 1.0,
                "Gripper force fraction must be finite and in (0, 1]"
            );
        }
        let mut motor_ids = std::collections::BTreeSet::new();
        for motor_id in self
            .joints
            .iter()
            .map(|motor| motor.id)
            .chain(self.gripper.iter().map(|gripper| gripper.motor.id))
            .chain(self.passive_motor_ids.iter().copied())
        {
            anyhow::ensure!(
                motor_id < 254 && motor_ids.insert(motor_id),
                "Motor ID {motor_id} is reserved or repeated"
            );
        }
        for motor in self
            .joints
            .iter()
            .chain(self.gripper.iter().map(|gripper| &gripper.motor))
        {
            let calibration = &motor.calibration;
            validate_encoder_calibration(
                motor.id,
                calibration.homing_offset,
                calibration.range_min,
                calibration.range_max,
            )?;
            anyhow::ensure!(
                [
                    motor.expected_gains.p,
                    motor.expected_gains.i,
                    motor.expected_gains.d
                ]
                .into_iter()
                .all(|gain| u8::try_from(gain).is_ok()),
                "Motor {} position gains must fit the servo registers",
                motor.id
            );
            anyhow::ensure!(
                motor.sign == 1 || motor.sign == -1,
                "Motor {} sign must be 1 or -1",
                motor.id
            );
            anyhow::ensure!(
                motor.offset_radians.is_finite(),
                "Motor {} offset must be finite",
                motor.id
            );
        }
        Ok(())
    }
}

pub(crate) struct ValidatedFeetechConfig(FeetechConfig);

impl TryFrom<FeetechConfig> for ValidatedFeetechConfig {
    type Error = anyhow::Error;

    fn try_from(config: FeetechConfig) -> anyhow::Result<Self> {
        config.validate()?;
        Ok(Self(config))
    }
}

impl std::ops::Deref for ValidatedFeetechConfig {
    type Target = FeetechConfig;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
