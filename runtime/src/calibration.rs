//! An SO-101's `LeRobot` calibration: where it lives, its entries, and the
//! encoder endpoints as joint radians.
//!
//! `LeRobot`'s homing leaves its captured reference near mid-turn. Mapping that
//! reference to the URDF requires per-arm signs and offsets; a step count maps as
//! `sign * (steps - 2047) * 2 pi / 4096 + offset`. Per arm, verify with torque off
//! that the calibration pose reads as the URDF's zero pose and that each joint
//! turns the right way; the profile's signs and offsets absorb any difference.

use std::collections::BTreeMap;
use std::path::{Component, Path, PathBuf};

use anyhow::{Context, ensure};
use serde::{Deserialize, Serialize};

use super::settings::FeetechSettings;

/// The step count of the calibration pose.
pub(crate) const HOMING_STEPS: f64 = 2047.0;
/// Encoder steps per turn of the Feetech STS3215.
pub const STS3215_RESOLUTION: i64 = 4096;

/// One motor's entry in a `LeRobot` calibration file.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LerobotMotorCalibration {
    pub id: u8,
    pub drive_mode: i64,
    pub homing_offset: i64,
    pub range_min: i64,
    pub range_max: i64,
}

fn environment_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name)
        .filter(|value| !value.is_empty())
        .map(|value| expand_home(Path::new(&value)))
}

fn expand_home(path: &Path) -> PathBuf {
    match (path.strip_prefix("~"), std::env::var_os("HOME")) {
        (Ok(rest), Some(home)) => Path::new(&home).join(rest),
        _ => path.to_owned(),
    }
}

/// `LeRobot`'s calibration directory: the same chain as
/// `lerobot.utils.constants.HF_LEROBOT_CALIBRATION` over `huggingface_hub`'s `HF_HOME`.
#[must_use]
pub fn lerobot_calibration_root() -> PathBuf {
    if let Some(root) = environment_path("HF_LEROBOT_CALIBRATION") {
        return root;
    }
    if let Some(lerobot_home) = environment_path("HF_LEROBOT_HOME") {
        return lerobot_home.join("calibration");
    }
    let hugging_face_home = environment_path("HF_HOME").unwrap_or_else(|| {
        environment_path("XDG_CACHE_HOME")
            .unwrap_or_else(|| expand_home(Path::new("~/.cache")))
            .join("huggingface")
    });
    hugging_face_home.join("lerobot").join("calibration")
}

/// Where `LeRobot` keeps an SO follower's calibration under its calibration root.
#[must_use]
pub fn so_follower_calibration_path(calibration_root: &Path, calibration_id: &str) -> PathBuf {
    calibration_root
        .join("robots")
        .join("so_follower")
        .join(format!("{calibration_id}.json"))
}

/// The arm's calibration entries by motor id; every driven motor must have one.
///
/// # Errors
/// The file is missing, malformed, lacks a driven motor, or has an empty range.
pub fn load_calibration(
    settings: &FeetechSettings,
    arm: &str,
    calibration_root: &Path,
) -> anyhow::Result<BTreeMap<u8, LerobotMotorCalibration>> {
    load_calibration_path(
        settings,
        &calibration_path_for_arm(settings, arm, calibration_root)?,
    )
}

pub(crate) fn calibration_path_for_arm(
    settings: &FeetechSettings,
    arm: &str,
    calibration_root: &Path,
) -> anyhow::Result<PathBuf> {
    let calibration_id = settings
        .calibration_ids
        .get(arm)
        .with_context(|| format!("No calibration id for arm {arm}"))?;
    ensure!(
        !calibration_id.contains(['/', '\\'])
            && matches!(
                Path::new(calibration_id).components().next(),
                Some(Component::Normal(_))
            ),
        "Calibration ID must be a single file name"
    );
    Ok(so_follower_calibration_path(
        calibration_root,
        calibration_id,
    ))
}

/// Read and validate calibration entries at an explicit registry or local path.
///
/// # Errors
/// The file is unreadable, malformed, repeats a motor ID or lacks a driven motor.
pub fn load_calibration_path(
    settings: &FeetechSettings,
    path: &Path,
) -> anyhow::Result<BTreeMap<u8, LerobotMotorCalibration>> {
    let bytes = std::fs::read(path)
        .with_context(|| format!("Cannot read the calibration {}", path.display()))?;
    parse_calibration(settings, path, &bytes)
}

pub(crate) fn parse_calibration(
    settings: &FeetechSettings,
    path: &Path,
    bytes: &[u8],
) -> anyhow::Result<BTreeMap<u8, LerobotMotorCalibration>> {
    let entries = parse_named_calibration(path, bytes)?;
    calibration_by_motor_id(entries.into_values(), settings.driven_motor_ids(), path)
}

pub(crate) fn parse_named_calibration(
    path: &Path,
    bytes: &[u8],
) -> anyhow::Result<BTreeMap<String, LerobotMotorCalibration>> {
    serde_json::from_slice(bytes).with_context(|| format!("Cannot parse {}", path.display()))
}

pub(crate) fn calibration_by_motor_id(
    entries: impl IntoIterator<Item = LerobotMotorCalibration>,
    required_motor_ids: impl IntoIterator<Item = u8>,
    path: &Path,
) -> anyhow::Result<BTreeMap<u8, LerobotMotorCalibration>> {
    let mut by_id = BTreeMap::new();
    for entry in entries {
        let motor_id = entry.id;
        ensure!(
            by_id.insert(motor_id, entry).is_none(),
            "Calibration {} repeats motor ID {motor_id}",
            path.display()
        );
    }
    for entry in by_id.values() {
        ensure!(
            entry.drive_mode == 0,
            "Motor {} in {} must use drive_mode 0",
            entry.id,
            path.display()
        );
        validate_encoder_calibration(
            entry.id,
            entry.homing_offset,
            entry.range_min,
            entry.range_max,
        )
        .with_context(|| format!("Invalid calibration in {}", path.display()))?;
    }
    let missing: Vec<u8> = required_motor_ids
        .into_iter()
        .filter(|motor| !by_id.contains_key(motor))
        .collect();
    ensure!(
        missing.is_empty(),
        "Calibration {} has no motors {missing:?}",
        path.display()
    );
    Ok(by_id)
}

pub(crate) fn validate_encoder_calibration(
    motor_id: u8,
    homing_offset: i64,
    minimum: i64,
    maximum: i64,
) -> anyhow::Result<()> {
    const MAX_HOMING_OFFSET: i64 = STS3215_RESOLUTION / 2 - 1;
    ensure!(
        (-MAX_HOMING_OFFSET..=MAX_HOMING_OFFSET).contains(&homing_offset),
        "Motor {motor_id} homing offset {homing_offset} does not fit the encoder"
    );
    ensure!(
        0 <= minimum && minimum < maximum && maximum < STS3215_RESOLUTION,
        "Motor {motor_id} has invalid calibrated range {minimum}..{maximum}"
    );
    Ok(())
}

pub(crate) fn encoder_steps_to_radians(steps: i64, sign: i32, offset_radians: f64) -> f64 {
    f64::from(sign) * (crate::numeric::register_value(steps) - HOMING_STEPS) * std::f64::consts::TAU
        / crate::numeric::register_value(STS3215_RESOLUTION)
        + offset_radians
}

/// This arm's calibrated encoder endpoints as joint radians, lower then upper.
///
/// # Errors
/// The calibration cannot be read or a range does not fit the encoder.
pub fn calibrated_joint_limits(
    settings: &FeetechSettings,
    arm: &str,
    calibration_root: &Path,
) -> anyhow::Result<Vec<(f64, f64)>> {
    settings.validate_joint_alignment(settings.motor_ids.len())?;
    let calibration = load_calibration(settings, arm, calibration_root)?;
    Ok(settings
        .motor_ids
        .iter()
        .zip(&settings.joint_signs)
        .zip(&settings.joint_offsets_radians)
        .map(|((motor_id, sign), offset)| {
            let entry = &calibration[motor_id];
            let first = encoder_steps_to_radians(entry.range_min, sign.as_integer(), *offset);
            let second = encoder_steps_to_radians(entry.range_max, sign.as_integer(), *offset);
            (first.min(second), first.max(second))
        })
        .collect())
}
