//! An SO-101's `LeRobot` calibration: where it lives, its entries, and the
//! encoder endpoints as joint radians.
//!
//! `LeRobot`'s homing leaves its captured reference near mid-turn. Mapping that
//! reference to the URDF requires per-arm signs and offsets; a step count maps as
//! `sign * (steps - 2047) * 2 pi / 4096 + offset`. Per arm, verify with torque off
//! that the calibration pose reads as the URDF's zero pose and that each joint
//! turns the right way; the profile's signs and offsets absorb any difference.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, ensure};
use serde::Deserialize;

use super::settings::{FeetechSettings, Sign};

/// The step count of the calibration pose.
const HOMING_STEPS: f64 = 2047.0;
/// Encoder steps per turn of the Feetech STS3215.
pub const STS3215_RESOLUTION: i64 = 4096;

/// One motor's entry in a `LeRobot` calibration file.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
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
    let calibration_id = settings
        .calibration_ids
        .get(arm)
        .with_context(|| format!("No calibration id for arm {arm}"))?;
    let path = so_follower_calibration_path(calibration_root, calibration_id);
    load_calibration_path(settings, &path)
}

/// Read and validate calibration entries at an explicit registry or local path.
///
/// # Errors
/// The file is unreadable, malformed, repeats a motor ID or lacks a driven motor.
pub fn load_calibration_path(
    settings: &FeetechSettings,
    path: &Path,
) -> anyhow::Result<BTreeMap<u8, LerobotMotorCalibration>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("Cannot read the calibration {}", path.display()))?;
    let entries: BTreeMap<String, LerobotMotorCalibration> =
        serde_json::from_str(&text).with_context(|| format!("Cannot parse {}", path.display()))?;
    let mut by_id = BTreeMap::new();
    for entry in entries.into_values() {
        let motor_id = entry.id;
        ensure!(
            by_id.insert(motor_id, entry).is_none(),
            "Calibration {} repeats motor ID {motor_id}",
            path.display()
        );
    }
    for entry in by_id.values() {
        // LeRobot's motor bus refuses such a motor too: normalizing divides by it.
        ensure!(
            entry.range_min != entry.range_max,
            "Motor {} in {} has equal range_min and range_max",
            entry.id,
            path.display()
        );
    }
    let missing: Vec<u8> = settings
        .driven_motor_ids()
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

#[allow(clippy::cast_precision_loss, reason = "step counts are below 2^12")]
fn steps_to_radians(steps: i64, sign: Sign, offset_radians: f64) -> f64 {
    f64::from(sign.as_integer()) * (steps as f64 - HOMING_STEPS) * std::f64::consts::TAU
        / STS3215_RESOLUTION as f64
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
    let calibration = load_calibration(settings, arm, calibration_root)?;
    settings
        .motor_ids
        .iter()
        .zip(&settings.joint_signs)
        .zip(&settings.joint_offsets_radians)
        .map(|((motor_id, sign), offset)| {
            let entry = &calibration[motor_id];
            ensure!(
                0 <= entry.range_min
                    && entry.range_min < entry.range_max
                    && entry.range_max < STS3215_RESOLUTION,
                "Motor {motor_id} has invalid calibrated range {}..{} for resolution {STS3215_RESOLUTION}",
                entry.range_min,
                entry.range_max
            );
            let first = steps_to_radians(entry.range_min, *sign, *offset);
            let second = steps_to_radians(entry.range_max, *sign, *offset);
            Ok((first.min(second), first.max(second)))
        })
        .collect()
}
