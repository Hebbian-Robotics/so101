//! Install registry calibration without replacing an arm's local calibration.

use std::io::{ErrorKind, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, ensure};
use uuid::Uuid;

use super::calibration::{STS3215_RESOLUTION, load_calibration_path, so_follower_calibration_path};
use super::settings::FeetechSettings;

/// A differing local calibration needs reconciliation, never silent replacement.
#[derive(Debug, PartialEq, Eq)]
pub enum CalibrationInstallation {
    Installed(PathBuf),
    Unchanged(PathBuf),
    Conflict(PathBuf),
}

struct PendingCalibration(PathBuf);

impl Drop for PendingCalibration {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

/// Atomically publish a complete calibration file if no destination exists.
///
/// # Errors
/// The registry calibration is invalid, or a file cannot be read/written.
/// A valid but differing local calibration is returned as `Conflict`.
pub fn install_calibration(
    source: &Path,
    settings: &FeetechSettings,
    arm: &str,
    calibration_root: &Path,
) -> anyhow::Result<CalibrationInstallation> {
    let calibration = load_calibration_path(settings, source)?;
    for entry in calibration.values() {
        ensure!(
            entry.drive_mode == 0,
            "SO-101 registry calibration must use drive_mode 0"
        );
        ensure!(
            0 <= entry.range_min
                && entry.range_min < entry.range_max
                && entry.range_max < STS3215_RESOLUTION,
            "Motor {} has invalid calibration range {}..{}",
            entry.id,
            entry.range_min,
            entry.range_max
        );
    }
    let calibration_id = settings
        .calibration_ids
        .get(arm)
        .context("Arm has no calibration ID")?;
    ensure!(
        Path::new(calibration_id).components().count() == 1
            && matches!(
                Path::new(calibration_id).components().next(),
                Some(std::path::Component::Normal(_))
            ),
        "Calibration ID must be a single file name"
    );
    let destination = so_follower_calibration_path(calibration_root, calibration_id);
    if destination.try_exists()? {
        return Ok(
            if load_calibration_path(settings, &destination)? == calibration {
                CalibrationInstallation::Unchanged(destination)
            } else {
                CalibrationInstallation::Conflict(destination)
            },
        );
    }
    let parent = destination
        .parent()
        .context("Calibration path has no parent")?;
    std::fs::create_dir_all(parent)?;
    let pending = PendingCalibration(parent.join(format!(".{}.pending", Uuid::new_v4())));
    let mut file = std::fs::File::create_new(&pending.0)?;
    // Re-read and validate the exact bytes we publish, even if a checkout changed
    // between its initial validation and this copy.
    let bytes = std::fs::read(source)?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    ensure!(
        load_calibration_path(settings, &pending.0)? == calibration,
        "Registry calibration changed during installation; retry"
    );
    match std::fs::hard_link(&pending.0, &destination) {
        Ok(()) => Ok(CalibrationInstallation::Installed(destination)),
        Err(error) if error.kind() == ErrorKind::AlreadyExists => Ok(
            if load_calibration_path(settings, &destination)? == calibration {
                CalibrationInstallation::Unchanged(destination)
            } else {
                CalibrationInstallation::Conflict(destination)
            },
        ),
        Err(error) => {
            Err(error).context("Cannot publish calibration without replacing existing data")
        }
    }
}
