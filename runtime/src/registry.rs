//! Install registry calibration without replacing an arm's local calibration.

use std::io::{ErrorKind, Write};
use std::path::{Path, PathBuf};

use anyhow::Context;
use uuid::Uuid;

use super::calibration::{calibration_path_for_arm, load_calibration_path, parse_calibration};
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
    let bytes = std::fs::read(source)
        .with_context(|| format!("Cannot read the calibration {}", source.display()))?;
    let calibration = parse_calibration(settings, source, &bytes)?;
    let destination = calibration_path_for_arm(settings, arm, calibration_root)?;
    let compare_local = || {
        Ok(
            if load_calibration_path(settings, &destination)? == calibration {
                CalibrationInstallation::Unchanged(destination.clone())
            } else {
                CalibrationInstallation::Conflict(destination.clone())
            },
        )
    };
    if destination.try_exists()? {
        return compare_local();
    }
    let parent = destination
        .parent()
        .context("Calibration path has no parent")?;
    std::fs::create_dir_all(parent)?;
    let pending = PendingCalibration(parent.join(format!(".{}.pending", Uuid::new_v4())));
    let mut file = std::fs::File::create_new(&pending.0)?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    match std::fs::hard_link(&pending.0, &destination) {
        Ok(()) => Ok(CalibrationInstallation::Installed(destination)),
        Err(error) if error.kind() == ErrorKind::AlreadyExists => compare_local(),
        Err(error) => {
            Err(error).context("Cannot publish calibration without replacing existing data")
        }
    }
}
