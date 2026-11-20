use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::calibration::so_follower_calibration_path;
use crate::discovery::ControllerSerial;
use crate::servo_registers::{
    self, GRIPPER_MAX_TORQUE_LIMIT, GRIPPER_PROTECTION_CURRENT, MAXIMUM_ACCELERATION,
    RETURN_DELAY_TIME,
};
use crate::setup::{
    Alignment, ArmSetup, GripperEndpoints, INCOMPLETE_CALIBRATION_FILE, ModelVariant, MotorIds,
    So101Joint,
};

use super::draft::{CalibrationEntries, Draft};
use super::hardware::{INCOMPLETE_RESET_FILE, SetupBus, read_arm, require_torque_off, write_new};

/// Per-arm changes to the standard servo registers, from `servo-overrides.toml` beside the
/// arm's `setup.toml`. Unset keys keep the standard value.
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct ServoRegisterOverrides {
    maximum_acceleration: Option<u8>,
    gripper_max_torque_limit: Option<u16>,
    gripper_protection_current: Option<u16>,
}

const REGISTERS: &[&str] = &[
    "homing_offset",
    "min_position_limit",
    "max_position_limit",
    "operating_mode",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "return_delay_time",
    "maximum_acceleration",
    "max_torque_limit",
    "protection_current",
    "lock",
];

type RegisterValues = BTreeMap<&'static str, [i64; 6]>;

#[derive(Debug, Clone)]
pub struct ArmFiles {
    directory: PathBuf,
    live_calibration: PathBuf,
    serial: ControllerSerial,
    new_model: ModelVariant,
}

#[derive(Debug, Clone, Serialize)]
pub struct SavedFiles {
    pub configuration: PathBuf,
    pub calibration: PathBuf,
    pub backup: PathBuf,
}

impl ArmFiles {
    #[must_use]
    pub fn new(
        directory: PathBuf,
        calibration_root: &Path,
        serial: ControllerSerial,
        new_model: ModelVariant,
    ) -> Self {
        let live_calibration =
            so_follower_calibration_path(calibration_root, &format!("so101_{}", serial.as_str()));
        Self {
            directory,
            live_calibration,
            serial,
            new_model,
        }
    }

    #[must_use]
    pub fn directory(&self) -> &Path {
        &self.directory
    }

    pub fn load(&self) -> Result<ArmSetup> {
        let path = self.directory.join("setup.toml");
        if path.try_exists()? {
            let setup = ArmSetup::parse(&fs::read_to_string(&path)?, &path)?;
            setup.validate(self.serial.as_str())?;
            return Ok(setup);
        }
        ensure!(
            !self.directory.join("profile.toml").try_exists()?,
            "This arm uses a host profile. Select a standalone setup directory."
        );
        Ok(ArmSetup {
            serial: self.serial.as_str().to_owned(),
            model_variant: self.new_model,
            motor_ids: MotorIds::default(),
            alignment: Alignment::default(),
            gripper: GripperEndpoints::default(),
        })
    }

    fn pending_calibration(&self) -> PathBuf {
        self.directory.join(INCOMPLETE_CALIBRATION_FILE)
    }

    pub(super) fn calibration_incomplete(&self) -> Result<bool> {
        Ok(self.pending_calibration().try_exists()?)
    }

    pub fn reset_incomplete(&self) -> Result<bool> {
        Ok(self
            .directory
            .join("backups")
            .join("servo-restore")
            .join(INCOMPLETE_RESET_FILE)
            .try_exists()?)
    }

    pub(super) fn require_completed_reset(&self) -> Result<()> {
        ensure!(
            !self.reset_incomplete()?,
            "An incomplete reset must be retried first"
        );
        Ok(())
    }

    #[must_use]
    pub fn calibration_path(&self) -> PathBuf {
        self.directory.join("calibration.json")
    }

    #[must_use]
    pub fn matches_setup(&self, setup: &ArmSetup) -> bool {
        self.directory.join("setup.toml").is_file()
            && self
                .calibration_incomplete()
                .is_ok_and(|incomplete| !incomplete)
            && self.load().is_ok_and(|saved| saved == *setup)
    }

    // WHY: A user whose arm misbehaves with the standard registers can change these three for
    // that arm alone, without touching control code; new setups never create the file.
    fn register_overrides(&self) -> Result<ServoRegisterOverrides> {
        let path = self.directory.join("servo-overrides.toml");
        if !path.try_exists()? {
            return Ok(ServoRegisterOverrides::default());
        }
        let overrides: ServoRegisterOverrides = toml::from_str(&fs::read_to_string(&path)?)
            .with_context(|| format!("Invalid {}", path.display()))?;
        ensure!(
            overrides
                .maximum_acceleration
                .is_none_or(|value| value <= 254)
                && overrides
                    .gripper_max_torque_limit
                    .is_none_or(|value| value <= 1000)
                && overrides
                    .gripper_protection_current
                    .is_none_or(|value| value <= 310),
            "Servo register override in {} is outside the supported range",
            path.display()
        );
        Ok(overrides)
    }

    pub(super) fn save(
        &self,
        bus: &mut impl SetupBus,
        setup: &ArmSetup,
        draft: &Draft,
        target: SaveTarget<'_>,
    ) -> Result<SavedFiles> {
        let ids = setup.motor_ids.ordered();
        require_torque_off(bus, ids)?;
        let entries = draft.entries(setup.motor_ids);
        let registers: RegisterValues = REGISTERS
            .iter()
            .map(|&register| Ok((register, read_arm(bus, ids, register)?)))
            .collect::<Result<_>>()?;
        let mut updates = calibration_registers(&entries);
        let indices: Vec<usize> = match target {
            SaveTarget::Joint { index, baseline } => {
                ensure!(
                    self.matches_setup(setup)
                        && setup
                            .motor_ids
                            .load_named_calibration(&self.calibration_path())?
                            == *baseline
                        && calibration_registers(baseline)
                            .iter()
                            .all(|(name, expected)| registers[name] == *expected),
                    "Saved setup or calibration changed. Reconnect before editing one joint."
                );
                vec![index]
            }
            SaveTarget::All => {
                self.add_servo_updates(&registers, &mut updates)?;
                (0..6).collect()
            }
        };
        let saved = SavedFiles {
            configuration: self.directory.join("setup.toml"),
            calibration: self.calibration_path(),
            backup: self
                .directory
                .join("backups")
                .join(Uuid::new_v4().to_string()),
        };
        let mut resulting_setup = setup.clone();
        resulting_setup.alignment = draft.alignment();
        resulting_setup.gripper = draft.gripper.clone();
        let config = toml::to_string_pretty(&resulting_setup)?;
        let calibration = serde_json::to_vec_pretty(&entries)?;
        fs::create_dir_all(&saved.backup)?;
        for (path, backup_name) in [
            (&saved.configuration, "setup.toml"),
            (&saved.calibration, "calibration.json"),
            (&self.live_calibration, "live-calibration.json"),
        ] {
            if path.try_exists()? {
                write_new(&saved.backup.join(backup_name), &fs::read(path)?)?;
            }
        }
        write_new(
            &saved.backup.join("registers.json"),
            &serde_json::to_vec_pretty(&RegisterBackup {
                motor_ids: ids,
                registers: &registers,
            })?,
        )?;
        let mut commit = || -> Result<()> {
            require_torque_off(bus, ids)?;
            let pending = self.pending_calibration();
            atomic_write(&pending, saved.backup.to_string_lossy().as_bytes())?;
            write_registers(
                bus,
                ids,
                &registers,
                &updates,
                &indices,
                matches!(target, SaveTarget::Joint { .. }),
            )?;
            require_torque_off(bus, ids)?;
            atomic_write(&saved.calibration, &calibration)?;
            atomic_write(&saved.configuration, config.as_bytes())?;
            atomic_write(&self.live_calibration, &calibration)?;
            fs::remove_file(&pending)?;
            Ok(())
        };
        commit().with_context(|| {
            format!(
                "Save stopped. Inspect the arm and backups at {} before continuing",
                saved.backup.display()
            )
        })?;
        Ok(saved)
    }

    fn add_servo_updates(
        &self,
        previous: &RegisterValues,
        updates: &mut RegisterValues,
    ) -> Result<()> {
        let overrides = self.register_overrides()?;
        let position_mode = rustypot::servo::feetech::sts3215::INFO
            .operating_modes
            .iter()
            .find(|(name, _)| *name == "position")
            .map(|(_, value)| i64::from(*value))
            .context("Missing STS3215 position mode")?;
        updates.insert("operating_mode", [position_mode; 6]);
        updates.insert("return_delay_time", [i64::from(RETURN_DELAY_TIME); 6]);
        updates.insert(
            "maximum_acceleration",
            [i64::from(
                overrides
                    .maximum_acceleration
                    .unwrap_or(MAXIMUM_ACCELERATION),
            ); 6],
        );
        let gripper_index = So101Joint::ALL.len() - 1;
        let mut max_torque = previous["max_torque_limit"];
        max_torque[gripper_index] = i64::from(
            overrides
                .gripper_max_torque_limit
                .unwrap_or(GRIPPER_MAX_TORQUE_LIMIT),
        );
        updates.insert("max_torque_limit", max_torque);
        let mut protection_current = previous["protection_current"];
        protection_current[gripper_index] = i64::from(
            overrides
                .gripper_protection_current
                .unwrap_or(GRIPPER_PROTECTION_CURRENT),
        );
        updates.insert("protection_current", protection_current);

        // INVARIANT: Register arrays follow MotorIds::ordered(), which is So101Joint::ALL order.
        let gains = So101Joint::ALL.map(servo_registers::position_gains);
        updates.insert(
            "p_coefficient",
            gains.map(|gain| i64::from(gain.proportional)),
        );
        updates.insert("i_coefficient", gains.map(|gain| i64::from(gain.integral)));
        updates.insert(
            "d_coefficient",
            gains.map(|gain| i64::from(gain.derivative)),
        );
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(super) enum SaveTarget<'a> {
    All,
    Joint {
        index: usize,
        baseline: &'a CalibrationEntries,
    },
}

#[derive(Serialize)]
struct RegisterBackup<'a> {
    motor_ids: [u8; 6],
    #[serde(flatten)]
    registers: &'a RegisterValues,
}

pub(super) fn calibration_registers(entries: &CalibrationEntries) -> RegisterValues {
    let motors = super::MOTOR_NAMES.map(|name| &entries[name]);
    BTreeMap::from([
        ("homing_offset", motors.map(|entry| entry.homing_offset)),
        ("min_position_limit", motors.map(|entry| entry.range_min)),
        ("max_position_limit", motors.map(|entry| entry.range_max)),
    ])
}

fn write_registers(
    bus: &mut impl SetupBus,
    ids: [u8; 6],
    previous: &RegisterValues,
    updates: &RegisterValues,
    indices: &[usize],
    preserve_lock: bool,
) -> Result<()> {
    let locked: Vec<_> = indices
        .iter()
        .copied()
        .filter(|&index| previous["lock"][index] != 0)
        .collect();
    let update = (|| -> Result<()> {
        for &index in &locked {
            bus.write(ids[index], "lock", 0)?;
        }
        for &register in REGISTERS {
            let Some(values) = updates.get(register) else {
                continue;
            };
            for &index in indices {
                if previous[register][index] != values[index] {
                    bus.write(ids[index], register, values[index])?;
                }
            }
            ensure!(
                read_arm(bus, ids, register)? == *values,
                "Calibration readback failed: {register}"
            );
        }
        Ok(())
    })();
    if preserve_lock {
        let lock_result = (|| -> Result<()> {
            for &index in &locked {
                bus.write(ids[index], "lock", previous["lock"][index])?;
            }
            ensure!(
                read_arm(bus, ids, "lock")? == previous["lock"],
                "Calibration readback failed: lock"
            );
            Ok(())
        })();
        if let Err(lock) = lock_result {
            return match update {
                Ok(()) => Err(lock),
                Err(error) => {
                    Err(error).context(format!("Could not restore EEPROM lock: {lock:#}"))
                }
            };
        }
    }
    update
}

fn atomic_write(path: &Path, contents: &[u8]) -> Result<()> {
    let parent = path.parent().context("Missing setup directory")?;
    fs::create_dir_all(parent)?;
    let temporary = parent.join(format!(".{}.pending", Uuid::new_v4()));
    let result = write_new(&temporary, contents).and_then(|()| {
        fs::rename(&temporary, path).with_context(|| format!("Cannot publish {}", path.display()))
    });
    if result.is_err() {
        let _ = fs::remove_file(temporary);
    }
    result
}

#[cfg(test)]
mod servo_register_override_tests {
    use super::{ArmFiles, ServoRegisterOverrides};
    use crate::discovery::ControllerSerial;
    use crate::setup::ModelVariant;

    struct ArmDirectory(std::path::PathBuf);

    impl Drop for ArmDirectory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn arm_with_overrides(
        name: &str,
        overrides: Option<&str>,
    ) -> anyhow::Result<(ArmDirectory, ArmFiles)> {
        let directory = std::env::temp_dir().join(format!(
            "so101-servo-overrides-{name}-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&directory)?;
        if let Some(text) = overrides {
            std::fs::write(directory.join("servo-overrides.toml"), text)?;
        }
        let files = ArmFiles::new(
            directory.clone(),
            &directory,
            ControllerSerial::parse("ARM_A")?,
            ModelVariant::SoftGripperCamera,
        );
        Ok((ArmDirectory(directory), files))
    }

    #[test]
    fn an_arm_without_an_override_file_runs_the_standard_registers() -> anyhow::Result<()> {
        let (_directory, files) = arm_with_overrides("absent", None)?;
        let overrides: ServoRegisterOverrides = files.register_overrides()?;
        assert!(overrides.maximum_acceleration.is_none());
        assert!(overrides.gripper_max_torque_limit.is_none());
        assert!(overrides.gripper_protection_current.is_none());
        Ok(())
    }

    #[test]
    fn an_override_file_changes_only_the_keys_it_sets() -> anyhow::Result<()> {
        let (_directory, files) =
            arm_with_overrides("partial", Some("maximum_acceleration = 100\n"))?;
        let overrides = files.register_overrides()?;
        assert_eq!(overrides.maximum_acceleration, Some(100));
        assert!(overrides.gripper_max_torque_limit.is_none());
        Ok(())
    }

    #[test]
    fn out_of_range_or_unknown_overrides_are_refused() -> anyhow::Result<()> {
        let (_too_high, files) =
            arm_with_overrides("range", Some("gripper_protection_current = 400\n"))?;
        assert!(files.register_overrides().is_err());
        let (_unknown, files) = arm_with_overrides("unknown", Some("p_coefficient = 8\n"))?;
        assert!(files.register_overrides().is_err());
        Ok(())
    }
}
