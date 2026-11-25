use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, anyhow, ensure};
use rustypot::bus::Bus;
use rustypot::servo::feetech::sts3215::{DEFINITION, INFO};
use uuid::Uuid;

use crate::driver::{BAUD_RATE, SerialBus, open_serial};

pub const ARM_IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];
pub const INCOMPLETE_RESET_FILE: &str = "reset-incomplete";
const STS3215_MODEL: u16 = 777;

pub trait SetupBus {
    fn read(&mut self, ids: &[u8], register: &str) -> Result<Vec<i64>>;
    fn write(&mut self, id: u8, register: &str, value: i64) -> Result<()>;
}

impl SetupBus for SerialBus {
    fn read(&mut self, ids: &[u8], register: &str) -> Result<Vec<i64>> {
        crate::driver::ServoBus::read(self, ids, register)
    }

    fn write(&mut self, id: u8, register: &str, value: i64) -> Result<()> {
        self.write_acknowledged(id, register, value)
    }
}

pub(super) fn read_arm(bus: &mut impl SetupBus, ids: [u8; 6], register: &str) -> Result<[i64; 6]> {
    bus.read(&ids, register)?
        .try_into()
        .map_err(|values: Vec<i64>| {
            anyhow!(
                "Expected six replies for {register}, received {}",
                values.len()
            )
        })
}

pub(super) fn require_torque_off(bus: &mut impl SetupBus, ids: [u8; 6]) -> Result<()> {
    ensure!(
        read_arm(bus, ids, "torque_enable")? == [0; 6],
        "Support the arm and disable torque on all six servos"
    );
    Ok(())
}

/// Program and verify a single isolated STS3215 at the runtime baud rate.
pub fn program_motor_id(path: &str, target_id: u8) -> Result<u8> {
    ensure!(ARM_IDS.contains(&target_id), "Motor ID must be 1–6");
    let serial = open_serial(path, Duration::from_millis(50))?;
    let mut bus = Bus::new(serial, (0..254).map(|id| (id, DEFINITION)).collect());
    for &(baud, _) in INFO.baudrates {
        bus.set_baudrate(baud).map_err(|error| anyhow!("{error}"))?;
        let motors = bus
            .broadcast_scan_all(DEFINITION)
            .map_err(|error| anyhow!("{error}"))?;
        if motors.is_empty() {
            continue;
        }
        ensure!(
            motors.len() == 1,
            "Connect only the loose motor being programmed"
        );
        let (&current_id, &model) = motors.first_key_value().context("Missing servo response")?;
        ensure!(
            model == STS3215_MODEL,
            "Expected STS3215, found model {model}"
        );
        verify_register(&mut bus, current_id, "torque_enable", 0)?;
        if baud != BAUD_RATE {
            bus.change_baudrate(DEFINITION, current_id, BAUD_RATE)
                .map_err(|error| anyhow!("{error}"))?;
            bus.set_baudrate(BAUD_RATE)
                .map_err(|error| anyhow!("{error}"))?;
            verify_register(&mut bus, current_id, "baudrate", 0)?;
        }
        if current_id != target_id {
            bus.change_id(DEFINITION, current_id, target_id)
                .map_err(|error| anyhow!("{error}"))?;
        }
        verify_register(&mut bus, target_id, "id", i64::from(target_id))?;
        verify_register(&mut bus, target_id, "baudrate", 0)?;
        if verify_register(&mut bus, target_id, "lock", 1).is_err() {
            bus.write_register(target_id, "lock", 1)
                .map_err(|error| anyhow!("{error}"))?;
            verify_register(&mut bus, target_id, "lock", 1)?;
        }
        return Ok(target_id);
    }
    anyhow::bail!("No motor responded. Check servo power and its cable.")
}

fn verify_register(bus: &mut Bus, id: u8, name: &str, expected: i64) -> Result<()> {
    let value = bus
        .with_retries(2, |bus| bus.read_register(id, name))
        .map_err(|error| anyhow!("{error}"))?;
    ensure!(
        value == expected,
        "Motor {id}: expected {name}={expected}, got {value}"
    );
    Ok(())
}

const RESTORE_REGISTERS: &[&str] = &[
    "id",
    "model_number",
    "firmware_major_version",
    "firmware_minor_version",
    "baud_rate",
    "response_status_level",
    "return_delay_time",
    "homing_offset",
    "min_position_limit",
    "max_position_limit",
    "operating_mode",
    "phase",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "maximum_acceleration",
    "maximum_velocity_limit",
    "acceleration_multiplier",
    "max_torque_limit",
    "protection_current",
    "minimum_startup_force",
    "cw_dead_zone",
    "ccw_dead_zone",
    "max_temperature_limit",
    "max_voltage_limit",
    "min_voltage_limit",
    "protective_torque",
    "protection_time",
    "overload_torque",
    "lock",
];

fn snapshot(bus: &mut impl SetupBus, id: u8) -> Result<BTreeMap<&'static str, i64>> {
    RESTORE_REGISTERS
        .iter()
        .map(|&register| {
            let value = bus.read(&[id], register)?;
            ensure!(value.len() == 1, "Missing {register} from motor {id}");
            Ok((register, value[0]))
        })
        .collect()
}

fn require_original_ids(bus: &mut impl SetupBus) -> Result<()> {
    ensure!(
        read_arm(bus, ARM_IDS, "id")? == ARM_IDS.map(i64::from),
        "Could not verify all six motor IDs"
    );
    require_torque_off(bus, ARM_IDS)
}

/// Restore parameters on selected motors; the entire assembled arm must answer
/// and have torque off. Each reset is addressed individually and backed up.
pub fn restore_servos(path: &str, ids: &[u8], backup_directory: &Path) -> Result<Vec<PathBuf>> {
    ensure!(
        !ids.is_empty()
            && ids
                .iter()
                .enumerate()
                .all(|(index, id)| { ARM_IDS.contains(id) && !ids[..index].contains(id) }),
        "Select one or more motor IDs in 1–6"
    );
    let mut bus = SerialBus::open(path, &ARM_IDS)?;
    require_original_ids(&mut bus)?;
    for &id in ids {
        let model = bus.read(&[id], "model_number")?;
        let major = bus.read(&[id], "firmware_major_version")?;
        let minor = bus.read(&[id], "firmware_minor_version")?;
        ensure!(
            model == [i64::from(STS3215_MODEL)] && major == [3] && minor == [10],
            "Motor {id}: restore is verified only for STS3215 model 777 firmware 3.10"
        );
    }
    let marker = backup_directory.join(INCOMPLETE_RESET_FILE);
    if ids.len() == ARM_IDS.len() && !marker.try_exists()? {
        std::fs::create_dir_all(backup_directory)?;
        write_new(&marker, b"Retry the full reset before calibrating.\n")?;
    }
    let backups = ids
        .iter()
        .map(|&id| {
            restore_servo(&mut bus, id, backup_directory)
                .with_context(|| format!("Reset stopped on motor {id}"))
        })
        .collect::<Result<_>>()?;
    if ids.len() == ARM_IDS.len() {
        std::fs::remove_file(marker)?;
    }
    Ok(backups)
}

fn restore_servo(bus: &mut SerialBus, id: u8, backup_directory: &Path) -> Result<PathBuf> {
    require_original_ids(bus)?;
    let before = snapshot(bus, id)?;
    std::fs::create_dir_all(backup_directory)?;
    let backup = backup_directory.join(format!("motor-{id}-{}.json", Uuid::new_v4()));
    write_new(&backup, &serde_json::to_vec_pretty(&before)?)?;
    let previous_lock = before["lock"];
    let restore = (|| -> Result<()> {
        if previous_lock != 0 {
            bus.write(id, "lock", 0)?;
        }
        ensure!(bus.read(&[id], "lock")? == [0], "EEPROM did not unlock");
        require_original_ids(bus)?;
        bus.set_timeout(Duration::from_millis(300))?;
        bus.reset_parameters_keep_id(id)
    })();
    std::thread::sleep(Duration::from_millis(500));
    let verify = (|| -> Result<()> {
        bus.set_timeout(Duration::from_millis(50))?;
        require_original_ids(bus)?;
        if bus.read(&[id], "lock")? != [previous_lock] {
            bus.write(id, "lock", previous_lock)?;
        }
        let after = snapshot(bus, id)?;
        write_new(
            &backup.with_extension("after.json"),
            &serde_json::to_vec_pretty(&after)?,
        )?;
        ensure!(
            after["lock"] == previous_lock,
            "EEPROM lock was not restored"
        );
        Ok(())
    })();
    match (restore, verify) {
        (Ok(()), Ok(())) => Ok(backup),
        (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error)
            .with_context(|| format!("Restore outcome uncertain. Backup: {}", backup.display())),
        (Err(restore), Err(verify)) => anyhow::bail!(
            "Restore outcome uncertain: {restore:#}. Verification failed: {verify:#}. Backup: {}",
            backup.display()
        ),
    }
}

pub(super) fn write_new(path: &Path, contents: &[u8]) -> Result<()> {
    let mut file = std::fs::File::create_new(path)
        .with_context(|| format!("Cannot create {}", path.display()))?;
    file.write_all(contents)?;
    file.sync_all()?;
    Ok(())
}
