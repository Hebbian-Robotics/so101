use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use hebbian_so101::commissioning::{ARM_IDS, ArmFiles, Session, SetupBus, Stage};
use hebbian_so101::discovery::ControllerSerial;
use hebbian_so101::setup::{ArmSetup, ModelVariant, MotorIds};
use uuid::Uuid;

struct Files(PathBuf);

impl Files {
    fn new() -> Result<Self> {
        let path = std::env::temp_dir().join(format!("so101-commissioning-{}", Uuid::new_v4()));
        std::fs::create_dir(&path)?;
        Ok(Self(path))
    }

    fn arm(&self) -> Result<ArmFiles> {
        Ok(ArmFiles::new(
            self.0.join("arms/board"),
            &self.0.join("lerobot"),
            ControllerSerial::parse("board")?,
            ModelVariant::SoftGripperCamera,
        ))
    }
}

impl Drop for Files {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[derive(Clone)]
struct MemoryBus {
    registers: Arc<Mutex<BTreeMap<String, [i64; 6]>>>,
    refused_write: Option<&'static str>,
}

impl MemoryBus {
    fn new() -> Self {
        let registers = BTreeMap::from([
            ("id", ARM_IDS.map(i64::from)),
            ("torque_enable", [0; 6]),
            ("present_position", [3800; 6]),
            ("homing_offset", [0; 6]),
            ("min_position_limit", [0; 6]),
            ("max_position_limit", [4095; 6]),
            ("operating_mode", [0; 6]),
            ("p_coefficient", [0; 6]),
            ("i_coefficient", [0; 6]),
            ("d_coefficient", [0; 6]),
            ("return_delay_time", [0; 6]),
            ("maximum_acceleration", [0; 6]),
            ("max_torque_limit", [1000; 6]),
            ("protection_current", [0; 6]),
            ("lock", [1; 6]),
        ]);
        Self {
            registers: Arc::new(Mutex::new(
                registers
                    .into_iter()
                    .map(|(name, values)| (name.to_owned(), values))
                    .collect(),
            )),
            refused_write: None,
        }
    }

    fn registers(&self) -> BTreeMap<String, [i64; 6]> {
        self.registers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    fn set(&self, register: &str, values: [i64; 6]) {
        self.registers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(register.into(), values);
    }
}

impl SetupBus for MemoryBus {
    fn read(&mut self, ids: &[u8], register: &str) -> Result<Vec<i64>> {
        let registers = self
            .registers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let values = registers
            .get(register)
            .with_context(|| format!("Unknown register {register}"))?;
        Ok(ids.iter().map(|&id| values[usize::from(id - 1)]).collect())
    }

    fn write(&mut self, id: u8, register: &str, value: i64) -> Result<()> {
        anyhow::ensure!(
            self.refused_write != Some(register),
            "Servo refused {register}"
        );
        let mut registers = self
            .registers
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let index = usize::from(id - 1);
        if register == "homing_offset" {
            let previous = registers["homing_offset"][index];
            registers
                .get_mut("present_position")
                .context("Missing encoder")?[index] =
                (registers["present_position"][index] + previous - value).rem_euclid(4096);
        }
        registers
            .get_mut(register)
            .with_context(|| format!("Unknown register {register}"))?[index] = value;
        Ok(())
    }
}

#[test]
fn calibration_keeps_physical_sweeps_when_remapped_and_saves_runtime_files() -> Result<()> {
    let root = Files::new()?;
    let files = root.arm()?;
    let bus = MemoryBus::new();
    let mut session = Session::connect(bus.clone(), files.clone())?;
    assert_eq!(session.stage(), Stage::Reference);
    let initial = bus.registers();
    session.start_sweep(None)?;
    assert!(
        session.finish_sweep().is_err(),
        "A stationary arm must not produce a draft"
    );
    bus.set("present_position", [4000; 6]);
    session.sample()?;
    let mapping = MotorIds::from_joint_order([6, 5, 4, 3, 2, 1])?;
    session.remap(mapping)?;
    for positions in [
        [100, 200, 250, 150, 200, 100],
        [550, 800, 750, 800, 800, 850],
        [800, 1700, 1600, 1700, 1800, 1700],
        [820, 2600, 1610, 1730, 1850, 1770],
    ] {
        bus.set("present_position", positions);
        session.sample()?;
    }
    session.finish_sweep()?;
    assert_eq!(session.stage(), Stage::Review);
    let draft = session.view();
    assert_eq!(draft.joints[0].range_max - draft.joints[0].range_min, 2066);
    assert_eq!(draft.joints[5].range_max - draft.joints[5].range_min, 1116);
    for register in [
        "homing_offset",
        "min_position_limit",
        "max_position_limit",
        "lock",
    ] {
        assert_eq!(
            bus.registers()[register],
            initial[register],
            "{register} changed during capture"
        );
    }
    assert!(!files.calibration_path().exists());

    session.save()?;
    assert_eq!(session.stage(), Stage::Saved);
    let saved = ArmSetup::load(&files.directory().join("setup.toml"), "board")?;
    let entries = saved
        .motor_ids
        .load_named_calibration(&files.calibration_path())?;
    assert_eq!(saved.motor_ids.ordered(), [6, 5, 4, 3, 2, 1]);
    assert_eq!(entries["shoulder_pan"].id, 6);
    assert_eq!(entries["gripper"].id, 1);
    assert!(
        root.0
            .join("lerobot/robots/so_follower/so101_board.json")
            .is_file()
    );
    assert_eq!(bus.registers()["p_coefficient"], [16, 16, 16, 32, 32, 16]);

    let mut reopened = Session::connect(bus.clone(), files.clone())?;
    assert!(reopened.view().can_recalibrate_joint);
    reopened.start_sweep(Some(0))?;
    let mut positions = bus.registers()["present_position"];
    positions[5] = (positions[5] - 1600).rem_euclid(4096);
    bus.set("present_position", positions);
    reopened.finish_sweep()?;
    assert!(reopened.edit(1, Some(0.02)).is_err());
    reopened.edit(0, Some(0.02))?;
    let before = bus.registers();
    reopened.save()?;
    let after = bus.registers();
    assert_eq!(after["p_coefficient"], before["p_coefficient"]);
    assert_eq!(after["homing_offset"][..5], before["homing_offset"][..5]);
    assert_eq!(after["lock"], before["lock"]);

    let mut interrupted_bus = bus.clone();
    interrupted_bus.refused_write = Some("p_coefficient");
    bus.set("p_coefficient", [0; 6]);
    let mut interrupted = Session::connect(interrupted_bus, files.clone())?;
    let failure = interrupted
        .save()
        .err()
        .context("Accepted an incomplete write")?;
    assert!(format!("{failure:#}").contains("Servo refused p_coefficient"));
    assert!(ArmSetup::load(&files.directory().join("setup.toml"), "board").is_err());
    let recovery = Session::connect(bus, files)?;
    assert_eq!(recovery.stage(), Stage::Reference);
    assert!(
        recovery
            .view()
            .message
            .contains("previous save did not finish")
    );
    Ok(())
}

#[test]
fn unsafe_torque_or_a_changed_reconnected_arm_cannot_continue_calibration() -> Result<()> {
    let root = Files::new()?;
    let files = root.arm()?;
    let bus = MemoryBus::new();
    bus.set("torque_enable", [0, 0, 0, 0, 0, 1]);
    assert!(Session::connect(bus.clone(), files.clone()).is_err());
    bus.set("torque_enable", [0; 6]);
    let mut draft = Session::connect(bus.clone(), files)?;
    draft.start_sweep(None)?;
    draft.pause();
    bus.set("homing_offset", [1, 0, 0, 0, 0, 0]);
    assert!(draft.reconnect(bus.clone()).is_err());
    assert!(!draft.has_bus());
    bus.set("homing_offset", [0; 6]);
    draft.reconnect(bus)?;
    assert_eq!(draft.stage(), Stage::Sweep);
    Ok(())
}
