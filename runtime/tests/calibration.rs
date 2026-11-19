//! Calibration files must describe a complete, usable set of physical motors.

use hebbian_so101::{SO101_JOINT_NAMES, setup::MotorIds};
use serde_json::json;

#[test]
fn calibration_mapping_requires_every_motor_and_valid_ranges() -> anyhow::Result<()> {
    let path = std::env::temp_dir().join(format!("so101-{}.json", uuid::Uuid::new_v4()));
    let complete: serde_json::Map<_, _> = SO101_JOINT_NAMES
        .into_iter()
        .chain(["gripper"])
        .zip(1..=6)
        .map(|(name, motor_id)| {
            (
                name.to_owned(),
                json!({
                    "id": motor_id,
                    "drive_mode": 0,
                    "homing_offset": 0,
                    "range_min": 500,
                    "range_max": 3500,
                }),
            )
        })
        .collect();
    let result = (|| -> anyhow::Result<()> {
        std::fs::write(&path, serde_json::to_vec(&complete)?)?;
        MotorIds::default().validate_calibration_mapping(&path)?;

        let mut incomplete = complete.clone();
        incomplete.remove("gripper");
        std::fs::write(&path, serde_json::to_vec(&incomplete)?)?;
        assert!(
            MotorIds::default()
                .validate_calibration_mapping(&path)
                .is_err()
        );

        let mut invalid = complete;
        invalid["gripper"]["range_max"] = json!(4096);
        std::fs::write(&path, serde_json::to_vec(&invalid)?)?;
        assert!(
            MotorIds::default()
                .validate_calibration_mapping(&path)
                .is_err()
        );
        Ok(())
    })();
    std::fs::remove_file(path)?;
    result
}
