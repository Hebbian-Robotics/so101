//! Tests of the Feetech servos against a fake bus.

use std::collections::HashMap;
use std::f64::consts::PI;

use anyhow::bail;
use rustypot::servo::Encoding;

use super::bus::{STATE_BLOCK_LENGTH, ServoBus, StateBlock};
use super::servos::Servos;
use super::servos::TORQUE_READ_INTERVAL;
use super::units::{VelocityEstimator, radians_to_steps, steps_to_radians};
use crate::config::{FeetechConfig, FeetechGripper, FeetechMotor, Gains, MotorCalibration};
use crate::telemetry::Reconnection;

fn motor(id: u8, sign: i32) -> FeetechMotor {
    FeetechMotor {
        id,
        sign,
        offset_radians: 0.1,
        calibration: MotorCalibration {
            homing_offset: 10 * i64::from(id),
            range_min: 500,
            range_max: 3500,
        },
        expected_gains: Gains { p: 16, i: 0, d: 32 },
    }
}

fn config() -> FeetechConfig {
    FeetechConfig {
        port: "unused".into(),
        joints: vec![motor(1, 1), motor(2, -1)],
        gripper: Some(FeetechGripper {
            motor: motor(3, 1),
            force_fraction: 0.5,
        }),
        passive_motor_ids: vec![],
        expected_return_delay_time: Some(0),
        max_temperature_celsius: 65.0,
        min_voltage_volts: 10.5,
        max_voltage_volts: 13.0,
    }
}

/// A bus of three healthy servos at rest at the calibration pose.
struct FakeBus {
    registers: HashMap<(u8, String), i64>,
    /// Goal writes land this far off, as a servo that did not take them.
    goal_error: i64,
    /// A motor continues moving after the hold goal was written.
    drift_after_goal: Option<(u8, i64)>,
    torque_changes: Vec<(Vec<u8>, bool)>,
    fail_reads: bool,
    /// Replies carry one value fewer than asked for, as a bus that dropped a motor.
    short_replies: bool,
    /// Bus read transactions so far.
    transactions: usize,
    confirmation_override: Option<(u8, i64, i64)>,
    fail_confirmation: bool,
}

impl FakeBus {
    fn healthy() -> Self {
        let mut registers = HashMap::new();
        for id in 1..=3_u8 {
            for (name, value) in [
                ("torque_enable", 0),
                ("homing_offset", 10 * i64::from(id)),
                ("min_position_limit", 500),
                ("max_position_limit", 3500),
                ("operating_mode", 0),
                ("p_coefficient", 16),
                ("i_coefficient", 0),
                ("d_coefficient", 32),
                ("present_voltage", 120),
                ("present_load", 0),
                ("present_speed", 0),
                ("return_delay_time", 0),
                ("status", 0),
                ("packet_status", 0),
                ("present_temperature", 30),
                ("present_position", 2047),
                ("goal_position", 0),
            ] {
                registers.insert((id, name.to_owned()), value);
            }
        }
        Self {
            registers,
            goal_error: 0,
            drift_after_goal: None,
            torque_changes: Vec::new(),
            fail_reads: false,
            short_replies: false,
            transactions: 0,
            confirmation_override: None,
            fail_confirmation: false,
        }
    }

    fn set(&mut self, id: u8, name: &str, value: i64) {
        self.registers.insert((id, name.to_owned()), value);
    }
}

impl ServoBus for FakeBus {
    /// Serve registers 56..=65 as the servo's bytes, so decoding is exercised.
    fn read_state_block(&mut self, ids: &[u8]) -> anyhow::Result<Vec<StateBlock>> {
        if self.fail_reads {
            bail!("no answer");
        }
        self.transactions += 1;
        if ids.len() == 1 && self.fail_confirmation {
            bail!("confirmation failed");
        }
        let served = ids.len() - usize::from(self.short_replies);
        ids[..served]
            .iter()
            .map(|id| {
                let register = |name: &str| self.registers[&(*id, name.to_owned())];
                let magnitude = Encoding::SignMagnitude { sign_bit: 15 };
                let mut bytes = vec![0_u8; usize::from(STATE_BLOCK_LENGTH)];
                let words = [
                    (0, magnitude.encode(register("present_position"), 2)),
                    (2, magnitude.encode(register("present_speed"), 2)),
                    (
                        4,
                        Encoding::SignMagnitude { sign_bit: 10 }
                            .encode(register("present_load"), 2),
                    ),
                ];
                for (offset, word) in words {
                    let word = u16::try_from(word.unwrap()).unwrap();
                    bytes[offset..offset + 2].copy_from_slice(&word.to_le_bytes());
                }
                bytes[6] = u8::try_from(register("present_voltage")).unwrap();
                bytes[7] = u8::try_from(register("present_temperature")).unwrap();
                bytes[9] = u8::try_from(register("status")).unwrap();
                let mut block = StateBlock::decode(&bytes)?;
                if ids.len() == 1
                    && let Some((motor_id, temperature, voltage)) = self.confirmation_override
                    && *id == motor_id
                {
                    block.temperature = temperature;
                    block.voltage = voltage;
                }
                block.packet_status = u8::try_from(register("packet_status"))?;
                Ok(block)
            })
            .collect()
    }

    fn read(&mut self, ids: &[u8], register: &str) -> anyhow::Result<Vec<i64>> {
        if self.fail_reads {
            bail!("no answer");
        }
        self.transactions += 1;
        let served = ids.len() - usize::from(self.short_replies);
        Ok(ids[..served]
            .iter()
            .map(|id| self.registers[&(*id, register.to_owned())])
            .collect())
    }

    fn write(&mut self, ids: &[u8], register: &str, values: &[i64]) -> anyhow::Result<()> {
        for (id, value) in ids.iter().zip(values) {
            let stored = if register == "goal_position" {
                value + self.goal_error
            } else {
                *value
            };
            self.registers.insert((*id, register.to_owned()), stored);
        }
        if register == "goal_position"
            && let Some((id, drift)) = self.drift_after_goal.take()
        {
            let position = self.registers[&(id, "present_position".into())];
            self.set(id, "present_position", position + drift);
        }
        Ok(())
    }

    fn reconnect(&mut self) -> Reconnection {
        if self.fail_reads {
            Reconnection::Failed(anyhow::anyhow!("no answer"))
        } else {
            Reconnection::Restored
        }
    }

    fn set_torque(&mut self, ids: &[u8], enabled: bool) -> anyhow::Result<()> {
        self.torque_changes.push((ids.to_vec(), enabled));
        for id in ids {
            self.set(*id, "torque_enable", i64::from(enabled));
        }
        Ok(())
    }
}

fn servos(bus: FakeBus) -> Servos<FakeBus> {
    Servos::new(bus, config()).unwrap()
}

#[test]
fn malformed_runtime_configuration_is_refused_before_constructing_servos() {
    type ConfigurationMutation = fn(&mut FeetechConfig);
    let invalid_configurations: &[(&str, ConfigurationMutation)] = &[
        ("nonfinite offset", |config| {
            config.joints[0].offset_radians = f64::NAN;
        }),
        ("nonfinite minimum voltage", |config| {
            config.min_voltage_volts = f64::NAN;
        }),
        ("nonfinite maximum voltage", |config| {
            config.max_voltage_volts = f64::INFINITY;
        }),
        ("nonfinite temperature", |config| {
            config.max_temperature_celsius = f64::NAN;
        }),
        ("negative temperature", |config| {
            config.max_temperature_celsius = -1.0;
        }),
        ("negative voltage", |config| config.min_voltage_volts = -1.0),
        ("empty joints", |config| config.joints.clear()),
        ("duplicate joint IDs", |config| {
            config.joints[1].id = config.joints[0].id;
        }),
        ("passive driven overlap", |config| {
            config.passive_motor_ids.push(1);
        }),
        ("duplicate passive IDs", |config| {
            config.passive_motor_ids = vec![4, 4];
        }),
        ("broadcast motor ID", |config| config.joints[0].id = 254),
        ("oversized encoder range", |config| {
            config.joints[0].calibration.range_max = 4096;
        }),
        ("unrepresentable homing offset", |config| {
            config.joints[0].calibration.homing_offset = 2048;
        }),
        ("unrepresentable position gain", |config| {
            config.joints[0].expected_gains.p = 256;
        }),
        ("invalid return delay", |config| {
            config.expected_return_delay_time = Some(256);
        }),
        ("nonfinite gripper force", |config| {
            config.gripper.as_mut().unwrap().force_fraction = f64::NAN;
        }),
        ("oversized gripper force", |config| {
            config.gripper.as_mut().unwrap().force_fraction = 1.01;
        }),
        ("zero gripper force", |config| {
            config.gripper.as_mut().unwrap().force_fraction = 0.0;
        }),
    ];
    for (case_name, invalidate) in invalid_configurations {
        let mut invalid = config();
        invalidate(&mut invalid);
        assert!(
            Servos::new(FakeBus::healthy(), invalid).is_err(),
            "{case_name}"
        );
    }
}

#[test]
fn malformed_joint_goals_are_refused_without_changing_motor_goals() {
    for positions in [
        vec![],
        vec![0.0],
        vec![0.0, 0.0, 0.0],
        vec![f64::NAN, 0.0],
        vec![0.0, f64::INFINITY],
        vec![0.0, 1e300],
    ] {
        let mut arm = servos(FakeBus::healthy());
        assert!(arm.command(&positions).is_err(), "{positions:?}");
        for motor_id in [1, 2] {
            assert_eq!(arm.bus.registers[&(motor_id, "goal_position".into())], 0);
        }
    }
}

#[test]
fn malformed_gripper_goals_are_refused_without_changing_the_motor_goal() {
    for angle in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e300] {
        let mut arm = servos(FakeBus::healthy());
        assert!(arm.command_gripper(angle).is_err(), "{angle}");
        assert_eq!(arm.bus.registers[&(3, "goal_position".into())], 0);
    }
}

#[test]
fn a_joint_angle_survives_the_trip_to_encoder_steps_and_back() {
    for sign in [1, -1] {
        let joint = motor(1, sign);
        for radians in [-1.9, -0.3, 0.0, 0.7, 1.8] {
            let back = steps_to_radians(radians_to_steps(radians, &joint), &joint);
            // One encoder step is 2 pi / 4096.
            assert!((back - radians).abs() < 2.0 * PI / 4096.0);
        }
    }
}

#[test]
fn the_calibration_pose_is_the_joint_offset_and_a_negative_sign_mirrors() {
    let positive = motor(1, 1);
    assert!((steps_to_radians(2047, &positive) - positive.offset_radians).abs() < 1e-12);
    let quarter_turn = 2047 + 1024;
    let forward = steps_to_radians(quarter_turn, &positive) - positive.offset_radians;
    let mirrored = steps_to_radians(quarter_turn, &motor(1, -1)) - positive.offset_radians;
    assert!((forward - PI / 2.0).abs() < 1e-3);
    assert!((forward + mirrored).abs() < 1e-12);
}

#[test]
fn a_healthy_arm_passes_preflight() {
    servos(FakeBus::healthy()).preflight().unwrap();
}

#[test]
fn recovery_observation_reads_motor_torque_and_fresh_feedback() {
    let mut bus = FakeBus::healthy();
    bus.set(1, "torque_enable", 1);
    bus.set(3, "torque_enable", 1);
    let observation = servos(bus).observe_recovery().unwrap();
    assert_eq!(observation.motor_torque, [(1, true), (2, false), (3, true)]);
    assert_eq!(observation.feedback.positions.len(), 2);
    assert_eq!(observation.feedback.temperatures_celsius, [30.0; 3]);
    assert_eq!(observation.feedback.voltages_volts, [12.0; 3]);
    assert!(observation.health_issue.is_none());
}

#[test]
fn recovery_observation_rejects_invalid_torque_and_flags_bad_voltage() {
    let mut invalid = FakeBus::healthy();
    invalid.set(2, "torque_enable", 2);
    assert!(servos(invalid).observe_recovery().is_err());
    let mut low_voltage = FakeBus::healthy();
    low_voltage.set(2, "present_voltage", 100);
    assert!(
        servos(low_voltage)
            .observe_recovery()
            .unwrap()
            .health_issue
            .is_some()
    );
}

#[test]
fn energized_recovery_holds_measured_positions_without_changing_torque() {
    let mut bus = FakeBus::healthy();
    for id in 1..=3 {
        bus.set(id, "torque_enable", 1);
    }
    let mut arm = servos(bus);
    arm.hold_recovery().unwrap();
    for id in 1..=3 {
        assert_eq!(arm.bus.registers[&(id, "goal_position".into())], 2047);
    }
    assert_eq!(arm.bus.torque_changes.len(), 0);

    let mut mixed = FakeBus::healthy();
    mixed.set(1, "torque_enable", 1);
    let mut arm = servos(mixed);
    assert!(arm.hold_recovery().is_err());
    assert_eq!(arm.bus.registers[&(1, "goal_position".into())], 0);
    assert_eq!(arm.bus.torque_changes.len(), 0);

    let mut mismatched = FakeBus::healthy();
    for id in 1..=3 {
        mismatched.set(id, "torque_enable", 1);
    }
    mismatched.goal_error = 1;
    let mut arm = servos(mismatched);
    assert!(arm.hold_recovery().is_err());
    assert_eq!(arm.bus.torque_changes.len(), 0);

    let mut moving = FakeBus::healthy();
    for id in 1..=3 {
        moving.set(id, "torque_enable", 1);
    }
    moving.drift_after_goal = Some((3, 100));
    let mut arm = servos(moving);
    assert!(arm.hold_recovery().is_err());
    assert_eq!(arm.bus.torque_changes.len(), 0);
}

#[test]
fn approved_recovery_enables_only_unpowered_motors_after_checks() {
    let mut bus = FakeBus::healthy();
    bus.set(1, "torque_enable", 1);
    bus.set(3, "torque_enable", 1);
    let mut arm = servos(bus);
    let observation = arm.enable_recovery(0.02).unwrap();
    assert!(observation.motor_torque.iter().all(|(_, enabled)| *enabled));
    assert_eq!(arm.bus.torque_changes, vec![(vec![2], true)]);

    let mut bus = FakeBus::healthy();
    bus.set(1, "torque_enable", 1);
    bus.set(2, "operating_mode", 1);
    let mut arm = servos(bus);
    assert!(arm.enable_recovery(0.02).is_err());
    assert_eq!(arm.bus.torque_changes.len(), 0);

    let mut bus = FakeBus::healthy();
    bus.set(1, "torque_enable", 1);
    bus.drift_after_goal = Some((2, 100));
    let mut arm = servos(bus);
    assert!(arm.enable_recovery(0.02).is_err());
    assert_eq!(arm.bus.torque_changes.len(), 0);
}

#[test]
fn preflight_refuses_what_is_not_as_the_profile_expects() {
    let cases: [(&str, u8, i64); 7] = [
        ("return_delay_time", 3, 250),
        ("torque_enable", 2, 1),
        ("homing_offset", 1, 99),
        ("max_position_limit", 3, 3000),
        ("operating_mode", 2, 1),
        ("d_coefficient", 1, 0),
        ("present_voltage", 2, 90),
    ];
    for (register, id, value) in cases {
        let mut bus = FakeBus::healthy();
        bus.set(id, register, value);
        assert!(servos(bus).preflight().is_err(), "{register} on {id}");
    }
}

#[test]
fn preflight_refuses_an_alarm() {
    let mut bus = FakeBus::healthy();
    bus.set(2, "status", 1 << 2);
    let message = servos(bus).preflight().unwrap_err().to_string();
    assert!(message.contains("temperature"), "{message}");
}

#[test]
fn enabling_holds_every_motor_at_its_present_position_then_turns_torque_on() {
    let mut bus = FakeBus::healthy();
    bus.set(1, "present_position", 2100);
    bus.set(3, "present_position", 1900);
    let mut arm = servos(bus);
    arm.enable().unwrap();
    assert_eq!(arm.bus.registers[&(1, "goal_position".into())], 2100);
    assert_eq!(arm.bus.registers[&(3, "goal_position".into())], 1900);
    assert_eq!(arm.bus.registers[&(3, "torque_limit".into())], 500);
    assert_eq!(arm.bus.torque_changes, vec![(vec![1, 2, 3], true)]);
}

#[test]
fn torque_does_not_go_on_when_a_hold_goal_did_not_take() {
    let mut bus = FakeBus::healthy();
    bus.goal_error = 1;
    let mut arm = servos(bus);
    assert!(arm.enable().is_err());
    assert_eq!(arm.bus.torque_changes.len(), 0);
}

#[test]
fn feedback_rejects_invalid_torque_and_checks_again_after_unverified_release() {
    let mut arm = servos(FakeBus::healthy());
    arm.enable().unwrap();
    arm.bus.set(1, "torque_enable", 2);
    for _ in 0..2 {
        let Err(error) = arm.read_feedback() else {
            panic!("Feedback accepted an invalid torque reading");
        };
        let error = error.to_string();
        assert!(error.contains("Invalid torque-enable value"), "{error}");
    }
    arm.bus.fail_reads = true;
    assert!(arm.release().is_err());
    arm.bus.fail_reads = false;
    let feedback = arm.read_feedback().unwrap();
    assert!(feedback.fault.unwrap().contains("torque off"));
    arm.release().unwrap();
    assert!(arm.read_feedback().unwrap().fault.is_none());
}

#[test]
fn feedback_converts_steps_loads_and_the_gripper() {
    let mut bus = FakeBus::healthy();
    bus.set(1, "present_position", 2047 + 512);
    bus.set(1, "present_load", -250);
    bus.set(2, "present_load", 400);
    bus.set(3, "present_position", 2047);
    let mut arm = servos(bus);
    let feedback = arm.read_feedback().unwrap();
    assert!((feedback.positions[0] - (PI / 4.0 + 0.1)).abs() < 1e-3);
    // Loads keep their direction, turned into the joint's frame by the motor's sign.
    assert!((feedback.efforts[0] + 25.0).abs() < 1e-9);
    assert!((feedback.efforts[1] + 40.0).abs() < 1e-9);
    assert!((feedback.gripper_angle.unwrap() - 0.1).abs() < 1e-9);
    assert!(feedback.fault.is_none());
}

#[test]
fn an_unset_return_delay_is_not_checked() {
    let mut bus = FakeBus::healthy();
    bus.set(3, "return_delay_time", 250);
    let mut unchecked = config();
    unchecked.expected_return_delay_time = None;
    Servos::new(bus, unchecked).unwrap().preflight().unwrap();
}

#[test]
fn the_state_block_decodes_signs_and_alarms() {
    let mut bytes = [0_u8; 10];
    bytes[0..2].copy_from_slice(&(0x8000_u16 | 0x0005).to_le_bytes());
    bytes[4..6].copy_from_slice(&(0x0400_u16 | 0x0007).to_le_bytes());
    bytes[6] = 121;
    bytes[7] = 41;
    bytes[9] = 0x20;
    assert_eq!(
        StateBlock::decode(&bytes).unwrap(),
        StateBlock {
            position: -5,
            load: -7,
            voltage: 121,
            temperature: 41,
            status: 0x20,
            packet_status: 0,
        }
    );
    assert!(StateBlock::decode(&bytes[..9]).is_err());
}

#[test]
fn an_alarm_or_overheating_is_reported_as_a_fault() {
    let mut bus = FakeBus::healthy();
    bus.set(1, "status", 1 << 3);
    bus.set(3, "present_temperature", 70);
    let feedback = servos(bus).read_feedback().unwrap();
    assert_eq!(feedback.temperatures_celsius, [30.0, 30.0, 70.0]);
    assert_eq!(feedback.voltages_volts, [12.0, 12.0, 12.0]);
    let fault = feedback.fault.unwrap();
    assert!(fault.contains("1: current"), "{fault}");
    assert!(fault.contains("3: over temperature 70C"), "{fault}");
}

#[test]
fn packet_alarm_is_reported_even_when_register_status_is_clear() {
    let mut bus = FakeBus::healthy();
    bus.set(2, "packet_status", 4);
    let fault = servos(bus).read_feedback().unwrap().fault.unwrap();
    assert!(fault.contains("2: temperature"), "{fault}");
}

#[test]
fn runtime_voltage_limits_apply_to_every_driven_motor() {
    for motor in [1, 2, 3] {
        let mut bus = FakeBus::healthy();
        bus.set(motor, "present_voltage", 90);
        let fault = servos(bus).read_feedback().unwrap().fault.unwrap();
        assert!(
            fault.contains(&format!("{motor}: supply voltage 9.0V")),
            "{fault}"
        );
    }
}

#[test]
fn marginal_temperature_and_voltage_need_three_consecutive_samples_per_motor() {
    let mut arm = servos(FakeBus::healthy());
    arm.bus.set(1, "present_temperature", 66);
    arm.bus.set(2, "present_voltage", 104);
    for _ in 0..2 {
        assert!(arm.read_feedback().unwrap().fault.is_none());
    }
    arm.bus.set(1, "present_temperature", 30);
    let fault = arm.read_feedback().unwrap().fault.unwrap();
    assert!(fault.contains("2: supply voltage 10.4V"), "{fault}");
    assert!(!fault.contains("1:"), "{fault}");
    arm.bus.set(2, "present_voltage", 120);
    arm.bus.set(1, "present_temperature", 66);
    assert!(arm.read_feedback().unwrap().fault.is_none());
    assert!(arm.read_feedback().unwrap().fault.is_none());
    assert!(
        arm.read_feedback()
            .unwrap()
            .fault
            .unwrap()
            .contains("1: over temperature")
    );
}

#[test]
fn severe_environmental_readings_and_servo_alarms_trip_immediately() {
    for (register, value) in [
        ("present_temperature", 105),
        ("present_voltage", 95),
        ("status", 4),
    ] {
        let mut bus = FakeBus::healthy();
        bus.set(2, register, value);
        assert!(
            servos(bus).read_feedback().unwrap().fault.is_some(),
            "{register}"
        );
    }
}

#[test]
fn severe_spike_is_confirmed_without_losing_the_original_reading() {
    let mut bus = FakeBus::healthy();
    bus.set(2, "present_temperature", 77);
    bus.confirmation_override = Some((2, 32, 120));
    let feedback = servos(bus).read_feedback().unwrap();
    assert!(feedback.fault.is_none());
    assert_eq!(feedback.temperatures_celsius[1], 77.0);
    assert_eq!(feedback.environment_confirmations.len(), 1);
    assert_eq!(
        feedback.environment_confirmations[0].temperature_celsius,
        32.0
    );
}

#[test]
fn severe_confirmed_reading_or_failed_confirmation_does_not_clear_a_fault() {
    let mut bus = FakeBus::healthy();
    bus.set(2, "present_temperature", 77);
    bus.confirmation_override = Some((2, 66, 120));
    assert!(
        servos(bus)
            .read_feedback()
            .unwrap()
            .fault
            .unwrap()
            .contains("2:")
    );
    let mut bus = FakeBus::healthy();
    bus.set(2, "present_temperature", 77);
    bus.fail_confirmation = true;
    assert!(servos(bus).read_feedback().is_err());
}

#[test]
fn severe_overheat_from_within_limits_flags_a_possible_corrupted_read() {
    let mut bus = FakeBus::healthy();
    bus.set(2, "present_temperature", 128);
    bus.confirmation_override = Some((2, 97, 118));
    let fault = servos(bus).read_feedback().unwrap().fault.unwrap();
    assert!(
        fault.contains("2: over temperature 97C (jumped from within limits to 128C"),
        "{fault}"
    );

    let mut arm = servos(FakeBus::healthy());
    arm.bus.set(1, "present_temperature", 66);
    arm.read_feedback().unwrap();
    arm.read_feedback().unwrap();
    let fault = arm.read_feedback().unwrap().fault.unwrap();
    assert!(
        fault.contains("1: over temperature") && !fault.contains("jumped"),
        "{fault}"
    );
}

#[test]
fn losing_torque_after_enabling_is_a_fault() {
    let mut arm = servos(FakeBus::healthy());
    arm.enable().unwrap();
    assert!(arm.read_feedback().unwrap().fault.is_none());
    arm.bus.set(2, "torque_enable", 0);
    std::thread::sleep(TORQUE_READ_INTERVAL);
    let fault = arm.read_feedback().unwrap().fault.unwrap();
    assert!(fault.contains("torque off"), "{fault}");
}

#[test]
fn a_command_writes_every_arm_goal_in_encoder_steps() {
    let mut arm = servos(FakeBus::healthy());
    arm.command(&[0.1, 0.1]).unwrap();
    // At the joint offset the goal is the calibration pose, for either sign.
    assert_eq!(arm.bus.registers[&(1, "goal_position".into())], 2047);
    assert_eq!(arm.bus.registers[&(2, "goal_position".into())], 2047);
}

#[test]
fn a_bus_that_stops_answering_is_a_plant_error() {
    let mut bus = FakeBus::healthy();
    bus.fail_reads = true;
    assert!(servos(bus).read_feedback().is_err());
}

#[test]
fn velocity_follows_motion_and_ignores_samples_too_close_together() {
    let mut estimator = VelocityEstimator::new(1);
    estimator.update(&[0.0], 0.0);
    let early = estimator.update(&[1.0], 0.005);
    assert!(early[0].abs() < 1e-12);
    let moving = estimator.update(&[0.1], 0.1);
    assert!(moving[0] > 0.0);
    let stale = estimator.update(&[5.0], 1.0);
    assert!(stale[0].abs() < 1e-12);
}

#[test]
fn a_bus_reply_with_too_few_values_is_an_error_not_a_panic() {
    let mut bus = FakeBus::healthy();
    bus.short_replies = true;
    let mut arm = servos(bus);
    assert!(arm.read_feedback().is_err());
    assert!(
        arm.preflight()
            .unwrap_err()
            .to_string()
            .contains("Cannot confirm torque off")
    );
    let mut healthy = servos(FakeBus::healthy());
    healthy.enable().unwrap();
    healthy.bus.short_replies = true;
    assert!(healthy.torque_enabled().is_err());
    assert!(healthy.read_feedback().is_err());
    assert!(
        healthy
            .release()
            .unwrap_err()
            .to_string()
            .contains("Cannot confirm torque off")
    );
}
