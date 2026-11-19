//! One arm's preflight, torque, feedback, position goals and verified recovery.

use std::time::{Duration, Instant};

use anyhow::{Context, bail, ensure};
use rustypot::servo::feetech::sts3215::DEFINITION;

use crate::calibration::STS3215_RESOLUTION;
use crate::config::{FeetechConfig, FeetechMotor, ValidatedFeetechConfig};
use crate::numeric;
use crate::telemetry::{EnvironmentConfirmation, Feedback, Reconnection};

use super::bus::{SerialBus, ServoBus, StateBlock};
use super::units::{VelocityEstimator, describe_alarms, radians_to_steps, steps_to_radians};

/// `Present_Load` counts 0.1 % of rated load.
const LOAD_UNITS_PER_PERCENT: f64 = 10.0;
/// A recovered motor must still be close to the position held after reconnecting.
const RECOVERY_HOLD_TOLERANCE_RADIANS: f64 = 0.02;
/// `Torque_Enable` is not in the state block, so it costs its own round trip: read
/// it every few cycles. A servo that trips on a fault reports it in the block's
/// status byte every cycle; this only catches a silent loss such as a power dip.
pub(super) const TORQUE_READ_INTERVAL: Duration = Duration::from_millis(100);

/// Three consecutive marginal readings at 50 Hz take about 60 ms.
const ENVIRONMENT_CONFIRMATION_SAMPLES: u8 = 3;
/// A reading this far beyond a configured limit is not treated as marginal.
const SEVERE_TEMPERATURE_MARGIN_CELSIUS: f64 = 5.0;
const SEVERE_VOLTAGE_MARGIN_VOLTS: f64 = 1.0;

fn checked_goal(radians: f64, motor: &FeetechMotor) -> anyhow::Result<i64> {
    ensure!(
        radians.is_finite(),
        "Motor {} goal must be finite",
        motor.id
    );
    let goal = radians_to_steps(radians, motor);
    ensure!(
        (0..STS3215_RESOLUTION).contains(&goal),
        "Motor {} goal exceeds the encoder range",
        motor.id
    );
    Ok(goal)
}

#[derive(Default)]
struct EnvironmentReadings {
    high_temperature: u8,
    low_voltage: u8,
    high_voltage: u8,
}

fn confirmed_excursion(counter: &mut u8, outside: bool, severe: bool) -> bool {
    if !outside {
        *counter = 0;
        return false;
    }
    *counter = counter.saturating_add(1);
    severe || *counter >= ENVIRONMENT_CONFIRMATION_SAMPLES
}

/// Flags an overheat confirmed only by one immediate re-read after a normal cycle.
fn sudden_overheat_note(sudden: bool, initial_temperature: f64) -> String {
    if !sudden {
        return String::new();
    }
    format!(
        " (jumped from within limits to {initial_temperature:.0}C in one cycle and a single \
         immediate re-read confirmed it; this can be two corrupted bus reads, so check the next \
         cycles' temperature in the trace)"
    )
}

pub struct Servos<B: ServoBus> {
    pub(super) bus: B,
    config: ValidatedFeetechConfig,
    position_mode: i64,
    arm_ids: Vec<u8>,
    driven_ids: Vec<u8>,
    all_ids: Vec<u8>,
    last_torque_read: Option<Instant>,
    velocity_estimator: VelocityEstimator,
    environment_readings: Vec<EnvironmentReadings>,
    /// The goals of the last command, kept so the next one reuses the buffer.
    goal_steps: Vec<i64>,
    started: Instant,
    /// Set once torque is on: a servo that loses it mid-motion is a fault.
    expect_torque: bool,
}

impl Servos<SerialBus> {
    /// # Errors
    /// The configuration is invalid or the port cannot be opened.
    pub fn open(config: FeetechConfig) -> anyhow::Result<Self> {
        let config = ValidatedFeetechConfig::try_from(config)?;
        let mut ids: Vec<u8> = config.joints.iter().map(|motor| motor.id).collect();
        ids.extend(config.gripper.iter().map(|gripper| gripper.motor.id));
        ids.extend(&config.passive_motor_ids);
        let bus = SerialBus::open(&config.port, &ids)?;
        Self::from_validated(bus, config)
    }
}

impl<B: ServoBus> Servos<B> {
    /// # Errors
    /// The configuration is invalid or the servo definition has no position mode.
    pub fn new(bus: B, config: FeetechConfig) -> anyhow::Result<Self> {
        Self::from_validated(bus, ValidatedFeetechConfig::try_from(config)?)
    }

    fn from_validated(bus: B, config: ValidatedFeetechConfig) -> anyhow::Result<Self> {
        let position_mode = DEFINITION
            .info
            .operating_modes
            .iter()
            .find(|(name, _)| *name == "position")
            .map(|(_, value)| i64::from(*value))
            .context("The STS3215 definition has no position mode")?;
        let arm_ids: Vec<u8> = config.joints.iter().map(|motor| motor.id).collect();
        let mut driven_ids = arm_ids.clone();
        driven_ids.extend(config.gripper.iter().map(|gripper| gripper.motor.id));
        let mut all_ids = driven_ids.clone();
        all_ids.extend(&config.passive_motor_ids);
        Ok(Self {
            bus,
            config,
            position_mode,
            velocity_estimator: VelocityEstimator::new(arm_ids.len()),
            environment_readings: (0..driven_ids.len())
                .map(|_| EnvironmentReadings::default())
                .collect(),
            goal_steps: Vec::with_capacity(arm_ids.len()),
            arm_ids,
            driven_ids,
            all_ids,
            last_torque_read: None,
            started: Instant::now(),
            expect_torque: false,
        })
    }

    /// The arm's motors, then the gripper's.
    fn driven_motors(&self) -> impl Iterator<Item = &FeetechMotor> {
        self.config
            .joints
            .iter()
            .chain(self.config.gripper.iter().map(|gripper| &gripper.motor))
    }

    #[must_use]
    pub fn bus(&self) -> &B {
        &self.bus
    }

    fn read_driven(&mut self, register: &str) -> anyhow::Result<Vec<i64>> {
        let values = self.bus.read(&self.driven_ids, register)?;
        ensure!(
            values.len() == self.driven_ids.len(),
            "The bus answered {} values for {register}, expected {}",
            values.len(),
            self.driven_ids.len()
        );
        Ok(values)
    }

    /// Read tuning registers of every driven motor, for inspection only: nothing is
    /// written. A register the bus cannot read is reported as an error, not fatal.
    pub fn dump_registers(
        &mut self,
        registers: &[&str],
    ) -> Vec<(String, Result<Vec<i64>, String>)> {
        registers
            .iter()
            .map(|register| {
                (
                    (*register).to_owned(),
                    self.read_driven(register)
                        .map_err(|error| format!("{error:#}")),
                )
            })
            .collect()
    }

    fn check_motor_configuration(&mut self) -> anyhow::Result<()> {
        let motors: Vec<FeetechMotor> = self.driven_motors().cloned().collect();
        let expected_calibration = [
            (
                "homing_offset",
                motors
                    .iter()
                    .map(|m| m.calibration.homing_offset)
                    .collect::<Vec<_>>(),
            ),
            (
                "min_position_limit",
                motors.iter().map(|m| m.calibration.range_min).collect(),
            ),
            (
                "max_position_limit",
                motors.iter().map(|m| m.calibration.range_max).collect(),
            ),
        ];
        for (register, expected) in expected_calibration {
            if self.read_driven(register)? != expected {
                bail!("Motor {register} differs from the calibration file");
            }
        }
        let modes = self.read_driven("operating_mode")?;
        if modes.iter().any(|mode| *mode != self.position_mode) {
            bail!("Every driven SO-101 motor must be in position mode");
        }
        if let Some(expected) = self.config.expected_return_delay_time {
            let delays = self.read_driven("return_delay_time")?;
            if delays.iter().any(|delay| *delay != expected) {
                bail!(
                    "Return_Delay_Time is {delays:?}, expected {expected} on every driven motor; \
                     each unit adds 2 us per motor to every read"
                );
            }
        }
        let proportional = self.read_driven("p_coefficient")?;
        let integral = self.read_driven("i_coefficient")?;
        let derivative = self.read_driven("d_coefficient")?;
        for (index, motor) in motors.iter().enumerate() {
            let gains = &motor.expected_gains;
            if [proportional[index], integral[index], derivative[index]]
                != [gains.p, gains.i, gains.d]
            {
                bail!(
                    "Motor {} has unexpected position gains {:?}",
                    motor.id,
                    [proportional[index], integral[index], derivative[index]]
                );
            }
        }
        let voltages: Vec<f64> = self
            .read_driven("present_voltage")?
            .iter()
            .map(|tenths| numeric::register_value(*tenths) / 10.0)
            .collect();
        if voltages.iter().any(|volts| {
            !(self.config.min_voltage_volts..=self.config.max_voltage_volts).contains(volts)
        }) {
            bail!("Supply voltage out of range: {voltages:?}");
        }
        let feedback = self.read_state()?;
        if let Some(fault) = feedback.fault {
            bail!("Motor alarm before motion: {fault}");
        }
        Ok(())
    }

    /// The ids of the driven motors, the arm's joints in order then the gripper.
    #[must_use]
    pub fn driven_ids(&self) -> &[u8] {
        &self.driven_ids
    }

    /// Whether torque is on, for each driven motor.
    ///
    /// # Errors
    /// The bus did not answer.
    pub fn torque_enabled(&mut self) -> anyhow::Result<Vec<bool>> {
        self.read_driven("torque_enable")?
            .into_iter()
            .zip(&self.driven_ids)
            .map(|(torque, motor_id)| match torque {
                0 => Ok(false),
                1 => Ok(true),
                _ => anyhow::bail!("Motor {motor_id}: Invalid torque-enable value: {torque}"),
            })
            .collect()
    }

    fn read_state(&mut self) -> anyhow::Result<Feedback> {
        let mut feedback = Feedback::default();
        self.read_state_into(&mut feedback)?;
        Ok(feedback)
    }

    /// Check alarms and confirm severe environmental outliers without advancing the motion.
    fn read_environment_faults(
        &mut self,
        blocks: &[StateBlock],
        torque: Option<&[bool]>,
        feedback: &mut Feedback,
    ) -> anyhow::Result<Vec<String>> {
        let mut faults = Vec::new();
        feedback.temperatures_celsius.clear();
        feedback.voltages_volts.clear();
        feedback.environment_confirmations.clear();
        // A servo alarm is immediate. Otherwise re-read severe environmental
        // excursions before issuing a new setpoint. The previous setpoint remains
        // active during this bounded read; no trajectory time advances here.
        let any_alarm = blocks
            .iter()
            .any(|block| block.status != 0 || block.packet_status != 0);
        for (index, motor_id) in self.driven_ids.iter().copied().enumerate() {
            let initial = blocks[index];
            let initial_temperature = numeric::register_value(initial.temperature);
            let initial_voltage = numeric::register_value(initial.voltage) / 10.0;
            feedback.temperatures_celsius.push(initial_temperature);
            feedback.voltages_volts.push(initial_voltage);
            let severe_temperature = initial_temperature
                >= self.config.max_temperature_celsius + SEVERE_TEMPERATURE_MARGIN_CELSIUS;
            let severe_low_voltage =
                initial_voltage <= self.config.min_voltage_volts - SEVERE_VOLTAGE_MARGIN_VOLTS;
            let severe_high_voltage =
                initial_voltage >= self.config.max_voltage_volts + SEVERE_VOLTAGE_MARGIN_VOLTS;
            let followup = if !any_alarm
                && (severe_temperature || severe_low_voltage || severe_high_voltage)
            {
                let confirmation = self.bus.read_state_block(&[motor_id])?;
                ensure!(
                    confirmation.len() == 1,
                    "Motor {motor_id}: confirmation returned {} state blocks",
                    confirmation.len()
                );
                let reading = confirmation[0];
                feedback
                    .environment_confirmations
                    .push(EnvironmentConfirmation {
                        motor_id,
                        temperature_celsius: numeric::register_value(reading.temperature),
                        voltage_volts: numeric::register_value(reading.voltage) / 10.0,
                        status: reading.status,
                        packet_status: reading.packet_status,
                    });
                Some(reading)
            } else {
                None
            };
            let evaluated = followup.unwrap_or(initial);
            let alarm = initial.status
                | i64::from(initial.packet_status)
                | evaluated.status
                | i64::from(evaluated.packet_status);
            let temperature = numeric::register_value(evaluated.temperature);
            let voltage = numeric::register_value(evaluated.voltage) / 10.0;
            let readings = &mut self.environment_readings[index];
            let overheated = confirmed_excursion(
                &mut readings.high_temperature,
                temperature > self.config.max_temperature_celsius,
                severe_temperature && temperature > self.config.max_temperature_celsius,
            );
            // Only the severe reading and one re-read agreed; two corrupted reads look the same.
            let sudden_overheat = overheated && readings.high_temperature == 1;
            let low_voltage = confirmed_excursion(
                &mut readings.low_voltage,
                voltage < self.config.min_voltage_volts,
                severe_low_voltage && voltage < self.config.min_voltage_volts,
            );
            let high_voltage = confirmed_excursion(
                &mut readings.high_voltage,
                voltage > self.config.max_voltage_volts,
                severe_high_voltage && voltage > self.config.max_voltage_volts,
            );
            if low_voltage || high_voltage {
                faults.push(format!(
                    "{motor_id}: supply voltage {voltage:.1}V outside configured limits"
                ));
            }
            let lost_torque = torque.is_some_and(|torque| !torque[index]);
            if alarm != 0 || overheated || lost_torque {
                faults.push(format!(
                    "{}: {}{}{} {temperature:.0}C{}",
                    motor_id,
                    if alarm == 0 && overheated {
                        "over temperature".to_owned()
                    } else {
                        describe_alarms(alarm)
                    },
                    if lost_torque { " torque off" } else { "" },
                    if overheated && alarm != 0 {
                        " over temperature"
                    } else {
                        ""
                    },
                    sudden_overheat_note(sudden_overheat, initial_temperature),
                ));
            }
        }
        Ok(faults)
    }

    fn read_state_into(&mut self, feedback: &mut Feedback) -> anyhow::Result<()> {
        // Torque is outside the live block, so it is checked every few cycles;
        // the block comes last so its position is the freshest measurement.
        let torque = if self.expect_torque
            && self
                .last_torque_read
                .is_none_or(|read| read.elapsed() >= TORQUE_READ_INTERVAL)
        {
            let torque = self.torque_enabled()?;
            self.last_torque_read = Some(Instant::now());
            Some(torque)
        } else {
            None
        };
        let blocks = self.bus.read_state_block(&self.driven_ids)?;
        ensure!(
            blocks.len() == self.driven_ids.len(),
            "The bus answered {} state blocks, expected {}",
            blocks.len(),
            self.driven_ids.len()
        );
        let sampled_at = self.started.elapsed().as_secs_f64();

        let faults = self.read_environment_faults(&blocks, torque.as_deref(), feedback)?;
        let joint_count = self.arm_ids.len();
        feedback.positions.clear();
        feedback.positions.extend(
            self.config
                .joints
                .iter()
                .zip(&blocks)
                .map(|(motor, block)| steps_to_radians(block.position, motor)),
        );
        feedback.gripper_angle = self
            .config
            .gripper
            .as_ref()
            .map(|gripper| steps_to_radians(blocks[joint_count].position, &gripper.motor));
        feedback.efforts.clear();
        feedback.efforts.extend(
            self.config
                .joints
                .iter()
                .zip(&blocks)
                .map(|(motor, block)| {
                    f64::from(motor.sign) * numeric::register_value(block.load)
                        / LOAD_UNITS_PER_PERCENT
                }),
        );
        let velocities = self
            .velocity_estimator
            .update(&feedback.positions, sampled_at);
        feedback.velocities.clear();
        feedback.velocities.extend_from_slice(velocities);
        feedback.fault = (!faults.is_empty()).then(|| faults.join(", "));
        Ok(())
    }
}

impl<B: ServoBus> Servos<B> {
    /// Verify configuration, health and pose stability, then enable unpowered motors.
    ///
    /// # Errors
    /// A required hardware check, read or write failed.
    pub fn enable_recovery(
        &mut self,
        drift_limit_radians: f64,
    ) -> anyhow::Result<crate::telemetry::RecoveryObservation> {
        ensure!(
            drift_limit_radians.is_finite() && drift_limit_radians >= 0.0,
            "Invalid recovery drift limit"
        );
        let initial = self.observe_recovery()?;
        ensure!(
            initial.health_issue.is_none(),
            "Recovery health check failed: {:?}",
            initial.health_issue
        );
        let initial_positions = self.read_driven("present_position")?;
        self.check_motor_configuration()?;
        let measured = self.read_driven("present_position")?;
        for ((motor, before), after) in self.driven_motors().zip(&initial_positions).zip(&measured)
        {
            ensure!(
                (steps_to_radians(*before, motor) - steps_to_radians(*after, motor)).abs()
                    <= drift_limit_radians,
                "Motor {} moved during recovery checks",
                motor.id
            );
        }
        let torque = self.torque_enabled()?;
        ensure!(
            torque
                .iter()
                .copied()
                .eq(initial.motor_torque.iter().map(|(_, enabled)| *enabled)),
            "Motor torque changed during recovery checks"
        );
        if let Some(gripper) = &self.config.gripper {
            let limit = numeric::rounded(gripper.force_fraction * 1000.0);
            self.bus
                .write(&[gripper.motor.id], "torque_limit", &[limit])?;
        }
        self.bus
            .write(&self.driven_ids, "goal_position", &measured)?;
        ensure!(
            self.read_driven("goal_position")? == measured,
            "Recovery hold goals were not accepted"
        );
        let before_enable = self.read_driven("present_position")?;
        for ((motor, goal), position) in self.driven_motors().zip(&measured).zip(&before_enable) {
            ensure!(
                (steps_to_radians(*goal, motor) - steps_to_radians(*position, motor)).abs()
                    <= drift_limit_radians,
                "Motor {} moved before recovery enable",
                motor.id
            );
        }
        let unpowered = self
            .driven_ids
            .iter()
            .zip(torque)
            .filter_map(|(id, enabled)| (!enabled).then_some(*id))
            .collect::<Vec<_>>();
        if !unpowered.is_empty() {
            self.bus.set_torque(&unpowered, true)?;
        }
        self.hold_recovery()
    }

    /// Read torque state and fresh health evidence without commanding motion.
    ///
    /// # Errors
    /// A required hardware check, read or write failed.
    pub fn observe_recovery(&mut self) -> anyhow::Result<crate::telemetry::RecoveryObservation> {
        let motor_torque = self
            .driven_ids
            .clone()
            .into_iter()
            .zip(self.torque_enabled()?)
            .collect();
        let feedback = self.read_state()?;
        let health_issue = feedback.fault.clone().or_else(|| {
            if feedback
                .temperatures_celsius
                .iter()
                .any(|temperature| *temperature > self.config.max_temperature_celsius)
            {
                Some("Motor temperature exceeds its configured limit".into())
            } else if feedback.voltages_volts.iter().any(|voltage| {
                !(self.config.min_voltage_volts..=self.config.max_voltage_volts).contains(voltage)
            }) {
                Some("Motor voltage is outside its configured range".into())
            } else {
                None
            }
        });
        Ok(crate::telemetry::RecoveryObservation {
            feedback,
            motor_torque,
            health_issue,
        })
    }
    /// Refresh and verify an already energized hold at the measured pose.
    ///
    /// # Errors
    /// A required hardware check, read or write failed.
    pub fn hold_recovery(&mut self) -> anyhow::Result<crate::telemetry::RecoveryObservation> {
        let observation = self.observe_recovery()?;
        ensure!(
            observation.motor_torque.iter().all(|(_, enabled)| *enabled),
            "Not every driven motor has torque"
        );
        ensure!(
            observation.health_issue.is_none(),
            "Recovery health check failed: {:?}",
            observation.health_issue
        );
        let measured = self.read_driven("present_position")?;
        self.bus
            .write(&self.driven_ids, "goal_position", &measured)?;
        ensure!(
            self.read_driven("goal_position")? == measured,
            "Recovery hold goals were not accepted"
        );
        let confirmed = self.observe_recovery()?;
        ensure!(
            confirmed.motor_torque.iter().all(|(_, enabled)| *enabled),
            "Motor torque changed during recovery"
        );
        ensure!(
            confirmed.health_issue.is_none(),
            "Recovery health check failed: {:?}",
            confirmed.health_issue
        );
        let confirmed_positions = self.read_driven("present_position")?;
        for ((motor, goal), position) in self
            .driven_motors()
            .zip(&measured)
            .zip(&confirmed_positions)
        {
            let error = (steps_to_radians(*goal, motor) - steps_to_radians(*position, motor)).abs();
            ensure!(
                error <= RECOVERY_HOLD_TOLERANCE_RADIANS,
                "Motor {} moved away from its recovery hold by {error:.3} rad",
                motor.id
            );
        }
        self.expect_torque = true;
        self.last_torque_read = None;
        Ok(confirmed)
    }
    /// Read fresh hardware feedback into new buffers.
    ///
    /// # Errors
    /// The bus does not answer or its reply cannot be decoded.
    pub fn read_feedback(&mut self) -> anyhow::Result<Feedback> {
        self.read_state()
    }

    /// Replace feedback while reusing its buffers.
    ///
    /// # Errors
    /// The bus does not answer or its reply cannot be decoded.
    pub fn read_feedback_into(&mut self, feedback: &mut Feedback) -> anyhow::Result<()> {
        self.read_state_into(feedback)
    }

    /// Send joint position goals. The caller owns motion limits and timing.
    ///
    /// # Errors
    /// A required hardware check, read or write failed.
    pub fn command(&mut self, positions: &[f64]) -> anyhow::Result<()> {
        // Position mode has no per-command speed cap. The caller must bound
        // speed, acceleration and jerk before sending goals.
        ensure!(
            positions.len() == self.arm_ids.len(),
            "Expected {} joint goals, got {}",
            self.arm_ids.len(),
            positions.len()
        );
        self.goal_steps.clear();
        for (radians, motor) in positions.iter().zip(&self.config.joints) {
            self.goal_steps.push(checked_goal(*radians, motor)?);
        }
        self.bus
            .write(&self.arm_ids, "goal_position", &self.goal_steps)
    }

    /// Send a gripper position goal in model radians.
    ///
    /// # Errors
    /// A required hardware check, read or write failed.
    pub fn command_gripper(&mut self, angle: f64) -> anyhow::Result<()> {
        let gripper = self
            .config
            .gripper
            .as_ref()
            .context("This arm has no gripper")?;
        let goal = checked_goal(angle, &gripper.motor)?;
        self.bus
            .write(&[gripper.motor.id], "goal_position", &[goal])
    }

    /// Check the servos before any torque: torque off, the arm's calibration and
    /// position-loop gains on the servos as the profile expects, position mode,
    /// supply voltage, and no alarm.
    ///
    /// # Errors
    /// The bus failed, or a servo is not as expected.
    pub fn preflight(&mut self) -> anyhow::Result<()> {
        let torque = self.bus.read(&self.all_ids, "torque_enable")?;
        ensure!(
            torque.len() == self.all_ids.len(),
            "Cannot confirm torque off: {} motors answered, expected {}",
            torque.len(),
            self.all_ids.len()
        );
        if torque.iter().any(|torque| *torque != 0) {
            bail!("All SO-101 motors must start with torque off");
        }
        self.check_motor_configuration()
    }

    /// Hold every driven motor where it is, then turn torque on. The gripper's
    /// grip force is capped in RAM (not EEPROM) first.
    ///
    /// # Errors
    /// The bus failed, or a servo's goal did not take.
    pub fn enable(&mut self) -> anyhow::Result<()> {
        if let Some(gripper) = &self.config.gripper {
            let id = [gripper.motor.id];
            let present = self.bus.read(&id, "present_position")?;
            self.bus.write(&id, "goal_position", &present)?;
            let limit = numeric::rounded(gripper.force_fraction * 1000.0);
            self.bus.write(&id, "torque_limit", &[limit])?;
        }
        // Capture and verify raw goals immediately before torque-on, so the
        // startup hold does not rest on an earlier floating-point command.
        let present = self.read_driven("present_position")?;
        self.bus
            .write(&self.driven_ids, "goal_position", &present)?;
        let goals = self.read_driven("goal_position")?;
        if goals != present {
            bail!(
                "Startup hold goals differ from measured positions: \
                 measured={present:?}, goals={goals:?}"
            );
        }
        self.bus.set_torque(&self.driven_ids, true)?;
        self.expect_torque = true;
        self.last_torque_read = None;
        Ok(())
    }

    /// Reopen the bus as at startup and hear every motor. Nothing is written:
    /// startup writes nothing either, and the registers a power cycle resets
    /// (torque, goals, the gripper's torque limit) are the ones `enable` writes.
    pub fn reconnect(&mut self) -> Reconnection {
        let reconnection = self.bus.reconnect();
        if matches!(reconnection, Reconnection::Restored) {
            self.expect_torque = false;
            self.last_torque_read = None;
            self.velocity_estimator = VelocityEstimator::new(self.arm_ids.len());
            self.environment_readings
                .fill_with(EnvironmentReadings::default);
        }
        reconnection
    }

    /// Turn torque off, which drops the arm.
    ///
    /// # Errors
    /// A servo did not take it.
    pub fn release(&mut self) -> anyhow::Result<()> {
        self.bus.set_torque(&self.all_ids, false)?;
        let torque = self.bus.read(&self.all_ids, "torque_enable")?;
        ensure!(
            torque.len() == self.all_ids.len() && torque.iter().all(|value| *value == 0),
            "Cannot confirm torque off for every motor; the arm may still be powered"
        );
        self.expect_torque = false;
        self.last_torque_read = None;
        Ok(())
    }
}
