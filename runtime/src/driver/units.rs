//! Encoder units, alarms and the velocity estimate.

use std::f64::consts::TAU;

pub(super) use crate::calibration::HOMING_STEPS;
use crate::calibration::{STS3215_RESOLUTION, encoder_steps_to_radians};
use crate::config::FeetechMotor;
use crate::numeric;

/// Velocity estimates keep a short memory.
pub(super) const VELOCITY_TIME_CONSTANT_SECONDS: f64 = 0.04;

/// A motor's joint angle for an encoder reading.
#[must_use]
pub fn steps_to_radians(steps: i64, motor: &FeetechMotor) -> f64 {
    encoder_steps_to_radians(steps, motor.sign, motor.offset_radians)
}

/// The encoder goal for a joint angle, rounding halves away from zero; a half
/// step is far below the encoder's accuracy.
#[must_use]
pub fn radians_to_steps(radians: f64, motor: &FeetechMotor) -> i64 {
    numeric::rounded(
        f64::from(motor.sign)
            * (radians - motor.offset_radians)
            * numeric::register_value(STS3215_RESOLUTION)
            / TAU
            + HOMING_STEPS,
    )
}

/// Feetech status bits, from Feetech's STS memory table.
pub(super) const ALARMS: [(i64, &str); 5] = [
    (1 << 0, "voltage"),
    (1 << 1, "encoder"),
    (1 << 2, "temperature"),
    (1 << 3, "current"),
    (1 << 5, "load"),
];

pub(super) fn describe_alarms(status: i64) -> String {
    let known: i64 = ALARMS.iter().map(|(bit, _)| bit).sum();
    let mut names: Vec<String> = ALARMS
        .iter()
        .filter(|(bit, _)| status & bit != 0)
        .map(|(_, name)| (*name).to_owned())
        .collect();
    if status & !known != 0 {
        names.push(format!("unknown {:#04x}", status & !known));
    }
    if names.is_empty() {
        "ok".to_owned()
    } else {
        names.join("+")
    }
}

/// Encoder velocity from timestamped samples with a light low-pass filter.
pub(super) struct VelocityEstimator {
    previous: Option<(Vec<f64>, f64)>,
    velocities: Vec<f64>,
}

impl VelocityEstimator {
    pub(super) fn new(joint_count: usize) -> Self {
        Self {
            previous: None,
            velocities: vec![0.0; joint_count],
        }
    }

    /// The estimate after this sample.
    pub(super) fn update(&mut self, positions: &[f64], seconds: f64) -> &[f64] {
        if let Some((previous, previous_seconds)) = &self.previous {
            let elapsed = seconds - previous_seconds;
            if elapsed > 0.0 && elapsed < 0.01 {
                // Too soon after the last sample to say anything new.
                return &self.velocities;
            }
            if elapsed > 0.0 && elapsed <= 0.2 {
                let alpha = elapsed / (VELOCITY_TIME_CONSTANT_SECONDS + elapsed);
                for ((velocity, now), before) in
                    self.velocities.iter_mut().zip(positions).zip(previous)
                {
                    *velocity += alpha * ((now - before) / elapsed - *velocity);
                }
            } else {
                self.velocities.fill(0.0);
            }
        }
        match &mut self.previous {
            Some((previous, previous_seconds)) => {
                previous.clear();
                previous.extend_from_slice(positions);
                *previous_seconds = seconds;
            }
            None => self.previous = Some((positions.to_vec(), seconds)),
        }
        &self.velocities
    }
}
