use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use serde::Serialize;

use crate::calibration::{LerobotMotorCalibration, STS3215_RESOLUTION};
use crate::numeric::register_value;
use crate::settings::Sign;
use crate::setup::{Alignment, GripperEndpoints, MotorIds};

use super::sweep::{ENCODER_MIDPOINT, rehomed_position};
use super::{EncoderSweep, HOME_GRIPPER, HOME_JOINTS, MOTOR_NAMES, joint_radians};

pub type CalibrationEntries = BTreeMap<String, LerobotMotorCalibration>;
pub const GRIPPER: usize = 5;

const MINIMUM_TRAVEL_DEGREES: [i64; 6] = [130, 130, 120, 130, 220, 75];

#[derive(Debug, Clone, Copy)]
pub struct DraftJoint {
    pub homing_offset: i64,
    pub range_min: i64,
    pub range_max: i64,
    pub sign: Sign,
    pub offset: f64,
    reference_steps: i64,
}

impl DraftJoint {
    pub fn position(self, raw_position: i64, prior_homing: i64) -> f64 {
        joint_radians(
            rehomed_position(raw_position, prior_homing, self.homing_offset),
            self.sign,
            self.offset,
        )
    }

    fn angle(self, steps: i64) -> f64 {
        joint_radians(steps, self.sign, self.offset)
    }
}

#[derive(Debug, Clone)]
pub struct Draft {
    pub joints: [DraftJoint; 6],
    pub gripper: GripperEndpoints,
}

impl Draft {
    pub fn from_saved(
        entries: &CalibrationEntries,
        alignment: &Alignment,
        gripper: GripperEndpoints,
    ) -> Result<Self> {
        let (signs, offsets) = alignment_arrays(alignment);
        let mut joints = Vec::with_capacity(6);
        for (index, name) in MOTOR_NAMES.into_iter().enumerate() {
            let entry = entries
                .get(name)
                .ok_or_else(|| anyhow::anyhow!("Missing calibration for {name}"))?;
            joints.push(DraftJoint {
                homing_offset: entry.homing_offset,
                range_min: entry.range_min,
                range_max: entry.range_max,
                sign: signs[index],
                offset: offsets[index],
                reference_steps: ENCODER_MIDPOINT,
            });
        }
        Ok(Self {
            joints: joints
                .try_into()
                .map_err(|_| anyhow::anyhow!("Expected six calibration joints"))?,
            gripper,
        })
    }

    pub fn estimate(
        sweep: &Sweep,
        previous_homing: [i64; 6],
        baseline: Option<Self>,
    ) -> Result<Self> {
        let incomplete: Vec<_> = sweep
            .active()
            .filter_map(|(index, sample)| {
                let steps = sample.maximum - sample.minimum;
                let expected = MINIMUM_TRAVEL_DEGREES[index];
                (steps * 360 < expected * STS3215_RESOLUTION).then(|| {
                    let travel = register_value(steps) * 360.0 / register_value(STS3215_RESOLUTION);
                    format!(
                        "{}: {travel:.0}° recorded (at least {expected}° expected)",
                        MOTOR_NAMES[index]
                    )
                })
            })
            .collect();
        ensure!(
            incomplete.is_empty(),
            "Incomplete sweep: {}. Move the listed joints gently and finish again.",
            incomplete.join(", ")
        );

        let mut draft = baseline.unwrap_or(Self {
            joints: [DraftJoint {
                homing_offset: 0,
                range_min: 0,
                range_max: 0,
                sign: Sign::Positive,
                offset: 0.0,
                reference_steps: ENCODER_MIDPOINT,
            }; 6],
            gripper: GripperEndpoints::default(),
        });
        let [pan, lift, elbow, flex, roll] = HOME_JOINTS;
        let home_angles = [pan, lift, elbow, flex, roll, HOME_GRIPPER];
        for (index, sample) in sweep.active() {
            let sign = sweep.signs[index];
            let estimate = sample.estimate(previous_homing[index], sign, home_angles[index])?;
            draft.joints[index] = DraftJoint {
                homing_offset: estimate.homing_offset,
                range_min: estimate.range_min,
                range_max: estimate.range_max,
                sign,
                offset: estimate.model_offset_radians,
                reference_steps: estimate.reference_steps,
            };
        }
        if sweep.samples[GRIPPER].is_some() {
            let gripper = draft.joints[GRIPPER];
            let minimum = gripper.angle(gripper.range_min);
            let maximum = gripper.angle(gripper.range_max);
            let (closed, open) = if (gripper.reference_steps - gripper.range_min).abs()
                <= (gripper.range_max - gripper.reference_steps).abs()
            {
                (minimum, maximum)
            } else {
                (maximum, minimum)
            };
            draft.gripper = GripperEndpoints {
                closed_position_radians: closed,
                open_position_radians: open,
            };
        }
        Ok(draft)
    }

    #[must_use]
    pub fn entries(&self, motor_ids: MotorIds) -> CalibrationEntries {
        MOTOR_NAMES
            .into_iter()
            .zip(motor_ids.ordered())
            .zip(&self.joints)
            .map(|((name, id), joint)| {
                (
                    name.to_owned(),
                    LerobotMotorCalibration {
                        id,
                        drive_mode: 0,
                        homing_offset: joint.homing_offset,
                        range_min: joint.range_min,
                        range_max: joint.range_max,
                    },
                )
            })
            .collect()
    }

    #[must_use]
    pub fn alignment(&self) -> Alignment {
        Alignment {
            joint_signs: std::array::from_fn(|index| self.joints[index].sign),
            joint_offsets_radians: std::array::from_fn(|index| self.joints[index].offset),
            gripper_sign: self.joints[GRIPPER].sign,
            gripper_offset_radians: self.joints[GRIPPER].offset,
        }
    }

    pub fn reverse(&mut self, index: usize) -> Result<()> {
        let joint = self
            .joints
            .get_mut(index)
            .ok_or_else(|| anyhow::anyhow!("Unknown joint"))?;
        let reference = joint.angle(joint.reference_steps);
        joint.sign = match joint.sign {
            Sign::Positive => Sign::Negative,
            Sign::Negative => Sign::Positive,
        };
        joint.offset = reference - joint_radians(joint.reference_steps, joint.sign, 0.0);
        if index == GRIPPER {
            self.gripper.closed_position_radians =
                2.0 * reference - self.gripper.closed_position_radians;
            self.gripper.open_position_radians =
                2.0 * reference - self.gripper.open_position_radians;
        }
        Ok(())
    }

    pub fn adjust(&mut self, index: usize, delta: f64) -> Result<()> {
        ensure!(
            (-0.1..=0.1).contains(&delta),
            "Use a finite adjustment between -0.1 and 0.1 radians"
        );
        let joint = self
            .joints
            .get_mut(index)
            .ok_or_else(|| anyhow::anyhow!("Unknown joint"))?;
        joint.offset += delta;
        if index == GRIPPER {
            self.gripper.closed_position_radians += delta;
            self.gripper.open_position_radians += delta;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Sweep {
    samples: [Option<EncoderSweep>; 6],
    signs: [Sign; 6],
}

impl Sweep {
    pub fn start(positions: [i64; 6], alignment: &Alignment, joint: Option<usize>) -> Result<Self> {
        ensure!(joint.is_none_or(|index| index < 6), "Unknown joint");
        let mut samples = [None; 6];
        for (index, position) in positions.into_iter().enumerate() {
            if joint.is_none_or(|selected| selected == index) {
                samples[index] = Some(EncoderSweep::start(position)?);
            }
        }
        Ok(Self {
            samples,
            signs: alignment_arrays(alignment).0,
        })
    }

    pub fn record(&mut self, positions: [i64; 6]) -> Result<()> {
        let mut updated = self.samples;
        for (sample, position) in updated.iter_mut().zip(positions) {
            if let Some(sample) = sample {
                sample.record(position)?;
            }
        }
        self.samples = updated;
        Ok(())
    }

    pub fn remap(&mut self, order: [usize; 6]) {
        self.samples = order.map(|previous| self.samples[previous]);
    }

    pub fn active(&self) -> impl Iterator<Item = (usize, &EncoderSweep)> {
        self.samples
            .iter()
            .enumerate()
            .filter_map(|(index, sample)| sample.as_ref().map(|sample| (index, sample)))
    }

    #[must_use]
    pub fn sign(&self, index: usize) -> Sign {
        self.signs[index]
    }
}

pub(super) fn alignment_arrays(alignment: &Alignment) -> ([Sign; 6], [f64; 6]) {
    let [pan, lift, elbow, flex, roll] = alignment.joint_signs;
    let [
        pan_offset,
        lift_offset,
        elbow_offset,
        flex_offset,
        roll_offset,
    ] = alignment.joint_offsets_radians;
    (
        [pan, lift, elbow, flex, roll, alignment.gripper_sign],
        [
            pan_offset,
            lift_offset,
            elbow_offset,
            flex_offset,
            roll_offset,
            alignment.gripper_offset_radians,
        ],
    )
}

#[derive(Debug, Clone, Serialize)]
pub struct JointReading {
    pub name: &'static str,
    pub position_radians: f64,
    pub range_min: i64,
    pub range_max: i64,
    pub sign: Sign,
    pub moving: bool,
    pub offset_radians: f64,
}
