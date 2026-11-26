use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail, ensure};
use serde::Serialize;

use crate::setup::{ArmSetup, MotorIds};

use super::draft::{CalibrationEntries, Draft, GRIPPER, JointReading, Sweep, alignment_arrays};
use super::hardware::{SetupBus, read_arm, require_torque_off};
use super::storage::{ArmFiles, SaveTarget, SavedFiles, calibration_registers};
use super::sweep::{position_distance, validate_position};
use super::{MOTOR_NAMES, joint_radians};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    Disconnected,
    Reference,
    Sweep,
    Review,
    Saved,
    Error,
}

#[derive(Debug, Clone, Serialize)]
pub struct SessionView {
    pub stage: Stage,
    pub joints: Vec<JointReading>,
    pub gripper_opening: f64,
    pub estimated: bool,
    pub can_recalibrate_joint: bool,
    pub can_reopen_saved_review: bool,
    pub recalibrating_joint: Option<usize>,
    pub message: String,
    pub configuration_path: String,
    pub calibration_path: String,
}

impl Default for SessionView {
    fn default() -> Self {
        Self {
            stage: Stage::Disconnected,
            joints: Vec::new(),
            gripper_opening: 0.5,
            estimated: false,
            can_recalibrate_joint: false,
            can_reopen_saved_review: false,
            recalibrating_joint: None,
            message: String::new(),
            configuration_path: String::new(),
            calibration_path: String::new(),
        }
    }
}

struct JointRecalibration {
    index: usize,
    baseline: CalibrationEntries,
}

enum ReviewSource {
    Saved,
    Adjusted,
    Estimated,
    SingleJoint(JointRecalibration),
}

impl ReviewSource {
    fn joint(&self) -> Option<usize> {
        match self {
            Self::SingleJoint(recalibration) => Some(recalibration.index),
            Self::Saved | Self::Adjusted | Self::Estimated => None,
        }
    }
}

enum State {
    Reference {
        message: String,
        can_reopen_saved_review: bool,
    },
    Sweep {
        recording: Sweep,
        recalibration: Option<JointRecalibration>,
    },
    Review {
        draft: Draft,
        source: ReviewSource,
    },
    Saved(SavedFiles, Vec<JointReading>, f64),
    Failed(String),
}

impl State {
    fn reference(message: impl Into<String>) -> Self {
        Self::Reference {
            message: message.into(),
            can_reopen_saved_review: false,
        }
    }
}

#[derive(PartialEq, Eq)]
struct MotorSnapshot {
    offsets: [i64; 6],
    minimums: [i64; 6],
    maximums: [i64; 6],
}

impl MotorSnapshot {
    fn read(bus: &mut impl SetupBus, ids: [u8; 6]) -> Result<Self> {
        require_torque_off(bus, ids)?;
        ensure!(
            read_arm(bus, ids, "id")? == ids.map(i64::from),
            "Motor IDs could not be verified"
        );
        Ok(Self {
            offsets: read_arm(bus, ids, "homing_offset")?,
            minimums: read_arm(bus, ids, "min_position_limit")?,
            maximums: read_arm(bus, ids, "max_position_limit")?,
        })
    }

    fn matches(&self, entries: &CalibrationEntries) -> bool {
        let registers = calibration_registers(entries);
        self.offsets == registers["homing_offset"]
            && self.minimums == registers["min_position_limit"]
            && self.maximums == registers["max_position_limit"]
    }

    fn remapped(&self, order: [usize; 6]) -> Self {
        Self {
            offsets: order.map(|index| self.offsets[index]),
            minimums: order.map(|index| self.minimums[index]),
            maximums: order.map(|index| self.maximums[index]),
        }
    }
}

pub struct Session<Bus> {
    bus: Option<Bus>,
    pub setup: ArmSetup,
    pub files: ArmFiles,
    snapshot: MotorSnapshot,
    state: State,
    positions: [i64; 6],
    previous_positions: [i64; 6],
    moving_until: [Option<Instant>; 6],
}

impl<Bus: SetupBus> Session<Bus> {
    pub fn connect(mut bus: Bus, files: ArmFiles) -> Result<Self> {
        let setup = files.load()?;
        let ids = setup.motor_ids.ordered();
        let snapshot = MotorSnapshot::read(&mut bus, ids)?;
        let positions = read_positions(&mut bus, ids)?;
        let mut session = Self {
            bus: Some(bus),
            setup,
            files,
            snapshot,
            state: State::reference(""),
            positions,
            previous_positions: positions,
            moving_until: [None; 6],
        };
        session.state = session.initial_state()?;
        Ok(session)
    }

    fn initial_state(&self) -> Result<State> {
        if self.files.reset_incomplete()? {
            return Ok(State::reference(
                "Retry the incomplete reset before calibrating.",
            ));
        }
        if self.files.calibration_incomplete()? {
            return Ok(State::reference(
                "The previous save did not finish. Inspect the backups and capture a new calibration.",
            ));
        }
        let path = self.files.calibration_path();
        if !path.try_exists()? {
            return Ok(State::reference(""));
        }
        let calibration = match self.setup.motor_ids.load_named_calibration(&path) {
            Ok(calibration) => calibration,
            Err(error) => {
                return Ok(State::reference(format!(
                    "Saved calibration could not be reused: {error:#}"
                )));
            }
        };
        if !self.snapshot.matches(&calibration) {
            return Ok(State::reference(
                "Saved calibration differs from the motor registers. Capture a new reference.",
            ));
        }
        let source = if self.files.matches_setup(&self.setup) {
            ReviewSource::Saved
        } else {
            ReviewSource::Adjusted
        };
        self.review_saved(&calibration, source)
    }

    fn review_saved(
        &self,
        calibration: &CalibrationEntries,
        source: ReviewSource,
    ) -> Result<State> {
        let draft = Draft::from_saved(
            calibration,
            &self.setup.alignment,
            self.setup.gripper.clone(),
        )?;
        Ok(State::Review { draft, source })
    }

    fn reload_baseline(&mut self) -> Result<CalibrationEntries> {
        self.files.require_completed_reset()?;
        let calibration = self
            .setup
            .motor_ids
            .load_named_calibration(&self.files.calibration_path())?;
        let ids = self.setup.motor_ids.ordered();
        let snapshot = MotorSnapshot::read(self.bus.as_mut().context("Reconnect the arm")?, ids)?;
        ensure!(
            snapshot == self.snapshot
                && snapshot.matches(&calibration)
                && self.files.matches_setup(&self.setup),
            "Saved calibration no longer matches the arm. Reconnect to inspect it."
        );
        Ok(calibration)
    }

    pub fn has_bus(&self) -> bool {
        self.bus.is_some()
    }

    pub fn pause(&mut self) {
        self.bus = None;
        self.moving_until = [None; 6];
    }

    pub fn reconnect(&mut self, mut bus: Bus) -> Result<()> {
        let ids = self.setup.motor_ids.ordered();
        let snapshot = MotorSnapshot::read(&mut bus, ids)?;
        ensure!(
            snapshot == self.snapshot,
            "Motor calibration changed. Disconnect and start a new draft."
        );
        self.previous_positions = read_positions(&mut bus, ids)?;
        self.bus = Some(bus);
        Ok(())
    }

    pub fn restart(&mut self) -> Result<()> {
        match &self.state {
            State::Saved(..) | State::Failed(_) => bail!("Reconnect before creating a new draft"),
            State::Sweep {
                recalibration: Some(_),
                ..
            }
            | State::Review {
                source: ReviewSource::SingleJoint(_),
                ..
            } => {
                let baseline = self.reload_baseline()?;
                self.state = self.review_saved(&baseline, ReviewSource::Saved)?;
            }
            _ => {
                self.state = State::Reference {
                    message: String::new(),
                    can_reopen_saved_review: matches!(
                        self.initial_state()?,
                        State::Review {
                            source: ReviewSource::Saved,
                            ..
                        }
                    ),
                }
            }
        }
        Ok(())
    }

    pub fn reopen_saved_review(&mut self) -> Result<()> {
        ensure!(
            matches!(
                self.state,
                State::Reference {
                    can_reopen_saved_review: true,
                    ..
                }
            ),
            "Start from a saved setup before returning to its review"
        );
        let baseline = self.reload_baseline()?;
        self.sample()?;
        self.state = self.review_saved(&baseline, ReviewSource::Saved)?;
        Ok(())
    }

    pub fn sample(&mut self) -> Result<()> {
        let ids = self.setup.motor_ids.ordered();
        let positions = read_positions(self.bus.as_mut().context("Reconnect the arm")?, ids)?;
        self.positions = positions;
        if let State::Sweep { recording, .. } = &mut self.state {
            recording.record(positions)?;
            let now = Instant::now();
            for (index, position) in positions.into_iter().enumerate() {
                if position_distance(position, self.previous_positions[index]) >= 8 {
                    self.previous_positions[index] = position;
                    self.moving_until[index] = Some(now + Duration::from_millis(600));
                }
            }
        } else {
            self.previous_positions = positions;
            self.moving_until = [None; 6];
        }
        Ok(())
    }

    pub fn positions(&self) -> [i64; 6] {
        self.positions
    }

    pub fn start_sweep(&mut self, joint: Option<usize>) -> Result<()> {
        self.files.require_completed_reset()?;
        let recalibration = if let Some(index) = joint {
            ensure!(index < 6, "Unknown joint");
            ensure!(
                matches!(
                    self.state,
                    State::Review {
                        source: ReviewSource::Saved,
                        ..
                    }
                ),
                "Single-joint recalibration requires a saved setup matching the arm"
            );
            Some(JointRecalibration {
                index,
                baseline: self.reload_baseline()?,
            })
        } else {
            ensure!(
                matches!(self.state, State::Reference { .. }),
                "Start at the reference pose before beginning a full sweep"
            );
            None
        };
        self.sample()?;
        let recording = Sweep::start(self.positions, &self.setup.alignment, joint)?;
        self.state = State::Sweep {
            recording,
            recalibration,
        };
        Ok(())
    }

    pub fn is_full_sweep(&self) -> bool {
        matches!(
            self.state,
            State::Sweep {
                recalibration: None,
                ..
            }
        )
    }

    pub fn remap(&mut self, mapping: MotorIds) -> Result<()> {
        mapping.validate()?;
        ensure!(
            self.is_full_sweep(),
            "Motor mapping is available during a full sweep"
        );
        let old_ids = self.setup.motor_ids.ordered();
        let new_ids = mapping.ordered();
        let mut order = [0; 6];
        for (index, id) in new_ids.into_iter().enumerate() {
            order[index] = old_ids
                .iter()
                .position(|&previous| previous == id)
                .context("Invalid prior motor mapping")?;
        }
        let snapshot =
            MotorSnapshot::read(self.bus.as_mut().context("Reconnect the arm")?, new_ids)?;
        ensure!(
            self.snapshot.remapped(order) == snapshot,
            "Motor registers changed. Start a fresh sweep."
        );
        if let State::Sweep { recording, .. } = &mut self.state {
            recording.remap(order);
        }
        self.positions = order.map(|index| self.positions[index]);
        self.previous_positions = order.map(|index| self.previous_positions[index]);
        self.moving_until = order.map(|index| self.moving_until[index]);
        self.snapshot = snapshot;
        self.setup.motor_ids = mapping;
        Ok(())
    }

    pub fn finish_sweep(&mut self) -> Result<()> {
        self.sample()?;
        let State::Sweep {
            recording,
            recalibration,
        } = &mut self.state
        else {
            bail!("Start a sweep before finishing it");
        };
        let baseline = recalibration
            .as_ref()
            .map(|recalibration| {
                Draft::from_saved(
                    &recalibration.baseline,
                    &self.setup.alignment,
                    self.setup.gripper.clone(),
                )
            })
            .transpose()?;
        let draft = Draft::estimate(recording, self.snapshot.offsets, baseline)?;
        let source = recalibration
            .take()
            .map_or(ReviewSource::Estimated, ReviewSource::SingleJoint);
        self.state = State::Review { draft, source };
        Ok(())
    }

    pub fn edit(&mut self, index: usize, delta: Option<f64>) -> Result<()> {
        let State::Review { draft, source } = &mut self.state else {
            bail!("Adjust alignment during review");
        };
        ensure!(
            source.joint().is_none_or(|joint| joint == index),
            "Only the selected joint can be adjusted"
        );
        match delta {
            Some(delta) => draft.adjust(index, delta)?,
            None => draft.reverse(index)?,
        }
        if matches!(source, ReviewSource::Saved) {
            *source = ReviewSource::Adjusted;
        }
        Ok(())
    }

    pub fn save(&mut self) -> Result<()> {
        self.files.require_completed_reset()?;
        let last_view = self.view();
        let State::Review { draft, source } = &self.state else {
            bail!("Review the calibration before saving");
        };
        let target = match source {
            ReviewSource::SingleJoint(recalibration) => SaveTarget::Joint {
                index: recalibration.index,
                baseline: &recalibration.baseline,
            },
            _ => SaveTarget::All,
        };
        let ids = self.setup.motor_ids.ordered();
        let bus = self.bus.as_mut().context("Reconnect the arm")?;
        let snapshot = MotorSnapshot::read(bus, ids)?;
        ensure!(
            snapshot == self.snapshot,
            "Motor calibration changed. Reconnect before saving."
        );
        read_positions(bus, ids)?;
        let mut bus = self.bus.take().context("Reconnect the arm")?;
        match self.files.save(&mut bus, &self.setup, draft, target) {
            Ok(saved) => {
                self.state = State::Saved(saved, last_view.joints, last_view.gripper_opening);
                Ok(())
            }
            Err(error) => {
                self.state = State::Failed(format!("{error:#}"));
                Err(error)
            }
        }
    }

    pub fn stage(&self) -> Stage {
        match self.state {
            State::Reference { .. } => Stage::Reference,
            State::Sweep { .. } => Stage::Sweep,
            State::Review { .. } => Stage::Review,
            State::Saved(..) => Stage::Saved,
            State::Failed(_) => Stage::Error,
        }
    }

    pub fn view(&self) -> SessionView {
        let mut view = SessionView {
            stage: self.stage(),
            ..SessionView::default()
        };
        match &self.state {
            State::Saved(saved, joints, gripper_opening) => {
                view.joints.clone_from(joints);
                view.gripper_opening = *gripper_opening;
                view.message = format!("Saved. Backups: {}", saved.backup.display());
                view.configuration_path = saved.configuration.display().to_string();
                view.calibration_path = saved.calibration.display().to_string();
            }
            State::Failed(message) => view.message.clone_from(message),
            State::Sweep {
                recording,
                recalibration,
            } => {
                view.recalibrating_joint = recalibration.as_ref().map(|item| item.index);
                let now = Instant::now();
                view.joints = recording
                    .active()
                    .map(|(index, sample)| JointReading {
                        name: MOTOR_NAMES[index],
                        position_radians: 0.0,
                        range_min: sample.minimum,
                        range_max: sample.maximum,
                        sign: recording.sign(index),
                        moving: self.moving_until[index].is_some_and(|until| now < until),
                        offset_radians: 0.0,
                    })
                    .collect();
            }
            State::Reference {
                message,
                can_reopen_saved_review,
            } => {
                view.message.clone_from(message);
                view.can_reopen_saved_review = *can_reopen_saved_review;
                let (signs, offsets) = alignment_arrays(&self.setup.alignment);
                view.joints = MOTOR_NAMES
                    .into_iter()
                    .enumerate()
                    .map(|(index, name)| {
                        let raw_position = self.positions[index];
                        JointReading {
                            name,
                            position_radians: joint_radians(
                                raw_position,
                                signs[index],
                                offsets[index],
                            ),
                            range_min: raw_position,
                            range_max: raw_position,
                            sign: signs[index],
                            moving: false,
                            offset_radians: offsets[index],
                        }
                    })
                    .collect();
            }
            State::Review { draft, source } => {
                view.estimated = matches!(
                    source,
                    ReviewSource::Estimated | ReviewSource::SingleJoint(_)
                );
                view.can_recalibrate_joint = matches!(source, ReviewSource::Saved);
                view.recalibrating_joint = source.joint();
                view.joints = MOTOR_NAMES
                    .into_iter()
                    .enumerate()
                    .map(|(index, name)| {
                        let joint = draft.joints[index];
                        JointReading {
                            name,
                            position_radians: joint
                                .position(self.positions[index], self.snapshot.offsets[index]),
                            range_min: joint.range_min,
                            range_max: joint.range_max,
                            sign: joint.sign,
                            moving: false,
                            offset_radians: joint.offset,
                        }
                    })
                    .collect();
                let closed = draft.gripper.closed_position_radians;
                let open = draft.gripper.open_position_radians;
                view.gripper_opening = ((view.joints[GRIPPER].position_radians - closed)
                    / (open - closed))
                    .clamp(0.0, 1.0);
            }
        }
        view
    }
}

fn read_positions(bus: &mut impl SetupBus, ids: [u8; 6]) -> Result<[i64; 6]> {
    require_torque_off(bus, ids)?;
    let positions = read_arm(bus, ids, "present_position")?;
    for position in positions {
        validate_position(position)?;
    }
    Ok(positions)
}
