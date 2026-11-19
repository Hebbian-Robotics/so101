//! Read-only calibration capture and verified setup operations for the SO-101.

mod draft;
pub use draft::{CalibrationEntries, Draft, JointReading, Sweep};
mod hardware;
pub use hardware::{ARM_IDS, SetupBus, program_motor_id, restore_servos};
mod session;
pub use session::{Session, SessionView, Stage};
mod storage;
pub use storage::{ArmFiles, SavedFiles};
mod sweep;

pub use sweep::{EncoderSweep, SweepEstimate, joint_radians};

pub const MOTOR_NAMES: [&str; 6] = [
    crate::SO101_JOINT_NAMES[0],
    crate::SO101_JOINT_NAMES[1],
    crate::SO101_JOINT_NAMES[2],
    crate::SO101_JOINT_NAMES[3],
    crate::SO101_JOINT_NAMES[4],
    "gripper",
];

pub const HOME_JOINTS: [f64; 5] = [-0.0037, -1.8435, 1.5765, 1.3296, -1.5536];
pub const HOME_GRIPPER: f64 = -0.12105;
