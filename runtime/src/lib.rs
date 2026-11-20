//! SO-101 hardware access without a station, control loop or policy runtime.

#![doc = include_str!("../README.md")]

pub mod calibration;
pub mod commissioning;
pub mod config;
pub mod discovery;
pub mod driver;
mod numeric;
pub mod registry;
pub mod servo_registers;
pub mod settings;
pub mod setup;
pub mod telemetry;

/// Standard SO-101 arm joint order, excluding the gripper.
pub const SO101_JOINT_NAMES: [&str; 5] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
];
