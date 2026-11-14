//! The SO-101's Feetech STS3215 servos, driven through rustypot (the library
//! `LeRobot`'s motor bus also uses).
//!
//! The checks before torque goes on are the preflight; torque goes on holding
//! the present pose. An STS3215 has no communication watchdog: with torque on it
//! keeps tracking its last goal until power is lost or torque is turned off, so
//! the arm holds if the executor dies.
//!
//! The bus sits behind [`ServoBus`] so the logic is tested against a fake;
//! only [`SerialBus`] touches a serial port. Joint signs, offsets, limits and
//! the gripper's endpoints are per arm; the caller supplies them from the
//! profile and the arm's `LeRobot` calibration file.

mod bus;
mod servos;
#[cfg(test)]
mod tests;
mod units;

pub use bus::{BusTiming, SerialBus, ServoBus, StateBlock};
pub use servos::Servos;
pub use units::{radians_to_steps, steps_to_radians};
