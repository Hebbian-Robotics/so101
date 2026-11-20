//! The STS3215 register settings every SO-101 arm runs, defined once by joint.
//!
//! Commissioning writes these registers and the driver's preflight verifies them, so both read
//! this module. Motor ids differ between arms; resolve them to joints through
//! [`MotorIds`](crate::setup::MotorIds) before looking a setting up.

use crate::setup::So101Joint;

/// `P_Coefficient`, `I_Coefficient` and `D_Coefficient` of the position loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionGains {
    pub proportional: u8,
    pub integral: u8,
    pub derivative: u8,
}

const STANDARD_POSITION_GAINS: PositionGains = PositionGains {
    proportional: 16,
    integral: 0,
    derivative: 32,
};

const LOAD_BEARING_POSITION_GAINS: PositionGains = PositionGains {
    proportional: 32,
    integral: 0,
    derivative: 32,
};

/// `Return_Delay_Time` on every servo. Each unit adds 2 us per motor to every bus read; the
/// factory value is 250.
pub const RETURN_DELAY_TIME: u8 = 0;

/// `Maximum_Acceleration` on every servo.
pub const MAXIMUM_ACCELERATION: u8 = 254;

/// `Max_Torque_Limit` on the gripper servo; the arm joints keep their factory value.
pub const GRIPPER_MAX_TORQUE_LIMIT: u16 = 500;

/// `Protection_Current` on the gripper servo; the arm joints keep their factory value.
pub const GRIPPER_PROTECTION_CURRENT: u16 = 250;

/// The position gains a joint's servo runs.
#[must_use]
pub const fn position_gains(joint: So101Joint) -> PositionGains {
    match joint {
        So101Joint::ShoulderLift | So101Joint::ElbowFlex => LOAD_BEARING_POSITION_GAINS,
        So101Joint::ShoulderPan
        | So101Joint::WristFlex
        | So101Joint::WristRoll
        | So101Joint::Gripper => STANDARD_POSITION_GAINS,
    }
}

#[cfg(test)]
mod servo_response_tests {
    use std::collections::BTreeMap;

    use serde::Deserialize;

    use super::{MAXIMUM_ACCELERATION, RETURN_DELAY_TIME, position_gains};
    use crate::setup::So101Joint;

    #[derive(Deserialize)]
    struct FittedRegisterSettings {
        position_gains_p_i_d: [u8; 3],
        position_gains_p_i_d_by_joint: BTreeMap<String, [u8; 3]>,
        return_delay_time: u8,
        maximum_acceleration: u8,
    }

    #[derive(Deserialize)]
    struct ServoResponse {
        register_settings: FittedRegisterSettings,
    }

    // WHY: The simulated servo response was fitted to arms running these settings; changing a
    // register setting without refitting would leave simulation describing other servos.
    #[test]
    fn register_settings_match_the_settings_the_servo_response_was_fitted_under()
    -> anyhow::Result<()> {
        let response: ServoResponse = toml::from_str(include_str!(
            "../../models/soft-gripper-camera/servo_response.toml"
        ))?;
        let fitted = response.register_settings;
        for joint in So101Joint::ALL {
            let gains = position_gains(joint);
            let fitted_gains = fitted
                .position_gains_p_i_d_by_joint
                .get(joint.name())
                .copied()
                .unwrap_or(fitted.position_gains_p_i_d);
            assert_eq!(
                [gains.proportional, gains.integral, gains.derivative],
                fitted_gains,
                "{} position gains",
                joint.name()
            );
        }
        assert_eq!(RETURN_DELAY_TIME, fitted.return_delay_time);
        assert_eq!(MAXIMUM_ACCELERATION, fitted.maximum_acceleration);
        Ok(())
    }
}
