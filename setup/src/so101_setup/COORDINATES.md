# Coordinate conventions and evidence

The package distinguishes raw Feetech encoder readings, LeRobot calibration
entries, and the angles of the bundled SO-101 URDF. They are different frames.

For a calibrated encoder reading `p`, model joint angle is:

```text
q = sign * (p - 2047) * 2π / 4096 + offset_radians
```

4096 is the STS3215 encoder's count per turn. 2047 is the half-turn reference used
by LeRobot homing. These constants follow the encoding and homing convention;
they are not fitted from measurements. Signs and offsets express the particular
assembly's alignment to the model. They are not established by travel limits.

For the soft-gripper-and-camera assembly, the reference capture assumes the
physical arm is in the displayed supported resting pose, with the gripper
closed. The sweep tracks encoder wraparound, including wrist roll, and records
the measured limits. The reference is approximate and must be reviewed against
the real arm. The stock-gripper workflow instead assumes five model joint angles
at zero and a gripper angle of 0.75 rad. The tool
computes candidate homing offsets and model offsets without writing registers.
Feetech reports `(encoder - homing_offset) mod 4096`. When the signed homing
register's limit leaves the captured reference one count away from 2047, the
model offset absorbs that count. Range collection uses this same candidate
coordinate system. The stock-gripper workflow follows LeRobot's full-turn
wrist convention.

The optional gripper endpoints describe intended closed/open positions. They
can be inside the motor's recorded range and can occur in either encoder order.
They remain distinct from the min/max calibration entries written for LeRobot.

LeRobot's degree normalization currently centers at `(range_min + range_max)/2`
and scales by `360/(resolution - 1)`. That convention is valid for its interface;
it is not the bundled URDF frame. Changing an allowed endpoint can change that
midpoint. This package keeps model alignment explicit so changing travel does
not silently redefine geometric zero. A consumer can transform between either
convention; this does not imply that LeRobot is less accurate.

## What has and has not been measured

The transform is a mathematical/model convention. Unit tests prove arithmetic,
round trips and file/register behavior; simulated readings and visual agreement
alone do not independently measure physical accuracy. A browser confirmation
records the operator's judgment, not external ground truth.

The host project's notes report a wrist-roll correction and restricted gripper
endpoints on a tested assembly. Those observations do not qualify the default
mapping or the new one-pose workflow across arms. The standalone nominal gains
are starting settings; a host adapter supplies its existing per-arm gain profile.

Before treating an arm as calibrated, check joint direction, reference alignment
and gripper endpoints with torque off. Compare several additional poses against
independent angle measurements or fixtures where practical, including positions
not used to capture the reference. Validate tracking, heat, voltage, load and
clearance separately before powered operation. The tool never enables torque or
sends goal positions; a failed save can still leave partially updated registers,
which must be reconciled from the recorded backups before control resumes.
