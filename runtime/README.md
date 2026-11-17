# SO-101 runtime

`hebbian-so101` is a standalone Rust crate for SO-101 hardware. It owns serial
register I/O, encoder/model conversions, preflight, measured-pose torque enable,
feedback, joint/gripper goals and verified recovery. It also reads `LeRobot`
calibration, derives joint limits, matches USB controller serials to local arm
entries and installs calibration without overwriting differing local data.

It depends on rustypot and small serialization/serial/diagnostic libraries. It
imports no station, HTTP API, planner, policy runtime or robot-use client. The
`schema` feature enables JSON Schema derives for consumers that need them; an
ordinary hardware application does not need that feature.

Live feedback uses rustypot’s `Bus::sync_read_raw_with_error` to read the
contiguous STS3215 state block and preserve each motor’s packet alarm bits.
All register operations share the bus’s single serial-port handle; retries and
request timing remain in this runtime.

## Use

A consumer supplies `config::FeetechConfig`, including the arm's calibration,
model alignment, expected gains, gripper force fraction and hardware health
limits. `config::load_arm_config` can build it from `settings::FeetechSettings`
and a `LeRobot` calibration directory. Both are independent data contracts,
retaining the existing serialized Feetech field names.

```rust,no_run
use hebbian_so101::{config::FeetechConfig, driver::Servos};

fn inspect(config: FeetechConfig) -> anyhow::Result<()> {
    let mut arm = Servos::open(config)?;
    arm.preflight()?;
    let feedback = arm.read_feedback()?;
    println!("Joint radians: {:?}", feedback.positions);
    println!("Motor temperatures: {:?}", feedback.temperatures_celsius);
    Ok(())
}
```

Opening the bus, preflight and reading feedback do not enable torque or send
position goals. `enable`, `command`, `command_gripper`, `release` and the recovery
methods are explicit operations. Preflight verifies calibration, expected gains,
position mode, return delay, health and torque state; it does not tune the arm.

This is a hardware driver rather than a motion controller. Its caller supplies
validated joint goals and owns joint/workspace limits, speed/acceleration/jerk
limits, command admission, timing, operator approval and process supervision.
The driver has no communication watchdog: enabled servos retain their last goal
if their caller dies. The station retains its independent safety guard.

`telemetry::Feedback`, `RecoveryObservation`, `Reconnection` and the typed
`CommunicationFailure` let another controller consume this hardware without
implementing our station's `Plant` interface. Feedback buffers can be reused.
USB discovery reads metadata; it never opens a port. Its registry convention is
`arms/<controller-serial>/`; config retrieval and authentication belong to callers.
Host-specific defaults and inherited `profile.toml` parsing remain outside this
crate.

## Validate and extract

From this directory:

```sh
cargo test
cargo fmt --check
cargo clippy --all-targets --all-features
```

The directory can also be copied outside this workspace and tested/built there.
Its manifest uses no workspace-inherited dependencies or lint settings. Existing
behavior tests exercise preflight refusals, hold-before-enable, recovery,
telemetry/alarms, environment confirmation, commands and encoder conversion
against an in-memory servo bus. They do not establish physical accuracy or tune
an arm.

The browser calibration application is the sibling `setup` Python/web package.
No station or policy runtime is required. Host applications adapt hardware
telemetry/errors to their own controller interfaces and keep application safety
policy outside this crate. See the repository README for upstream references.
