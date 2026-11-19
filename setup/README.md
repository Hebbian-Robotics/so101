# Calibration and setup

From a source checkout, install Node 24, pnpm 12, Python 3.12 or newer and uv,
then build the browser assets and start the tool:

```sh
pnpm install --frozen-lockfile
pnpm build
uv run --project setup so101-calibrate
```

Open the local URL it prints. Use the sidebar to choose **Calibrate arm** or
**Program motor IDs**.

To prepare loose STS3215 servos for a follower, open **Program motor IDs**.
Choose **New follower batch** to program IDs 1–6 in order, skipping motors you
do not need. The checklist distinguishes skipped motors from verified ones.
Choose **Program one ID** to program any number of motors with the same joint ID;
the last successfully programmed ID is remembered on this browser. Disconnect
any active calibration session first. Turn off servo power before connecting or swapping motors, and connect
only the loose servo being programmed. The tool reads back the ID at 1 Mbps
before marking it verified. Label each motor with its verified joint and ID,
then install and daisy chain the motors during assembly.

Open **Calibrate arm** after assembly and select its USB controller. On a fresh
launch, the calibration page selects a single connected controller automatically.
Support the disabled arm in the shown
resting pose with the gripper closed, then start recording and gently sweep
each joint through its safe travel, including wrist roll and gripper opening.
The tool flags unusually short sweeps so you can continue recording any missed
joint. Never force a joint to satisfy the check. Review and adjust the live
model before saving. The home pose is an estimate,
so compare multiple positions.

If one joint needs a fresh calibration, open the arm’s saved setup and choose
**Recalibrate one joint** on the review screen. This option is available when
the saved calibration matches the motor registers. Select the joint, set it
to the reference shown, then start the sweep and move it by hand through its
safe travel. Finish the sweep and confirm its live direction and alignment
before saving. Save updates that joint’s calibration and leaves the other
joints and servo gains intact. **Discard joint calibration** returns to the
saved setup.

The server reads motor positions and streams updates to the browser. Streaming
does not increase the motor sampling rate; move slowly and pause at endpoints
so the tool can record the full range.

Draft capture and range recording change no registers. Save backs up the prior
files/registers, writes and verifies calibration/settings with torque off, then
releases the port. If save fails, reconcile the recorded backup before control.
The tool never enables torque or sends position goals.

`ArmSetup` holds serial identity, motor ID order, model alignment, gripper
endpoints and expected servo settings. `NativeSetupStore` writes `arms/<serial>/setup.toml` and
LeRobot-compatible `calibration.json`. Applications can implement `SetupStore`
for their own format; config retrieval and Git operations remain with callers.
The default store is `$SO101_SETUP_DIRECTORY`, otherwise
`$XDG_CONFIG_HOME/so101-setup` (defaulting to `~/.config/so101-setup`).

Motor IDs may run from `1` to `6` or from `6` to `1` from the base to the
gripper. The default is ascending. For a different order, open **Edit motor mapping** during the full sweep. Choose
an ascending or descending shortcut, or assign each motor ID to a joint. IDs
1 through 6 must each appear once. Move the joints by hand with torque off to
verify the correspondence. Save records the mapping in the `[motor_ids]`
table and leaves the programmed servo IDs intact.

New arms default to the soft gripper with wrist camera and start calibration in
the resting pose. For stock gripper geometry, start with
`so101-calibrate --model-variant stock`.
Existing arms keep the `model_variant` in their `setup.toml`; to change an
existing arm's model, edit that field before starting the tool. The viewer selects the corresponding model and saving calibration
preserves the selection. This selects geometry; it does not recalibrate an
existing arm or establish camera optical calibration.

See [coordinate conventions](src/so101_setup/COORDINATES.md),
[development/build commands](../CONTRIBUTING.md), and the
[official LeRobot guide](https://huggingface.co/docs/lerobot/so101).
Use an external control application for powered operation. This workflow still
requires validation on the particular physical arm.

A full calibration save sets maximum acceleration to 254 on all six motors,
gripper maximum torque to 500, and gripper protection current to 250. These are
raw register values. A single-joint recalibration leaves them alone.

For an arm that needs different values, manually add `servo-overrides.toml`
beside its `setup.toml`. Set only the keys it needs:
`maximum_acceleration`, `gripper_max_torque_limit`, or
`gripper_protection_current`. The tool applies them on the next full save.
New arm setups do not create this file.

### Restore one servo's parameters

Stop the motion station and close the calibration tool. With the arm supported,
all six servos powered and torque off, run:

```sh
uv run --project setup so101-restore-servo --serial BOARD_SERIAL --motor-id 4 --confirm-restore
```

This sends a parameter restore to the specified ID on the daisy chain. It
requires six responding servos and a target STS3215 reporting model 777 and
firmware 3.10. Connect only one arm to the chosen board and make sure its motor
IDs are unique. Before and after snapshots go under the arm's `backups/`
directory. The restore changes the target servo's stored parameters, including
calibration. Restore its calibration and verify it with torque off before
enabling the arm.
