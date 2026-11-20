# Calibration and setup

Run `so101 setup` to open the bundled browser UI. Use the sidebar to choose
**Calibrate arm** or **Program motor IDs**. From a source checkout, build the
browser assets and run the Rust CLI:

```sh
pnpm install --frozen-lockfile
pnpm build
cargo run --manifest-path setup/native/Cargo.toml -- setup
```

To update an executable installed directly from GitHub Releases, close the
running setup session and run `so101 update`. The command selects the archive
for your platform, verifies its SHA-256 digest from GitHub, and replaces the
executable in place. Run `so101 update --check` to check without installing.
Use your package manager for a package-managed install.

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
endpoints, and expected servo settings. Setup writes `arms/<serial>/setup.toml`
and LeRobot-compatible `calibration.json`.
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
`so101 setup --model-variant stock`.
Existing arms keep the `model_variant` in their `setup.toml`; to change an
existing arm's model, edit that field before starting the tool. The viewer selects the corresponding model and saving calibration
preserves the selection. This selects geometry; it does not recalibrate an
existing arm or establish camera optical calibration.

See [coordinate conventions](src/so101_setup/COORDINATES.md),
[development/build commands](../CONTRIBUTING.md), and the
[official LeRobot guide](https://huggingface.co/docs/lerobot/so101).
Use an external control application for powered operation. This workflow still
requires validation on the particular physical arm.

A full calibration save writes the standard servo registers that every arm
runs, defined once in the runtime's `servo_registers` module: position gains
(P/I/D 32/0/32 on the shoulder lift and elbow, 16/0/32 elsewhere), return delay
0 and maximum acceleration 254 on all six motors, and gripper maximum torque 500
and protection current 250. These are raw register values. A single-joint
recalibration leaves them alone. Per-arm overrides are not supported: a
`setup.toml` with a `[servos]` section or a `servo-overrides.toml` beside it is
refused. The legacy Python setup still carries its own copy of these values.

## Reset servo parameters

Stop the motion station and close the calibration tool. With the arm supported,
all six servos powered and torque off, run:

```sh
so101 servo reset --serial BOARD_SERIAL --motor-id 4 --confirm
```

This sends a parameter restore to the specified ID on the daisy chain. It
requires six responding servos and a target STS3215 reporting model 777 and
firmware 3.10. Connect only one arm to the chosen board and make sure its motor
IDs are unique. Before and after snapshots go under the arm's `backups/`
directory. The restore changes the target servo's stored parameters, including
calibration. Restore its calibration and verify it with torque off before
enabling the arm.

Use `--all` in place of `--motor-id` to reset all six. This is the same reset
offered before a full calibration. A partial full reset must be retried before
calibration can proceed.

## Calibrate from the terminal

Run a setup service in one terminal so it can keep sampling during the sweep:

```sh
so101 setup --no-browser
```

In a second terminal, select the controller, choose whether to reset, and
capture the reference and sweep:

```sh
so101 list
so101 calibrate begin --serial BOARD_SERIAL --skip-reset
so101 calibrate sweep start
# Move each joint by hand through its safe range.
so101 calibrate sweep finish
so101 calibrate review --watch
```

If the arm opens to an existing calibration review, run `so101 calibrate restart`
and choose whether to reset before starting a new full sweep.

Use `so101 calibrate map 6,5,4,3,2,1` during the full sweep to change the
physical motor-to-joint mapping. Use `so101 calibrate check-order` before and
after moving only the base, then `so101 calibrate apply-order` to apply the
detected order. During review, `so101 calibrate reverse shoulder-pan` changes
direction, and `so101 calibrate adjust shoulder-pan -0.02` changes its offset in
radians. Check the reported angles and direction against the physical arm.
`so101 calibrate save` displays the draft and asks for confirmation. For scripts,
pass `--confirm --revision <revision>` with the revision from `so101 calibrate
review --json`.

To recalibrate one joint, connect to a saved setup that matches the arm, then
run `so101 calibrate sweep start --joint shoulder-pan`. Finish, review, and
save the same way. Use `so101 calibrate restart` to discard the draft, or
`so101 calibrate disconnect` to release the controller.

To program a loose motor's physical ID, stop calibration or disconnect the arm,
then connect and power just the loose servo. List adapters and program the ID:

```sh
so101 list --adapters
so101 servo id --port USB_PORT --motor-id 4 --isolated
```
