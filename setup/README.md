# Calibration and setup

Run `so101-calibrate` and open the local URL it prints. A single connected USB
controller is selected automatically; choose one if several are connected.
For the soft-gripper-and-camera assembly, support the disabled arm in the shown
resting pose with the gripper closed, then start recording and gently sweep
each joint through its safe travel, including wrist roll and gripper opening.
The tool flags unusually short sweeps so you can continue recording any missed
joint. Never force a joint to satisfy the check. Review and adjust the live
model before saving. The home pose is an estimate,
so compare multiple positions. The stock-gripper variant uses its reference
pose and explicit gripper endpoints.

Draft capture and range recording change no registers. Save backs up the prior
files/registers, writes and verifies calibration/settings with torque off, then
releases the port. If save fails, reconcile the recorded backup before control.
The tool never enables torque or sends position goals.

`ArmSetup` holds serial identity, model alignment, gripper endpoints and expected
servo settings. `NativeSetupStore` writes `arms/<serial>/setup.toml` and
LeRobot-compatible `calibration.json`. Applications can implement `SetupStore`
for their own format; config retrieval and Git operations remain with callers.
The default store is `$SO101_SETUP_DIRECTORY`, otherwise
`$XDG_CONFIG_HOME/so101-setup` (defaulting to `~/.config/so101-setup`).

The default model is `"stock"`. For a new arm with the optional TPU gripper and
wrist camera assembly, start with `so101-calibrate --model-variant soft-gripper-camera`.
Existing arms keep the `model_variant` in their `setup.toml`; to change an
existing arm's model, edit that field before starting the tool. The viewer selects the corresponding model and saving calibration
preserves the selection. This selects geometry; it does not recalibrate an
existing arm or establish camera optical calibration.

See [coordinate conventions](src/so101_setup/COORDINATES.md),
[development/build commands](../CONTRIBUTING.md), and the
[official LeRobot guide](https://huggingface.co/docs/lerobot/so101).
Use an external control application for powered operation. This workflow still
requires validation on the particular physical arm.
