# Calibration and setup

Run `so101-calibrate` and open the local URL it prints. USB serials select saved
setups; unknown controllers require explicit selection. With the arm supported
and torque off, match the reference pose, record safe joint travel and explicit
gripper endpoints, review model alignment, then save.

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

The default `model_variant` is `"stock"`. For the optional TPU gripper and wrist
camera assembly, set `model_variant = "soft-gripper-camera"` in that arm's
`setup.toml`. The viewer selects the corresponding model and saving calibration
preserves the selection. This selects geometry; it does not recalibrate an
existing arm or establish camera optical calibration.

See [coordinate conventions](src/so101_setup/COORDINATES.md),
[development/build commands](../CONTRIBUTING.md), and the
[official LeRobot guide](https://huggingface.co/docs/lerobot/so101).
Use an external control application for powered operation. This workflow still
requires validation on the particular physical arm.
