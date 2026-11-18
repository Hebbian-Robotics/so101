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
Choose **New follower batch** to program IDs 1–6 in order or **Single replacement**
to select a joint and its ID directly. Disconnect any active calibration session
first. Turn off servo power before connecting or swapping motors, and connect
only the loose servo being programmed. The tool reads back the ID at 1 Mbps
before marking it verified. Label each motor with its verified joint and ID,
then install and daisy chain the motors during assembly.

Open **Calibrate arm** after assembly and select its USB controller. On a fresh
launch, the calibration page selects a single connected controller automatically.
For the soft-gripper-and-camera assembly, support the disabled arm in the shown
resting pose with the gripper closed, then start recording and gently sweep
each joint through its safe travel, including wrist roll and gripper opening.
The tool flags unusually short sweeps so you can continue recording any missed
joint. Never force a joint to satisfy the check. Review and adjust the live
model before saving. The home pose is an estimate,
so compare multiple positions. The stock-gripper variant uses its reference
pose and explicit gripper endpoints.

The server reads motor positions and streams updates to the browser. Streaming
does not increase the motor sampling rate; move slowly and pause at endpoints
so the tool can record the full range.

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
