# SO-101 drivers and setup

Connect, calibrate and control SO-101 arms from your own application. Use the
hardware driver with your policy or teleoperation stack, or run the calibration
tool on its own. No RUA installation is required.

## Included

- Rust hardware driver: servo I/O, joint feedback, preflight, torque control and recovery.
- Python calibration tool with a browser UI and read-only 3D model.
- TypeScript model-rendering helpers for React/Three.js viewers.

## Calibrate an arm

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and use
Python 3.12 or newer. Install the released tool and start it:

```sh
uv tool install https://github.com/Hebbian-Robotics/so101/releases/download/v0.1.0/hebbian_so101_setup-0.1.0-py3-none-any.whl
so101-calibrate
```

The wheel is hosted on [GitHub Releases](https://github.com/Hebbian-Robotics/so101/releases),
not PyPI. It includes the browser UI and robot models; you don't need Node,
a Rust toolchain or a source checkout.

Open the local URL printed in the terminal. With the arm supported and torque
off, connect its USB controller, match the reference pose, record safe joint
travel and gripper endpoints, then review and save. The tool never powers or
moves the arm.

Choose a different setup directory when needed:

```sh
so101-calibrate --setup-directory ./my-arm-setups
```

Each controller gets its own `arms/<serial>/setup.toml` and LeRobot-compatible
`calibration.json`. See the [setup guide](setup/README.md) for saved files,
backups and calibration details.

## Use the Rust driver

In an existing Rust project, add the driver from this release:

```sh
cargo add hebbian-so101 --git https://github.com/Hebbian-Robotics/so101 --tag v0.1.0
```

Use `Servos` to open the configured bus, run preflight and read joint angles and
motor health. Enabling torque and sending joint/gripper goals are explicit
operations. Start with the [Rust usage example and configuration contract](runtime/README.md#use).

The driver handles hardware operations. Your application owns motion limits,
workspace checks, timing and process supervision. The Python calibration tool
is optional when using the driver.

## Use the model helpers

In an existing React/Three.js project:

```sh
pnpm add '@hebbian/so101-model@github:Hebbian-Robotics/so101#v0.1.0'
```

Import helpers from `@hebbian/so101-model/robot-model`. For example,
`useRobotModel` loads `robot/so101/robot.urdf` relative to your page. Serve the
[URDF and meshes](setup/src/so101_setup/assets/robot/so101) at that path and
preserve their Apache license. These helpers render the model; use the Rust
driver or your control stack to communicate with the arm.

The optional [TPU gripper and wrist-camera assembly](models/soft-gripper-camera/README.md)
is bundled as `robot-soft-gripper-camera.urdf`, alongside the stock model. Select
`model_variant = "soft-gripper-camera"` in calibration configuration and use the
same URDF in your control application. `useRobotModel(modelUrl)` can load either
variant. Keep the optional model's separate attribution notice with its meshes.

## Hardware and development

For assembly and the official setup workflow, follow the
[LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) and
[The Robot Studio hardware repository](https://github.com/TheRobotStudio/SO-ARM100).
Servo transport uses [rustypot](https://docs.rs/rustypot/latest/rustypot/).
This library's [coordinate conventions](setup/src/so101_setup/COORDINATES.md)
explain its model alignment; physical calibration and tuning still need checks
on the particular arm. Per-arm configs are user-owned and aren't shipped here.

See [contributing](CONTRIBUTING.md) to build from source and run checks.

## License

Software is [MIT](LICENSE). Robot URDF/mesh assets retain their Apache-2.0 license;
third-party browser notices accompany built wheels. See [NOTICE](NOTICE) for
provenance.
