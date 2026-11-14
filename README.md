# SO-101 drivers and setup

SO-101 hardware access and a local calibration tool, independent of any policy,
teleoperation system or application stack.

- **Rust:** `hebbian-so101` provides servo I/O, model-angle feedback, preflight,
  measured-pose enable, joint/gripper goals and verified recovery.
- **Python/browser:** `so101-calibrate` discovers controllers, captures calibration
  and checks model alignment in a read-only 3D view. The tool never powers or
  moves the arm.
- **TypeScript:** `@hebbian/so101-model` exports the model helpers used by the UI.

## Install the calibration tool

Download the Python wheel from [Releases](https://github.com/Hebbian-Robotics/so101/releases),
then install it with `uv tool install ./hebbian_so101_setup-*.whl` and run
`so101-calibrate`. Python 3.12 or newer is required; the wheel includes the UI
and licensed robot assets. No Node or Rust toolchain is needed for that install.
Use `--setup-directory` to choose where per-arm setup files are stored.

For source installation and checks, see [CONTRIBUTING.md](CONTRIBUTING.md).
See the [setup guide](setup/README.md) and [Rust API guide](runtime/README.md)
for the package interfaces. Consumers can pin this repository's commit in Cargo
or pnpm; Python wheel dependencies can be pinned by release URL and hash.

## Scope and upstream references

The driver implements hardware operations. Its caller owns motion limits,
workspace checks, timing and process supervision. Software tests do not validate
physical calibration or per-arm tuning; see [coordinate conventions and evidence](setup/src/so101_setup/COORDINATES.md).
Per-arm configs belong in a separate user-owned store; none are shipped here.

For assembly, motor setup and the official calibration workflow, use the
[LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) and
[The Robot Studio hardware repository](https://github.com/TheRobotStudio/SO-ARM100).
Servo transport uses [rustypot](https://docs.rs/rustypot/latest/rustypot/).
This library's model coordinates and setup contract are documented here; it is
not the official LeRobot implementation.

## License

Software is MIT licensed. Robot URDF/mesh assets retain their Apache-2.0 license;
third-party browser notices accompany built wheels. See [NOTICE](NOTICE) for
provenance and [LICENSE](LICENSE) for the software license.
