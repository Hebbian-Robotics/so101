# Soft gripper and wrist camera

Optional original-scale TPU gripper and HBVCAM-W20217 V22 camera assembly.
The stock model remains `robot.urdf`; select `robot-soft-gripper-camera.urdf`
for this assembly. Both retain the same arm joint names and mechanical frames.
Calibration signs, offsets and endpoints remain per-arm configuration.

## Hardware and frames

- The [gripper STEP assembly](https://github.com/xiehuangbao888/soarm_soft_gripper/tree/9ea41e73bf16b6171310913c792bd20ebc6245d3)
  contains a fixed body, moving carrier, two TPU fingers and a PLA camera mount.
- The wrist socket's 24 mm bore and seating plane establish the attachment frame;
  the carrier horn's square screw pattern establishes its angular datum. The
  locating slot and 8.1 mm-spaced screw pair locate the bracket on the body.
- The bracket and camera follow `gripper_link` and `wrist_roll`. The camera is
  centered in the bracket, secured from the back, with the rear cable connector
  on the edge nearest the gripper.
- The [HBVCAM-W20217 V22](https://www.hbvcamera.com/1MP-%20HD-usb-cameras/hbvcam-ov9732-1mp-hd-free-driver-face-recognition-camera-module.html)
  uses an OV9732 sensor and a 100-degree lens; [Seeed SKU 100079619](https://www.seeedstudio.com/Camera-Module-OV9732-1MP-100-USB2-0-USB2-0-L38-W38-H16-05mm-p-6982.html)
  is the supplier listing.
- `gripper_frame_link` is the midpoint between the inward surfaces of the distal
  1 mm of both unloaded fingers at model gripper angle zero. Local +Z points
  toward the tips and the closing axis is local X. Tool position is not the
  lowest point of the assembly.
- `wrist_camera_board_link` is a nominal mounting frame: +Z toward the scene and
  +Y toward the connector edge. It is not a calibrated optical frame. Camera
  collision geometry reserves 38 x 38 mm across the board, 16.05 mm in front
  of the mounting plane and 6.2 mm behind it for the rear connector. This
  conservative envelope is not exact lens or cable geometry. The bracket and
  finger collision geometry uses their generated meshes.

Verify fingertip alignment, jaw travel and clearance against the installed arm
with torque off. TPU deformation, optical extrinsics, image rotation, mirroring
and lens intrinsics require physical checks or calibration. Table height and
workspace placement remain scene observations.

## Generate and select

Run from the source checkout:

```sh
uv run models/soft-gripper-camera/tools/build_model.py
uv run models/soft-gripper-camera/tools/build_mjcf.py
```

The generator checks the pinned CAD checksum and writes the optional URDF and
meshes under `setup/src/so101_setup/assets/robot/so101`. It does not modify the
stock URDF. Source dimensions are millimeters; exported meshes use meters.
The separate mount STL translates into STEP coordinates by
`[18.523165, -54.884434, -44.800674]` mm with no rotation.

The gripper and moving-jaw inertials are estimates, not weighings: each printed
part's CAD volume times its material density and an effective fill, plus the
camera module as a uniform box, with the replaced stock wrist part removed from
the stock gripper inertial. [sources.toml](sources.toml) `[mass_estimate]`
records the inputs.

`build_mjcf.py` writes `robot-soft-gripper-camera.xml`, a MuJoCo model of the
same assembly for simulation and policy training. It keeps the URDF's links,
frames, collision meshes and inertials, adds a position actuator per joint with
the servo response fitted to real arms ([servo_response.toml](servo_response.toml),
including the command delay as the model's `command_delay_seconds` numeric), adds a
`gripper_frame` site on `gripper_link` in the pose of `gripper_frame_link`, bounds
each collision hull to 128 vertices and excludes contacts between
jointed links.

The servo response holds only for the register settings it was fitted under,
which commissioning writes to every arm. A runtime test fails when those
defaults change; refit the response from new traces before changing them.

For calibration, set `model_variant = "soft-gripper-camera"` in the arm's
`setup.toml`; omit it or use `"stock"` for the default. Control applications must
select the same URDF that they render. Camera and gripper geometry is generic;
serial ports, tuning and calibration files do not belong in the model package.

## Sources and attribution

[sources.toml](sources.toml) contains source revisions, hashes and dimensions;
[UPSTREAM_README.md](UPSTREAM_README.md) preserves the printing instructions.
The soft-gripper source declares no license at the pinned revision and the
separate STL has no specified source license. These assets and derived meshes
are not covered by the software MIT or stock model Apache-2.0 licenses. Their
notice accompanies the optional model as `SOFT_GRIPPER_NOTICE.txt`.
