# SO-101 model assets

From TheRobotStudio/SO-ARM100, commit a758567c3978dfeefe282ede0500085a48fe8f78.
Apache-2.0; see LICENSE.txt. Meshes are decimated display geometry and URDF mesh
paths are flattened. Model coordinates require per-arm physical verification.

Use [robot-assemblies.toml](robot-assemblies.toml) to select the URDF for a
setup's `model_variant`.
Paths are relative to this directory. The manifest gives the arm joints in
base-to-tool order, the gripper joint, the tool frame and the closing axis in
that frame. The optional wrist-camera frame describes the nominal mount.

The optional robot-soft-gripper-camera.urdf uses separate TPU-finger and camera
geometry. See SOFT_GRIPPER_NOTICE.txt for its sources, license status, nominal
camera envelope and grasp-frame definition. The Apache-2.0 statement above
applies to the stock assets, not these optional derived meshes.
