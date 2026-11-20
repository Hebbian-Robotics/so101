# /// script
# requires-python = ">=3.12"
# dependencies = ["mujoco==3.10.0"]
# ///
"""Write a MuJoCo model of the soft gripper and camera assembly from its URDF.

The URDF carries the geometry, frames and (estimated) inertials; this adds what a physics
simulation also needs: the STS3215 servos' joint dynamics, a position actuator per joint and a
site at the tool frame. Run build_model.py first.
"""

from pathlib import Path
from xml.etree import ElementTree

import mujoco

ASSET_DIRECTORY = (
    Path(__file__).resolve().parents[3] / "setup/src/so101_setup/assets/robot/so101"
)
URDF_FILENAME = "robot-soft-gripper-camera.urdf"
MJCF_FILENAME = "robot-soft-gripper-camera.xml"
JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
# STS3215 joint dynamics and position-servo model from MuJoCo Menagerie's SO-101
# (google-deepmind/mujoco_menagerie f3b6013, robotstudio_so101/so101.xml, class "sts3215"),
# fitted there for a servo proportional gain of 16. They describe the servo, not the links.
SERVO_ARMATURE = 0.028
SERVO_DAMPING = 0.60
SERVO_FRICTION_LOSS = 0.052
SERVO_POSITION_GAIN = 998.22
SERVO_VELOCITY_GAIN = 2.731
SERVO_FORCE_LIMIT = 2.94
TOOL_SITE = "gripper_frame"


def build_mjcf() -> None:
    urdf = ElementTree.parse(ASSET_DIRECTORY / URDF_FILENAME).getroot()
    # WHY: Keep the meshes' subdirectories and every link as its own body, so the model's
    # names and frames are the URDF's.
    ElementTree.SubElement(
        ElementTree.SubElement(urdf, "mujoco"),
        "compiler",
        strippath="false",
        fusestatic="false",
        discardvisual="true",
    )
    spec = mujoco.MjSpec.from_string(ElementTree.tostring(urdf, encoding="unicode"))
    spec.modelname = "so101_soft_gripper_camera"
    # WHY: The string has no directory to resolve meshes from; the written model sits beside
    # its meshes, so it names them relative to itself.
    spec.meshdir = str(ASSET_DIRECTORY)
    spec.option.timestep = 0.005
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.option.iterations = 10
    spec.option.ls_iterations = 20
    for name in JOINTS:
        joint = spec.joint(name)
        joint.armature = SERVO_ARMATURE
        # CONSTRAINT: MuJoCo 3.10 takes damping as polynomial coefficients; only the linear one.
        joint.damping = [SERVO_DAMPING, 0.0, 0.0]
        joint.frictionloss = SERVO_FRICTION_LOSS
        actuator = spec.add_actuator(name=name, target=name)
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.set_to_position(kp=SERVO_POSITION_GAIN, kv=SERVO_VELOCITY_GAIN)
        actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        actuator.ctrlrange = joint.range
        actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        actuator.forcerange = [-SERVO_FORCE_LIMIT, SERVO_FORCE_LIMIT]
    spec.body("gripper_frame_link").add_site(name=TOOL_SITE)
    # WHY: MuJoCo skips parent-child contacts only between moving bodies; the base is fixed
    # to the world, and its servo horn sits inside the shoulder's meshes.
    for joint in urdf.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} has no parent or child link")
        spec.add_exclude(
            bodyname1=parent.attrib["link"], bodyname2=child.attrib["link"]
        )
    # CONSTRAINT: Writing recompiles the spec, so it keeps the absolute mesh directory until
    # the text is out.
    absolute = f'meshdir="{ASSET_DIRECTORY}/"'
    text = spec.to_xml()
    if absolute not in text:
        raise ValueError("Expected the absolute mesh directory in the written model")
    (ASSET_DIRECTORY / MJCF_FILENAME).write_text(text.replace(absolute, 'meshdir="."'))
    print(f"Wrote {MJCF_FILENAME}")


if __name__ == "__main__":
    build_mjcf()
