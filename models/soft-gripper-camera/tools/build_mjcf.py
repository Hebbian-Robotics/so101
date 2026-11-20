# /// script
# requires-python = ">=3.12"
# dependencies = ["mujoco==3.10.0"]
# ///
"""Write a MuJoCo model of the soft gripper and camera assembly from its URDF.

The URDF carries the geometry, frames and (estimated) inertials; this adds what a physics
simulation also needs: the servos' response fitted to the real arms (servo_response.toml), a
position actuator per joint and a site at the tool frame. Run build_model.py first.
"""

import tomllib
from pathlib import Path
from xml.etree import ElementTree

import mujoco

ASSET_DIRECTORY = Path(__file__).resolve().parents[3] / "setup/src/so101_setup/assets/robot/so101"
SERVO_RESPONSE = Path(__file__).resolve().parents[1] / "servo_response.toml"
# MuJoCo has no command delay; simulators read it from the model under this name.
COMMAND_DELAY_NUMERIC = "command_delay_seconds"
TOOL_SITE = "gripper_frame"
MAXIMUM_HULL_VERTICES = 128


def build_mjcf() -> None:
    manifest = tomllib.loads((ASSET_DIRECTORY / "model.toml").read_text())
    assembly = manifest["variants"]["soft-gripper-camera"]
    urdf = ElementTree.parse(ASSET_DIRECTORY / assembly["urdf"]).getroot()
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
    response = tomllib.loads(SERVO_RESPONSE.read_text())["response"]
    unfitted = response["unfitted"]
    for name in (*manifest["joint_names"], manifest["gripper_joint"]):
        group = response["gain_groups"][response["gain_group_by_joint"][name]]
        joint = spec.joint(name)
        joint.armature = unfitted["armature"]
        # CONSTRAINT: MuJoCo 3.10 takes damping as polynomial coefficients; only the linear one.
        joint.damping = [unfitted["joint_damping"], 0.0, 0.0]
        joint.frictionloss = response["joint_friction_loss"]
        actuator = spec.add_actuator(name=name, target=name)
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.set_to_position(kp=group["position_gain"], kv=group["velocity_damping"])
        actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        actuator.ctrlrange = joint.range
        actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        actuator.forcerange = [-unfitted["force_limit"], unfitted["force_limit"]]
    spec.add_numeric(name=COMMAND_DELAY_NUMERIC, data=[response["command_delay_seconds"]])
    # WHY: Collision uses each mesh's convex hull; a bounded hull keeps contact cheap on the
    # high-resolution CAD meshes, as Menagerie's SO-101 does.
    for mesh in spec.meshes:
        mesh.maxhullvert = MAXIMUM_HULL_VERTICES
    # WHY: Kinematics code reads the tool as an offset on the wrist-roll body, the last jointed
    # link, so the site sits there in the fixed tool frame's pose.
    tool_frame = spec.compile().body(manifest["tool_frame"])
    spec.body("gripper_link").add_site(
        name=TOOL_SITE, pos=tool_frame.pos.tolist(), quat=tool_frame.quat.tolist()
    )
    # WHY: MuJoCo skips parent-child contacts only between moving bodies; the base is fixed
    # to the world, and its servo horn sits inside the shoulder's meshes.
    for joint in urdf.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} has no parent or child link")
        spec.add_exclude(bodyname1=parent.attrib["link"], bodyname2=child.attrib["link"])
    # CONSTRAINT: Writing recompiles the spec, so it keeps the absolute mesh directory until
    # the text is out.
    absolute = f'meshdir="{ASSET_DIRECTORY}/"'
    text = spec.to_xml()
    if absolute not in text:
        raise ValueError("Expected the absolute mesh directory in the written model")
    destination = ASSET_DIRECTORY / assembly["mjcf"]
    destination.write_text(text.replace(absolute, 'meshdir="."'))
    print(f"Wrote {destination.name}")


if __name__ == "__main__":
    build_mjcf()
