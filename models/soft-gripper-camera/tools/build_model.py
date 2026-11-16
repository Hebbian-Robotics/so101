# /// script
# requires-python = ">=3.12"
# dependencies = ["cadquery==2.8.0", "trimesh==5.2.1"]
# ///
"""Build the optional assembly from pinned CAD in its mechanical joint frames."""

import hashlib
import math
import tomllib
from pathlib import Path
from xml.etree import ElementTree

import cadquery
import numpy
import trimesh
from numpy.typing import NDArray

Vector = NDArray[numpy.float64]
SOURCE_DIRECTORY = Path(__file__).resolve().parents[1]
ASSET_DIRECTORY = (
    SOURCE_DIRECTORY.parents[1] / "setup/src/so101_setup/assets/robot/so101"
)


def coordinates(vector: cadquery.Vector) -> Vector:
    return numpy.array(vector.toTuple(), dtype=numpy.float64)


def wrist_attachment_transform(body: cadquery.Shape) -> Vector:
    """Locate the 24 mm wrist socket axis and its outward seating plane."""
    socket = next(
        face._geomAdaptor().Cylinder()
        for face in body.Faces()
        if face.geomType() == "CYLINDER"
        and math.isclose(face._geomAdaptor().Cylinder().Radius(), 12, abs_tol=1e-5)
    )
    wrist_axis = numpy.array(socket.Axis().Direction().Coord(), dtype=numpy.float64)
    socket_origin = numpy.array(socket.Location().Coord(), dtype=numpy.float64)
    seating_planes = [
        face
        for face in body.Faces()
        if face.geomType() == "PLANE"
        and face.Area() > 200
        and coordinates(face.normalAt()) @ wrist_axis > 0.999
    ]
    seating_plane = max(
        seating_planes, key=lambda face: coordinates(face.Center()) @ wrist_axis
    )
    seating_origin = socket_origin + wrist_axis * (
        (coordinates(seating_plane.Center()) - socket_origin) @ wrist_axis
    )
    jaw_axis = numpy.array([0.0, -1.0, 0.0])
    jaw_axis -= (jaw_axis @ wrist_axis) * wrist_axis
    jaw_axis /= numpy.linalg.norm(jaw_axis)
    rotation = numpy.stack([numpy.cross(jaw_axis, wrist_axis), jaw_axis, wrist_axis])
    transform = numpy.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ seating_origin
    return transform


def triangle_mesh(shape: cadquery.Shape, transform: Vector) -> trimesh.Trimesh:
    vertices, triangles = shape.tessellate(0.05, 0.1)
    mesh = trimesh.Trimesh(
        vertices=[vertex.toTuple() for vertex in vertices],
        faces=triangles,
        process=True,
    )
    mesh.apply_transform(transform)
    mesh.apply_scale(0.001)
    return mesh


def add_mesh_geometry(link: ElementTree.Element, mesh_filename: str, kind: str) -> None:
    geometry = ElementTree.SubElement(ElementTree.SubElement(link, kind), "geometry")
    ElementTree.SubElement(geometry, "mesh", filename=mesh_filename)


def add_fixed_mesh_link(
    robot: ElementTree.Element,
    link_name: str,
    mesh_filename: str,
    collision_filenames: list[str],
) -> None:
    link = ElementTree.SubElement(robot, "link", name=link_name)
    add_mesh_geometry(link, mesh_filename, "visual")
    for collision_filename in collision_filenames:
        add_mesh_geometry(link, collision_filename, "collision")
    joint = ElementTree.SubElement(
        robot, "joint", name=f"{link_name}_joint", type="fixed"
    )
    ElementTree.SubElement(joint, "parent", link="gripper_link")
    ElementTree.SubElement(joint, "child", link=link_name)


def export_mesh(mesh: trimesh.Trimesh, filename: str) -> str:
    destination = ASSET_DIRECTORY / filename
    destination.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(destination, file_type="stl")
    return filename


def add_camera_module(
    robot: ElementTree.Element,
    mount: cadquery.Shape,
    attachment: Vector,
    board_width_mm: float,
    front_reserve_mm: float,
    rear_reserve_mm: float,
) -> None:
    """Place the nominal module envelope on the bracket's four mounting corners."""
    screw_axes = []
    direction = None
    for face in mount.Faces():
        if face.geomType() != "CYLINDER":
            continue
        cylinder = face._geomAdaptor().Cylinder()
        axis = numpy.array(cylinder.Axis().Direction().Coord())
        if (
            not math.isclose(cylinder.Radius(), 1.0, abs_tol=1e-5)
            or not 0.9 < abs(axis[0]) < 0.91
        ):
            continue
        if direction is None:
            direction = axis if axis[0] > 0 else -axis
        center = numpy.array(cylinder.Location().Coord())
        center -= (center @ direction) * direction
        if not any(
            numpy.linalg.norm(center - existing) < 0.01 for existing in screw_axes
        ):
            screw_axes.append(center)
    if direction is None or len(screw_axes) != 8:
        raise ValueError(
            "Expected eight rounded mounting-slot edges at four camera corners"
        )
    planar_faces = [
        face
        for face in mount.Faces()
        if face.geomType() == "PLANE"
        and face.Area() > 20
        and abs(coordinates(face.normalAt()) @ direction) > 0.999
    ]
    # The camera is on the side facing the fingertips; rear screws pass through the bracket.
    forward = -direction
    plane = min(planar_faces, key=lambda face: coordinates(face.Center()) @ direction)
    board_center = numpy.mean(screw_axes, axis=0)
    board_center += (coordinates(plane.Center()) @ direction) * direction
    # The connector is on the board edge nearest the gripper, toward assembly +Y.
    source_down = numpy.array([0.0, 1.0, 0.0])
    source_down -= (source_down @ forward) * forward
    source_down /= numpy.linalg.norm(source_down)
    source_rotation = numpy.stack(
        [numpy.cross(source_down, forward), source_down, forward], axis=1
    )
    camera_transform = numpy.eye(4)
    camera_transform[:3, :3] = attachment[:3, :3] @ source_rotation
    camera_transform[:3, 3] = (
        attachment[:3, :3] @ board_center + attachment[:3, 3]
    ) * 0.001
    camera = ElementTree.SubElement(robot, "link", name="wrist_camera_board_link")
    # Reserve the full supplier-listed height in front and the OEM connector
    # projection behind the mounting plane; this is an envelope, not lens CAD.
    for kind in ("visual", "collision"):
        element = ElementTree.SubElement(camera, kind)
        envelope_center = (front_reserve_mm - rear_reserve_mm) * 0.0005
        ElementTree.SubElement(
            element, "origin", xyz=f"0 0 {envelope_center:.9f}", rpy="0 0 0"
        )
        geometry = ElementTree.SubElement(element, "geometry")
        width = board_width_mm * 0.001
        depth = (front_reserve_mm + rear_reserve_mm) * 0.001
        ElementTree.SubElement(
            geometry, "box", size=f"{width:.9f} {width:.9f} {depth:.9f}"
        )
    joint = ElementTree.SubElement(
        robot, "joint", name="wrist_camera_board_joint", type="fixed"
    )
    ElementTree.SubElement(joint, "parent", link="gripper_link")
    ElementTree.SubElement(joint, "child", link="wrist_camera_board_link")
    ElementTree.SubElement(
        joint,
        "origin",
        xyz=" ".join(f"{value:.9f}" for value in camera_transform[:3, 3]),
        rpy=" ".join(
            f"{value:.9f}"
            for value in trimesh.transformations.euler_from_matrix(camera_transform)
        ),
    )


def build_model() -> None:
    manifest = tomllib.loads((SOURCE_DIRECTORY / "sources.toml").read_text())
    source = manifest["gripper"]
    source_path = SOURCE_DIRECTORY / source["file"]
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != source["sha256"]:
        raise ValueError("Gripper CAD does not match its pinned checksum")
    imported_solids = cadquery.importers.importStep(str(source_path)).solids().vals()
    solids: list[cadquery.Shape] = []
    for imported_solid in imported_solids:
        if not isinstance(imported_solid, cadquery.Solid):
            raise TypeError("Expected solid CAD components")
        solids.append(imported_solid)
    if len(solids) != 5:
        raise ValueError(
            "Expected body, carrier, fixed finger, moving finger and camera mount"
        )
    body, carrier, fixed_finger, moving_finger, camera_mount = solids
    attachment = wrist_attachment_transform(body)
    mesh_prefix = "soft-gripper-camera"
    meshes = {
        "body": triangle_mesh(body, attachment),
        "fixed_finger": triangle_mesh(fixed_finger, attachment),
        "moving_carrier": triangle_mesh(carrier, attachment),
        "moving_finger": triangle_mesh(moving_finger, attachment),
        "camera_mount": triangle_mesh(camera_mount, attachment),
    }
    robot = ElementTree.parse(ASSET_DIRECTORY / "robot.urdf").getroot()
    robot.set("name", "so101_soft_gripper_camera")
    gripper = robot.find("link[@name='gripper_link']")
    moving_jaw = robot.find("link[@name='moving_jaw_so101_v1_link']")
    gripper_joint_origin = robot.find("joint[@name='gripper']/origin")
    if gripper is None or moving_jaw is None or gripper_joint_origin is None:
        raise ValueError("Stock model is missing its gripper joint frames")
    for geometry_kind in ("visual", "collision"):
        for element in list(gripper.findall(geometry_kind)):
            mesh_element = element.find("geometry/mesh")
            if (
                mesh_element is not None
                and mesh_element.get("filename") == "wrist_roll_follower_so101_v1.stl"
            ):
                gripper.remove(element)
        for element in list(moving_jaw.findall(geometry_kind)):
            moving_jaw.remove(element)
    joint_position = numpy.array(
        [float(value) for value in gripper_joint_origin.attrib["xyz"].split()]
    )
    joint_rotation = trimesh.transformations.euler_matrix(math.pi / 2, 0, 0)
    joint_rotation[:3, 3] = joint_position
    # The horn's square bolt pattern establishes the carrier's angular datum.
    horn_centers = []
    for face in carrier.Faces():
        if face.geomType() != "CYLINDER":
            continue
        cylinder = face._geomAdaptor().Cylinder()
        if not math.isclose(cylinder.Radius(), 1.6, abs_tol=1e-5):
            continue
        location = numpy.array(cylinder.Location().Coord())
        center = attachment[:3, :3] @ location + attachment[:3, 3]
        if math.isclose(
            numpy.linalg.norm(center[[0, 2]] * 0.001 - joint_position[[0, 2]]),
            0.007,
            abs_tol=0.00001,
        ) and not any(
            numpy.linalg.norm(center[[0, 2]] - existing[[0, 2]]) < 0.001
            for existing in horn_centers
        ):
            horn_centers.append(center)
    if len(horn_centers) != 4:
        raise ValueError("Expected four 7 mm-radius horn screw axes")
    lower_holes = sorted(horn_centers, key=lambda point: point[0])[:2]
    horn_direction = lower_holes[0] - lower_holes[1]
    if horn_direction[2] < 0:
        horn_direction = -horn_direction
    reference_angle = math.atan2(-horn_direction[0], horn_direction[2])
    reference_rotation = trimesh.transformations.rotation_matrix(
        reference_angle, [0, 0, 1]
    )
    moving_to_joint = numpy.linalg.inv(joint_rotation @ reference_rotation)
    for role in ("moving_carrier", "moving_finger"):
        meshes[role].apply_transform(moving_to_joint)
        filename = export_mesh(meshes[role], f"{mesh_prefix}/{role}.stl")
        add_mesh_geometry(moving_jaw, filename, "visual")
        add_mesh_geometry(moving_jaw, filename, "collision")
    for role in ("body", "fixed_finger"):
        filename = export_mesh(meshes[role], f"{mesh_prefix}/{role}.stl")
        add_mesh_geometry(gripper, filename, "visual")
        add_mesh_geometry(gripper, filename, "collision")
    mount_filename = export_mesh(
        meshes["camera_mount"], f"{mesh_prefix}/camera_mount.stl"
    )
    add_fixed_mesh_link(
        robot, "wrist_camera_mount_link", mount_filename, [mount_filename]
    )
    camera = manifest["camera"]
    add_camera_module(
        robot,
        camera_mount,
        attachment,
        camera["collision_board_width_mm"],
        camera["collision_front_reserve_mm"],
        camera["collision_rear_reserve_mm"],
    )
    fixed_vertices = meshes["fixed_finger"].vertices
    moving_vertices = meshes["moving_finger"].vertices.copy()
    moving_vertices = trimesh.transformations.transform_points(
        moving_vertices, joint_rotation
    )
    fixed_tip = fixed_vertices[
        fixed_vertices[:, 2] <= fixed_vertices[:, 2].min() + 0.001
    ]
    moving_tip = moving_vertices[
        moving_vertices[:, 2] <= moving_vertices[:, 2].min() + 0.001
    ]
    grasp_position = [
        (fixed_tip[:, 0].max() + moving_tip[:, 0].min()) / 2,
        (fixed_tip[:, 1].mean() + moving_tip[:, 1].mean()) / 2,
        (fixed_tip[:, 2].min() + moving_tip[:, 2].min()) / 2,
    ]
    tool_origin = robot.find("joint[@name='gripper_frame_joint']/origin")
    if tool_origin is None:
        raise ValueError("Stock model is missing its tool frame")
    tool_origin.set("xyz", " ".join(f"{value:.9f}" for value in grasp_position))
    ElementTree.indent(robot)
    ElementTree.ElementTree(robot).write(
        ASSET_DIRECTORY / "robot-soft-gripper-camera.urdf",
        encoding="utf-8",
        xml_declaration=True,
    )
    print(f"Carrier reference angle: {reference_angle:.9f} rad")
    print(f"Nominal grasp point in gripper_link: {grasp_position} m")
    print("Camera and bracket collision envelopes included")


if __name__ == "__main__":
    build_model()
