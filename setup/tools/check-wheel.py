"""Verify the installed wheel outside the checkout, including its actual HTTP CLI.

Run with a built wheel path. The temporary setup store has no registered arms,
so API discovery reads USB metadata and never opens a serial device.
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from xml.etree import ElementTree


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    arguments = parser.parse_args()
    wheel = arguments.wheel.resolve(strict=True)
    if wheel.is_dir():
        wheels = list(wheel.glob("*.whl"))
        if len(wheels) != 1:
            raise ValueError("The distribution directory must contain exactly one wheel")
        wheel = wheels[0]
    with tempfile.TemporaryDirectory(prefix="so101-wheel-check-") as temporary:
        directory = Path(temporary)
        environment = directory / "environment"
        subprocess.run(
            ["uv", "venv", "--python", sys.executable, str(environment)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
        subprocess.run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--exclude-newer",
                "3 days",
                str(wheel),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        subprocess.run(
            [
                str(python),
                "-I",
                "-c",
                """
import importlib.util
from so101_setup.runtime import asset_directory
for forbidden in ('hebbian_robot_use_tools', 'hebbian_robot_use_client', 'torch', 'lerobot'):
    assert importlib.util.find_spec(forbidden) is None, forbidden
assets = asset_directory()
assert (assets / 'index.html').is_file()
assert (assets / 'THIRD_PARTY_NOTICES.txt').is_file()
assert (assets / 'robot/so101/robot.urdf').is_file()
assert (assets / 'robot/so101/LICENSE.txt').is_file()
""",
            ],
            cwd=directory,
            check=True,
        )
        process = subprocess.Popen(
            [
                str(python),
                "-I",
                "-m",
                "so101_setup.server",
                "--setup-directory",
                str(directory / "setups"),
                "--port",
                "0",
            ],
            cwd=directory,
            stdout=subprocess.PIPE,
            text=True,
        )
        try:
            assert process.stdout is not None
            line = process.stdout.readline()
            address = urlsplit(line.removeprefix("SO-101 calibration: ").strip())
            if address.hostname != "127.0.0.1" or address.port is None or not address.fragment:
                raise RuntimeError(f"Calibration CLI did not start: {line!r}")
            origin = f"http://127.0.0.1:{address.port}"
            for path in ("/", "/robot/so101/LICENSE.txt", "/robot/so101/SOFT_GRIPPER_NOTICE.txt"):
                with urlopen(origin + path, timeout=5) as response:
                    assert response.status == 200 and response.read()
            for model in ("robot.urdf", "robot-soft-gripper-camera.urdf"):
                with urlopen(f"{origin}/robot/so101/{model}", timeout=5) as response:
                    robot = ElementTree.fromstring(response.read())
                mesh_paths = {mesh.attrib["filename"] for mesh in robot.iter("mesh")}
                assert mesh_paths, model
                for mesh_path in mesh_paths:
                    with urlopen(f"{origin}/robot/so101/{mesh_path}", timeout=5) as response:
                        assert response.status == 200 and response.read(), mesh_path
            request = Request(
                origin + "/api/state",
                headers={"Authorization": f"Bearer {address.fragment}"},
            )
            with urlopen(request, timeout=5) as response:
                state = json.load(response)
                assert state["stage"] == "disconnected"
                assert all(not controller["registered"] for controller in state["controllers"])
        finally:
            process.terminate()
            process.wait(timeout=10)
    print(
        "Installed wheel verified: independent imports, bundled UI/model and local HTTP discovery; no serial device opened."
    )


if __name__ == "__main__":
    main()
