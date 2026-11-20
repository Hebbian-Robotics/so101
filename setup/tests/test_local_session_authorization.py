"""Other local accounts must not inherit the calibrator's session.

`so101-calibrate` binds 127.0.0.1 and currently sets `so101_session` when any
client requests `/` or `/motor-ids`. These tests run as a second account that
cannot write the operator's setup directory or open the operator's serial
device. They fail while that account can still save calibration, restore
servos, program a motor id, or invoke the other session actions.

The native setup service in `setup/native/src/service.rs` sets the same cookie
from its page handler. Its `/api/action`, `/api/motor-ids`, `/api/reset`,
`/api/state`, `/api/events`, `/api/controllers`, and asset routes are not
covered here.
"""

from __future__ import annotations

import json
import os
import pty
import secrets
import subprocess
import tempfile
from collections.abc import Callable, Iterator
from pathlib import Path
from threading import Thread
from typing import Any

import pytest
import rustypot

from so101_setup.calibration import CalibrationSession
from so101_setup.configuration import MOTOR_IDS, motor_id_mapping
from so101_setup.device import RegisterBus
from so101_setup.discovery import ControllerChoice
from so101_setup.motor_ids import ProgrammingPort
from so101_setup.restore import SETTINGS
from so101_setup.server import CalibrationApplication, CalibrationHTTPServer, request_handler

PEER_ACCOUNT = "so101calpeer"
CLIENT = r"""
import json
import os
import sys
from http.client import HTTPConnection

def summarize(response):
    content_type = response.getheader("Content-Type") or ""
    if content_type.startswith("text/event-stream"):
        response.close()
        return {"status": response.status, "stream": True}
    body = response.read()
    summary = {"status": response.status}
    try:
        payload = json.loads(body)
    except json.JSONDecodeError:
        payload = None
    if isinstance(payload, dict):
        if "message" in payload:
            summary["message"] = str(payload["message"])[:300]
        if "stage" in payload:
            summary["stage"] = payload["stage"]
        if isinstance(payload.get("motor_id"), int):
            summary["motor_id"] = payload["motor_id"]
        if isinstance(payload.get("review_revision"), int):
            summary["review_revision"] = payload["review_revision"]
        ports = payload.get("ports")
        if isinstance(ports, list):
            summary["ports"] = [
                item.get("port") for item in ports if isinstance(item, dict) and "port" in item
            ]
    return summary

def main():
    plan = json.loads(sys.argv[1] and open(sys.argv[1], encoding="utf-8").read())
    port = plan["port"]
    host = f"127.0.0.1:{port}"
    origin = f"http://{host}"
    connection = HTTPConnection("127.0.0.1", port, timeout=30)
    connection.request("GET", plan["entry"], headers={"Host": host})
    page = connection.getresponse()
    cookie_header = page.getheader("Set-Cookie")
    page.read()
    cookie = cookie_header.split(";", 1)[0] if cookie_header else None
    calls = []
    planned = plan["calls"]
    for index, call in enumerate(planned):
        headers = {"Host": host, "Origin": origin}
        if cookie:
            headers["Cookie"] = cookie
        connection.request(call["method"], call["path"], call.get("body"), headers)
        summary = summarize(connection.getresponse())
        calls.append(summary)
        revision = summary.get("review_revision")
        if isinstance(revision, int):
            for later in planned[index + 1 :]:
                if later.get("fill_review_revision"):
                    later["body"] = json.dumps(
                        {
                            "action": "save",
                            "model_verified": True,
                            "review_revision": revision,
                        }
                    )
    with open(sys.argv[2], "w", encoding="utf-8") as output:
        json.dump(
            {"entry_status": page.status, "cookie_set": cookie is not None, "calls": calls},
            output,
        )
    os.chmod(sys.argv[2], 0o644)
    connection.close()

if __name__ == "__main__":
    main()
"""


class ArmBus(RegisterBus):
    """Register values at the serial boundary, as the calibration tests use."""

    def __init__(self, motor_ids: list[int] | None = None) -> None:
        self.driven_motor_ids = [1, 2, 3, 4, 5, 6] if motor_ids is None else motor_ids
        self.all_motor_ids = self.driven_motor_ids
        self.definition = rustypot.Sts3215PyController.definition()
        self.registers = {
            "torque_enable": [0] * 6,
            "present_position": [2047] * 6,
            "homing_offset": [0] * 6,
            "min_position_limit": [0] * 6,
            "max_position_limit": [4095] * 6,
            "operating_mode": [0] * 6,
            "p_coefficient": [32] * 6,
            "i_coefficient": [0] * 6,
            "d_coefficient": [32] * 6,
            "return_delay_time": [250] * 6,
            "maximum_acceleration": [50] * 6,
            "max_torque_limit": [1000] * 6,
            "protection_current": [310] * 6,
            "lock": [0] * 6,
        }
        self.writes: list[tuple[int, str, int]] = []

    def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
        if name == "id":
            ids = self.driven_motor_ids if motor_ids is None else motor_ids
            return list(ids)
        ids = self.driven_motor_ids if motor_ids is None else motor_ids
        return [self.registers[name][motor_id - 1] for motor_id in ids]

    def write_acknowledged(self, motor_id: int, register: str, value: int) -> None:
        self.writes.append((motor_id, register, value))
        self.registers[register][motor_id - 1] = value

    def close(self) -> None:
        return None


class RestoreBus:
    """Servo EEPROM image checked by `restore_parameters` before it sends the reset frame."""

    def __init__(self, port: str) -> None:
        self.port = port
        self.driven_motor_ids = [1, 2, 3, 4, 5, 6]
        self.registers = {name: [0] * 6 for name in SETTINGS}
        self.registers["torque_enable"] = [0] * 6
        self.registers["id"] = [1, 2, 3, 4, 5, 6]
        self.registers["model_number"] = [777] * 6
        self.registers["firmware_major_version"] = [3] * 6
        self.registers["firmware_minor_version"] = [10] * 6
        self.writes: list[tuple[int, str, int]] = []

    def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
        ids = self.driven_motor_ids if motor_ids is None else motor_ids
        return [self.registers[name][motor_id - 1] for motor_id in ids]

    def write_acknowledged(self, motor_id: int, register: str, value: int) -> None:
        self.writes.append((motor_id, register, value))
        self.registers[register][motor_id - 1] = value

    def close(self) -> None:
        return None


def peer_account() -> str:
    if subprocess.run(["id", PEER_ACCOUNT], capture_output=True, check=False).returncode != 0:
        subprocess.run(
            [
                "sudo",
                "-n",
                "useradd",
                "--no-create-home",
                "--shell",
                "/usr/sbin/nologin",
                PEER_ACCOUNT,
            ],
            check=True,
        )
    return PEER_ACCOUNT


def run_as_peer(
    script: str, *arguments: str, timeout: float = 30
) -> subprocess.CompletedProcess[str]:
    program = Path(tempfile.mktemp(prefix="so101-peer-", dir="/tmp"))
    program.write_text(script)
    program.chmod(0o644)
    try:
        return subprocess.run(
            ["sudo", "-n", "-u", peer_account(), "python3", str(program), *arguments],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    finally:
        program.unlink(missing_ok=True)


def assert_peer_cannot_write(directory: Path) -> None:
    probe = run_as_peer(
        "import os, sys\n"
        "try:\n"
        "    os.makedirs(sys.argv[1], exist_ok=True)\n"
        "    with open(os.path.join(sys.argv[1], 'peer-write'), 'w', encoding='utf-8') as handle:\n"
        "        handle.write('x')\n"
        "except PermissionError:\n"
        "    raise SystemExit(0)\n"
        "raise SystemExit(3)\n",
        str(directory),
    )
    assert probe.returncode == 0, probe.stderr


def assert_peer_cannot_open(path: str) -> None:
    probe = run_as_peer(
        "import os, sys\n"
        "try:\n"
        "    os.open(sys.argv[1], os.O_RDWR)\n"
        "except PermissionError:\n"
        "    raise SystemExit(0)\n"
        "raise SystemExit(3)\n",
        path,
    )
    assert probe.returncode == 0, probe.stderr


def private_directory(path: Path) -> Path:
    path.mkdir(parents=True, mode=0o700)
    path.chmod(0o700)
    return path


def files_under(*roots: Path) -> list[str]:
    found = []
    for root in roots:
        if not root.exists():
            continue
        found.extend(
            str(item.relative_to(root.parent)) for item in root.rglob("*") if item.is_file()
        )
    return sorted(found)


def arm_snapshot(application: CalibrationApplication, bus: ArmBus, *roots: Path) -> dict[str, Any]:
    session = application.session
    return {
        "files": files_under(*roots),
        "writes": list(bus.writes),
        "auto_connect": application.auto_connect,
        "reset_decided": application.servo_reset_decided,
        "incomplete": sorted(application.incomplete_servo_resets),
        "connected": session is not None,
        "state": None if session is None else type(session.state).__name__,
    }


def page_assets(directory: Path) -> Path:
    assets = directory / "assets"
    assets.mkdir()
    (assets / "index.html").write_text("Calibration UI")
    return assets


def connect_arm(
    directory: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[CalibrationApplication, ArmBus]:
    from so101_setup import server

    bus = ArmBus()
    monkeypatch.setattr(
        server,
        "discover_controllers",
        lambda *_args: [
            ControllerChoice(serial="ARM_A", port="operator-adapter", registered=False)
        ],
    )
    monkeypatch.setattr(server, "RegisterBus", lambda _port, _motor_ids: bus)
    application = CalibrationApplication(directory / "setup", directory / "calibration")
    assert application.view().stage == "reference"
    assert application.session is not None
    return application, bus


def record_travel(session: CalibrationSession, bus: ArmBus) -> None:
    session.start_sweep()
    for position in (2047, 600, 2047, 3494):
        bus.registers["present_position"] = [position] * 6
        session.view()
    session.finish_sweep()


def exchange_directory() -> Path:
    directory = Path("/tmp/so101-peer-exchange")
    directory.mkdir(exist_ok=True)
    directory.chmod(0o777)
    return directory


def other_account_requests(port: int, entry: str, calls: list[dict[str, Any]]) -> dict[str, Any]:
    directory = exchange_directory()
    plan = Path(tempfile.mktemp(prefix="so101-plan-", dir=directory))
    result = Path(tempfile.mktemp(prefix="so101-result-", dir=directory))
    plan.write_text(json.dumps({"port": port, "entry": entry, "calls": calls}))
    plan.chmod(0o644)
    try:
        completed = run_as_peer(CLIENT, str(plan), str(result), timeout=60)
        if completed.returncode != 0 or not result.is_file():
            raise AssertionError(
                f"other account could not finish its request: {completed.returncode} {completed.stderr[-400:]}"
            )
        loaded = json.loads(result.read_text())
    finally:
        plan.unlink(missing_ok=True)
        result.unlink(missing_ok=True)
    if not isinstance(loaded, dict):
        raise AssertionError("other account returned an unexpected result")
    return loaded


def served(application: CalibrationApplication, assets: Path) -> Iterator[CalibrationHTTPServer]:
    token = secrets.token_urlsafe(32)
    server = CalibrationHTTPServer(("127.0.0.1", 0), request_handler(application, assets, token))
    thread = Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
    thread.start()
    try:
        yield server
    finally:
        application.stream_stop.set()
        with application.stream_condition:
            application.stream_condition.notify_all()
        server.shutdown()
        thread.join(timeout=5)
        server.server_close()
        application.close()


def assert_requests_refused(result: dict[str, Any], changed: bool) -> None:
    statuses = [call["status"] for call in result["calls"]]
    refused = bool(statuses) and all(status in {401, 403} for status in statuses)
    if refused and not changed:
        return
    pytest.fail(
        f"other local account used the calibrator session: {result['calls']}, state_changed={changed}"
    )


@pytest.mark.parametrize("entry", ["/", "/motor-ids"], ids=["calibration-page", "motor-id-page"])
def test_session_cookie_from_each_page_cannot_save(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, entry: str
) -> None:
    setup = private_directory(tmp_path / "setup")
    calibration = private_directory(tmp_path / "calibration")
    assert_peer_cannot_write(setup)
    application, bus = connect_arm(tmp_path, monkeypatch)
    session = application.session
    assert session is not None
    record_travel(session, bus)
    before = arm_snapshot(application, bus, setup, calibration)
    with_server(
        application,
        page_assets(tmp_path),
        entry,
        [
            {"method": "GET", "path": "/api/state"},
            {
                "method": "POST",
                "path": "/api/action",
                "fill_review_revision": True,
                "body": json.dumps(
                    {"action": "save", "model_verified": True, "review_revision": 0}
                ),
            },
        ],
        lambda result: assert_requests_refused(
            result, arm_snapshot(application, bus, setup, calibration) != before
        ),
    )


def test_other_local_account_cannot_factory_reset_servos(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from so101_setup import restore

    setup = private_directory(tmp_path / "setup")
    calibration = private_directory(tmp_path / "calibration")
    assert_peer_cannot_write(setup)
    frames: list[bytes] = []

    class RestoreLink:
        def __init__(
            self,
            port: str,
            baudrate: int,
            timeout: float = 0,
            write_timeout: float = 0,
            exclusive: bool = False,
        ) -> None:
            self._response = b""

        def __enter__(self) -> RestoreLink:
            return self

        def __exit__(self, *_exc: object) -> bool:
            return False

        def reset_input_buffer(self) -> None:
            return None

        def write(self, frame: bytes) -> int:
            payload = bytes(frame)
            frames.append(payload)
            motor_id = payload[2]
            self._response = bytes((0xFF, 0xFF, motor_id, 2, 0, ~(motor_id + 2) & 0xFF))
            return len(payload)

        def flush(self) -> None:
            return None

        def read(self, size: int) -> bytes:
            return self._response[:size]

    monkeypatch.setattr(restore, "RegisterBus", RestoreBus)
    monkeypatch.setattr(restore, "Serial", RestoreLink)
    application, bus = connect_arm(tmp_path, monkeypatch)
    before = arm_snapshot(application, bus, setup, calibration)
    with_server(
        application,
        page_assets(tmp_path),
        "/",
        [{"method": "POST", "path": "/api/action", "body": json.dumps({"action": "reset_servos"})}],
        lambda result: assert_requests_refused(
            result,
            arm_snapshot(application, bus, setup, calibration) != before or bool(frames),
        ),
    )


def test_other_local_account_cannot_program_a_motor_id(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from so101_setup import server

    master, slave = pty.openpty()
    port_name = os.ttyname(slave)
    assert_peer_cannot_open(port_name)
    monkeypatch.setattr(
        server,
        "programming_ports",
        lambda: [ProgrammingPort(port=port_name, description="USB serial adapter")],
    )
    application = CalibrationApplication(
        private_directory(tmp_path / "setup"),
        private_directory(tmp_path / "calibration"),
        auto_connect=True,
    )
    try:
        with_server(
            application,
            page_assets(tmp_path),
            "/motor-ids",
            [
                {"method": "GET", "path": "/api/motor-ids"},
                {
                    "method": "POST",
                    "path": "/api/motor-ids",
                    "body": json.dumps({"port": port_name, "motor_id": 2, "isolated_motor": True}),
                },
            ],
            lambda result: assert_requests_refused(
                result,
                application.auto_connect is not True or pty_bytes(master) != b"",
            ),
        )
    finally:
        os.close(master)
        os.close(slave)


def pty_bytes(master: int) -> bytes:
    import select

    collected = b""
    while select.select([master], [], [], 0)[0]:
        chunk = os.read(master, 65536)
        if not chunk:
            break
        collected += chunk
    return collected


REQUESTS = [
    ("read-state", "GET", "/api/state", None),
    ("read-events", "GET", "/api/events", None),
    ("read-motor-ids", "GET", "/api/motor-ids", None),
    ("connect", "POST", "/api/action", {"action": "connect", "serial": "ARM_A"}),
    ("disconnect", "POST", "/api/action", {"action": "disconnect"}),
    ("restart", "POST", "/api/action", {"action": "restart"}),
    ("reopen", "POST", "/api/action", {"action": "reopen"}),
    ("reset-servos", "POST", "/api/action", {"action": "reset_servos"}),
    ("skip-servo-reset", "POST", "/api/action", {"action": "skip_servo_reset"}),
    ("check-motor-order", "POST", "/api/action", {"action": "check_motor_order"}),
    ("apply-detected-motor-order", "POST", "/api/action", {"action": "apply_detected_motor_order"}),
    (
        "map-motor-ids",
        "POST",
        "/api/action",
        {"action": "map_motor_ids", "motor_ids": motor_id_mapping(list(reversed(MOTOR_IDS)))},
    ),
    ("start-sweep", "POST", "/api/action", {"action": "start_sweep"}),
    ("finish-sweep", "POST", "/api/action", {"action": "finish_sweep"}),
    ("reverse-joint", "POST", "/api/action", {"action": "reverse_joint", "index": 0}),
    (
        "adjust-joint",
        "POST",
        "/api/action",
        {"action": "adjust_joint", "index": 0, "delta_radians": 0.01},
    ),
    (
        "save",
        "POST",
        "/api/action",
        {"action": "save", "model_verified": True, "review_revision": 0},
    ),
    (
        "program-motor-while-connected",
        "POST",
        "/api/motor-ids",
        {"port": "operator-adapter", "motor_id": 1, "isolated_motor": True},
    ),
]


@pytest.mark.parametrize(
    ("method", "path", "body"),
    [(method, path, body) for _name, method, path, body in REQUESTS],
    ids=[name for name, *_rest in REQUESTS],
)
def test_other_local_account_cannot_invoke_session_operation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    method: str,
    path: str,
    body: dict[str, Any] | None,
) -> None:
    setup = private_directory(tmp_path / "setup")
    calibration = private_directory(tmp_path / "calibration")
    application, bus = connect_arm(tmp_path, monkeypatch)
    before = arm_snapshot(application, bus, setup, calibration)
    payload = None if body is None else json.dumps(body)
    with_server(
        application,
        page_assets(tmp_path),
        "/",
        [{"method": method, "path": path, "body": payload}],
        lambda result: assert_requests_refused(
            result, arm_snapshot(application, bus, setup, calibration) != before
        ),
    )


def with_server(
    application: CalibrationApplication,
    assets: Path,
    entry: str,
    calls: list[dict[str, Any]],
    check: Callable[[dict[str, Any]], None],
) -> None:
    generator = served(application, assets)
    server = next(generator)
    try:
        result = other_account_requests(server.server_port, entry, calls)
        check(result)
    finally:
        generator.close()
