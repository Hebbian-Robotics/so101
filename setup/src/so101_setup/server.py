"""Local calibration HTTP adapter. The browser has no motor-motion endpoint."""

import argparse
import contextlib
import json
import mimetypes
import os
import secrets
import threading
from collections.abc import Callable
from functools import partial
from http import HTTPStatus
from http.cookies import CookieError, SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from socket import socket
from socketserver import BaseServer
from typing import Annotated, Literal
from urllib.parse import unquote, urlsplit

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

from .calibration import (
    HOME_GRIPPER_RADIANS,
    HOME_JOINTS,
    REFERENCE_GRIPPER_RADIANS,
    REFERENCE_JOINTS,
    CalibrationSession,
    Failed,
    JointView,
    Saved,
)
from .configuration import ModelVariant, MotorIdOrder
from .device import RegisterBus, RegisterReadError
from .discovery import ControllerChoice, discover_controllers
from .motor_ids import FOLLOWER_MOTORS, program_motor_id, programming_ports
from .runtime import asset_directory, setup_directory, setup_lock
from .storage import NativeSetupStore, SetupStore


class ActionModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ProgramMotorAction(ActionModel):
    port: str = Field(min_length=1)
    motor_id: int = Field(ge=1, le=6, strict=True)
    isolated_motor: Literal[True]


class ConnectAction(ActionModel):
    action: Literal["connect"]
    serial: str = Field(pattern=r"^[A-Za-z0-9_-]+$")


class SimpleAction(ActionModel):
    action: Literal["capture_reference", "finish_sweep", "disconnect", "restart", "reopen"]


class StartSweepAction(ActionModel):
    action: Literal["start_sweep"]
    joint_index: int | None = Field(default=None, ge=0, le=5, strict=True)


class ReverseAction(ActionModel):
    action: Literal["reverse_joint"]
    index: int = Field(ge=0, le=5)


class AdjustAction(ActionModel):
    action: Literal["adjust_joint"]
    index: int = Field(ge=0, le=5)
    delta_radians: float = Field(ge=-0.1, le=0.1, allow_inf_nan=False)


class GripperEndpointAction(ActionModel):
    action: Literal["capture_gripper_endpoint"]
    endpoint: Literal["closed", "open"]


class SaveAction(ActionModel):
    action: Literal["save"]
    model_verified: Literal[True]


Action = Annotated[
    ConnectAction
    | SimpleAction
    | StartSweepAction
    | ReverseAction
    | AdjustAction
    | GripperEndpointAction
    | SaveAction,
    Field(discriminator="action"),
]
ACTION_ADAPTER = TypeAdapter(Action)


class ApplicationView(BaseModel):
    model_config = ConfigDict(extra="forbid")
    stage: Literal["disconnected", "reference", "sweep", "review", "saved", "error"]
    controllers: list[ControllerChoice] = Field(default_factory=list)
    serial: str = ""
    model_variant: ModelVariant = "stock"
    joints: list[JointView] = Field(default_factory=list)
    gripper_closed_recorded: bool = False
    gripper_open_recorded: bool = False
    estimated: bool = False
    sweep_only: bool = False
    can_recalibrate_joint: bool = False
    recalibrating_joint: int | None = None
    gripper_opening: float = 0.5
    home_joints_radians: list[float] = Field(default_factory=lambda: list(HOME_JOINTS))
    home_gripper_radians: float = HOME_GRIPPER_RADIANS
    message: str = ""
    command: str = ""
    configuration_path: str = ""
    calibration_path: str = ""


class CalibrationApplication:
    def __init__(
        self,
        config_directory: Path,
        calibration_root: Path,
        store: SetupStore | None = None,
        saved_command: Callable[[str], str] | None = None,
        *,
        motor_id_order_override: tuple[str, MotorIdOrder] | None = None,
    ) -> None:
        self.config_directory = config_directory.resolve()
        self.store = NativeSetupStore() if store is None else store
        self.saved_command = saved_command
        self.motor_id_order_override = motor_id_order_override
        self.calibration_root = calibration_root
        self.session: CalibrationSession | None = None
        self.auto_connect = True
        self.waiting_for_controller = False
        self.calibration_registers: dict[str, list[int]] = {}
        self.lock = threading.RLock()
        self.stream_condition = threading.Condition()
        self.stream_subscribers = 0
        self.stream_sequence = 0
        self.stream_payload = b""
        self.stream_stop = threading.Event()
        self.stream_thread: threading.Thread | None = None

    def subscribe(self) -> int:
        with self.stream_condition:
            self.stream_subscribers += 1
            if self.stream_thread is None or not self.stream_thread.is_alive():
                self.stream_stop.clear()
                self.stream_thread = threading.Thread(target=self.sample_views, daemon=True)
                self.stream_thread.start()
            return self.stream_sequence

    def unsubscribe(self) -> None:
        with self.stream_condition:
            self.stream_subscribers -= 1

    def sample_views(self) -> None:
        while not self.stream_stop.is_set():
            with self.stream_condition:
                if self.stream_subscribers == 0:
                    self.stream_thread = None
                    return
            with self.lock:
                try:
                    payload = (
                        b"event: view\ndata: " + self.view().model_dump_json().encode() + b"\n\n"
                    )
                except Exception as error:
                    payload = (
                        b"event: view-error\ndata: "
                        + json.dumps({"message": str(error)}).encode()
                        + b"\n\n"
                    )
                with self.stream_condition:
                    self.stream_payload = payload
                    self.stream_sequence += 1
                    self.stream_condition.notify_all()
            self.stream_stop.wait(0.2)

    def controllers(self) -> list[ControllerChoice]:
        controllers = discover_controllers(self.config_directory, self.store)
        if self.motor_id_order_override is None:
            return controllers
        override_serial, _ = self.motor_id_order_override
        return [controller for controller in controllers if controller.serial == override_serial]

    @staticmethod
    def read_calibration_registers(bus: RegisterBus) -> dict[str, list[int]]:
        return {
            name: bus.read_registers(name)
            for name in ("id", "homing_offset", "min_position_limit", "max_position_limit")
        }

    def connect(self, serial: str) -> None:
        if self.session is not None:
            raise ValueError("Disconnect the current arm before selecting another")
        candidates = [
            controller for controller in self.controllers() if controller.serial == serial
        ]
        if len(candidates) != 1:
            raise ValueError("The USB serial must identify exactly one connected controller")
        controller = candidates[0]
        directory = self.config_directory / "arms" / serial
        setup = self.store.load(directory, serial)
        if self.motor_id_order_override is not None:
            _, motor_id_order = self.motor_id_order_override
            setup = setup.model_copy(update={"motor_id_order": motor_id_order})
        bus = RegisterBus(controller.port, setup.motor_ids_in_joint_order())
        try:
            session = CalibrationSession(setup, directory, self.calibration_root, bus, self.store)
            self.calibration_registers = self.read_calibration_registers(bus)
            self.session = session
        except Exception:
            bus.close()
            raise

    def reconnect(self, session: CalibrationSession) -> None:
        controller = next(
            (
                controller
                for controller in self.controllers()
                if controller.serial == session.serial
            ),
            None,
        )
        if controller is None:
            raise RegisterReadError(f"Waiting for USB controller {session.serial}")
        bus = RegisterBus(controller.port, session.setup.motor_ids_in_joint_order())
        try:
            if any(bus.read_registers("torque_enable")):
                raise RuntimeError("Disable torque on all motors before reconnecting")
            if self.read_calibration_registers(bus) != self.calibration_registers:
                raise RuntimeError(
                    "Motor calibration changed; discard this draft before continuing"
                )
        except Exception:
            bus.close()
            raise
        session.bus = bus
        self.waiting_for_controller = False

    def view(self) -> ApplicationView:
        with self.lock:
            if self.session is None:
                controllers = self.controllers()
                if self.auto_connect and len(controllers) == 1:
                    try:
                        self.connect(controllers[0].serial)
                    except (OSError, RuntimeError, ValueError) as error:
                        return ApplicationView(
                            stage="disconnected",
                            controllers=controllers,
                            message=f"Could not connect to {controllers[0].serial}: {error}",
                        )
                else:
                    return ApplicationView(
                        stage="disconnected",
                        controllers=controllers,
                        message="Plug in an arm, or select its USB controller below.",
                    )
            assert self.session is not None
            session = self.session
            try:
                if self.waiting_for_controller:
                    self.reconnect(session)
                view = ApplicationView.model_validate(session.view().model_dump())
            except RegisterReadError as error:
                if isinstance(session.state, (Saved, Failed)):
                    raise
                if not self.waiting_for_controller:
                    self.waiting_for_controller = True
                    with contextlib.suppress(OSError, RuntimeError):
                        session.bus.close()
                return ApplicationView(
                    stage="error",
                    serial=session.serial,
                    message=f"Calibration paused. Reconnecting to the same board: {error}",
                )
            except (OSError, RuntimeError) as error:
                if not self.waiting_for_controller:
                    raise
                return ApplicationView(stage="error", serial=session.serial, message=str(error))
            view.model_variant = self.session.setup.model_variant
            if view.model_variant == "stock":
                view.home_joints_radians = list(REFERENCE_JOINTS)
                view.home_gripper_radians = REFERENCE_GRIPPER_RADIANS
            if view.stage == "saved" and self.saved_command is not None:
                view.command = self.saved_command(view.serial)
            return view

    def act(self, action: Action) -> ApplicationView:
        with self.lock:
            if isinstance(action, ConnectAction):
                self.connect(action.serial)
            elif isinstance(action, SimpleAction) and action.action == "disconnect":
                self.close_session()
                self.auto_connect = False
            else:
                if self.session is None:
                    raise ValueError("Connect an arm first")
                if self.waiting_for_controller:
                    raise ValueError("Wait for the original controller to reconnect")
                match action:
                    case SimpleAction(action="reopen"):
                        if self.session.view().stage != "saved":
                            raise ValueError("Reopen is available after saving")
                        serial = self.session.serial
                        self.session = None
                        self.connect(serial)
                    case SimpleAction(action="restart"):
                        self.session.restart()
                    case StartSweepAction(joint_index=joint_index):
                        self.session.start_sweep(joint_index)
                    case SimpleAction(action="capture_reference"):
                        self.session.capture_reference()
                    case SimpleAction(action="finish_sweep"):
                        self.session.finish_sweep()
                    case ReverseAction(index=index):
                        self.session.reverse_joint(index)
                    case AdjustAction(index=index, delta_radians=delta):
                        self.session.adjust_joint(index, delta)
                    case GripperEndpointAction(endpoint=endpoint):
                        self.session.capture_gripper_endpoint(endpoint)
                    case SaveAction(model_verified=verified):
                        self.session.save(model_verified=verified)
            return self.view()

    def program_motor(self, action: ProgramMotorAction) -> int:
        with self.lock:
            if self.session is not None:
                raise ValueError("Open Calibrate arm and disconnect the arm first.")
            if action.port not in {controller.port for controller in programming_ports()}:
                raise ValueError(
                    "The selected USB adapter is unavailable. Refresh the adapter list."
                )
            self.auto_connect = False
            return program_motor_id(action.port, action.motor_id)

    def close(self) -> None:
        self.stream_stop.set()
        with self.stream_condition:
            self.stream_condition.notify_all()
            stream_thread = self.stream_thread
        if stream_thread is not None and stream_thread is not threading.current_thread():
            stream_thread.join()
        self.close_session()

    def close_session(self) -> None:
        if self.session is not None:
            if not self.waiting_for_controller:
                self.session.bus.close()
            self.session = None
            self.waiting_for_controller = False


def calibration_root() -> Path:
    # Same environment precedence as LeRobot and the Rust calibration adapter.
    if root := os.environ.get("HF_LEROBOT_CALIBRATION"):
        return Path(root).expanduser()
    if root := os.environ.get("HF_LEROBOT_HOME"):
        return Path(root).expanduser() / "calibration"
    hugging_face = (
        Path(os.environ["HF_HOME"]).expanduser()
        if os.environ.get("HF_HOME")
        else Path(os.environ.get("XDG_CACHE_HOME") or "~/.cache").expanduser() / "huggingface"
    )
    return hugging_face / "lerobot/calibration"


class CalibrationRequestHandler(BaseHTTPRequestHandler):
    def __init__(
        self,
        request: socket,
        client_address: tuple[str, int],
        server: BaseServer,
        *,
        application: CalibrationApplication,
        asset_directory: Path,
        token: str,
    ) -> None:
        self.application = application
        self.asset_directory = asset_directory
        self.token = token
        super().__init__(request, client_address, server)

    def setup(self) -> None:
        super().setup()
        self.connection.settimeout(5)

    def reply(
        self,
        status: HTTPStatus,
        content: bytes,
        content_type: str = "application/json",
        *,
        set_session: bool = False,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if set_session:
            self.send_header(
                "Set-Cookie", f"so101_session={self.token}; HttpOnly; SameSite=Strict; Path=/"
            )
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(content)

    def local_host(self) -> bool:
        address = self.server.server_address
        if not isinstance(address, tuple) or len(address) < 2:
            return False
        port = address[1]
        return self.headers.get("Host", "") in {f"127.0.0.1:{port}", f"localhost:{port}"}

    def authenticated(self) -> bool:
        try:
            cookies = SimpleCookie(self.headers.get("Cookie", ""))
        except CookieError:
            return False
        session = cookies.get("so101_session")
        return session is not None and secrets.compare_digest(session.value, self.token)

    def do_GET(self) -> None:
        if not self.local_host():
            self.reply(HTTPStatus.FORBIDDEN, b"{}")
            return
        path = urlsplit(self.path).path
        if path == "/api/events":
            if not self.authenticated():
                self.reply(HTTPStatus.UNAUTHORIZED, b"{}")
                return
            self.stream_events()
            return
        if path == "/api/motor-ids":
            if not self.authenticated():
                self.reply(HTTPStatus.UNAUTHORIZED, b"{}")
                return
            self.reply(
                HTTPStatus.OK,
                json.dumps(
                    {
                        "motors": [motor.model_dump() for motor in FOLLOWER_MOTORS],
                        "ports": [port.model_dump() for port in programming_ports()],
                    }
                ).encode(),
            )
            return
        if path == "/api/state":
            if not self.authenticated():
                self.reply(
                    HTTPStatus.UNAUTHORIZED,
                    b'{"message":"Reload the calibration page to start a session."}',
                )
                return
            try:
                self.reply(HTTPStatus.OK, self.application.view().model_dump_json().encode())
            except Exception as error:
                self.reply(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    json.dumps({"message": str(error)}).encode(),
                )
            return
        requested = (self.asset_directory / unquote(path).lstrip("/")).resolve()
        if path in {"/", "/motor-ids"}:
            requested = self.asset_directory / "index.html"
        if not requested.is_relative_to(self.asset_directory) or not requested.is_file():
            self.reply(HTTPStatus.NOT_FOUND, b"Not found", "text/plain")
            return
        self.reply(
            HTTPStatus.OK,
            requested.read_bytes(),
            mimetypes.guess_type(requested.name)[0] or "application/octet-stream",
            set_session=path in {"/", "/motor-ids"},
        )

    def stream_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        sequence = self.application.subscribe()
        try:
            while not self.application.stream_stop.is_set():
                with self.application.stream_condition:
                    self.application.stream_condition.wait_for(
                        lambda last_sequence=sequence: (
                            self.application.stream_sequence != last_sequence
                            or self.application.stream_stop.is_set()
                        ),
                        timeout=1,
                    )
                    if self.application.stream_stop.is_set():
                        break
                    if self.application.stream_sequence == sequence:
                        continue
                    sequence = self.application.stream_sequence
                    payload = self.application.stream_payload
                self.wfile.write(payload)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            self.application.unsubscribe()

    def do_POST(self) -> None:
        if (
            not self.local_host()
            or self.headers.get("Origin") != f"http://{self.headers.get('Host', '')}"
        ):
            self.reply(HTTPStatus.FORBIDDEN, b"{}")
            return
        path = urlsplit(self.path).path
        if path not in {"/api/action", "/api/motor-ids"}:
            self.reply(HTTPStatus.NOT_FOUND, b"{}")
            return
        if not self.authenticated():
            self.reply(HTTPStatus.UNAUTHORIZED, b"{}")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("Action body must be 1-4096 bytes")
            body = self.rfile.read(length)
            if path == "/api/motor-ids":
                motor_id = self.application.program_motor(
                    ProgramMotorAction.model_validate_json(body)
                )
                self.reply(HTTPStatus.OK, json.dumps({"motor_id": motor_id}).encode())
            else:
                action = ACTION_ADAPTER.validate_json(body)
                self.reply(HTTPStatus.OK, self.application.act(action).model_dump_json().encode())
        except Exception as error:
            self.reply(HTTPStatus.BAD_REQUEST, json.dumps({"message": str(error)}).encode())

    def log_message(self, format: str, *args: object) -> None:
        pass


def request_handler(
    application: CalibrationApplication, asset_directory: Path, token: str
) -> Callable[[socket, tuple[str, int], BaseServer], CalibrationRequestHandler]:
    return partial(
        CalibrationRequestHandler,
        application=application,
        asset_directory=asset_directory,
        token=token,
    )


class CalibrationHTTPServer(ThreadingHTTPServer):
    # Let an in-flight save finish before releasing the port and motion lock
    # during ordinary shutdown. The request socket timeout bounds idle clients.
    daemon_threads = False


def serve_application(
    application: CalibrationApplication,
    assets: Path,
    port: int,
    operation_lock: contextlib.AbstractContextManager[object],
) -> None:
    if not (assets / "index.html").is_file():
        raise ValueError(
            "Browser assets are missing. Build this package's web application before building/installing its wheel, or pass --assets."
        )
    token = secrets.token_urlsafe(32)
    with (
        operation_lock,
        contextlib.closing(application),
        CalibrationHTTPServer(
            ("127.0.0.1", port), request_handler(application, assets.resolve(), token)
        ) as server,
    ):
        print(
            f"SO-101 calibration: http://127.0.0.1:{server.server_port}/",
            flush=True,
        )
        with contextlib.suppress(KeyboardInterrupt):
            try:
                server.serve_forever()
            finally:
                application.stream_stop.set()
                with application.stream_condition:
                    application.stream_condition.notify_all()


def command_line() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-directory", type=Path, default=setup_directory())
    parser.add_argument(
        "--model-variant",
        choices=("stock", "soft-gripper-camera"),
        default="stock",
        help="Model for new arm entries; existing setup files keep their model.",
    )
    parser.add_argument("--serial", help="USB serial of the arm whose motor ID order is being set")
    parser.add_argument(
        "--motor-id-order",
        choices=("ascending", "descending"),
        help="Motor IDs from base to gripper for the selected serial. Saved after calibration.",
    )
    parser.add_argument("--assets", type=Path, default=asset_directory())
    parser.add_argument("--port", type=int, default=8795)
    arguments = parser.parse_args()
    if (arguments.serial is None) != (arguments.motor_id_order is None):
        parser.error("--serial and --motor-id-order must be used together")
    motor_id_order_override = (
        (arguments.serial, arguments.motor_id_order) if arguments.serial is not None else None
    )
    application = CalibrationApplication(
        arguments.setup_directory,
        calibration_root(),
        NativeSetupStore(arguments.model_variant),
        motor_id_order_override=motor_id_order_override,
    )
    serve_application(application, arguments.assets, arguments.port, setup_lock())


if __name__ == "__main__":
    command_line()
