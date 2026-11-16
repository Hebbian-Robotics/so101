"""Local calibration HTTP adapter. The browser has no motor-motion endpoint."""

import argparse
import contextlib
import json
import mimetypes
import os
import secrets
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Annotated, Literal
from urllib.parse import unquote, urlsplit

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter

from .calibration import (
    REFERENCE_GRIPPER_RADIANS,
    REFERENCE_JOINTS,
    CalibrationSession,
    JointView,
    Reference,
)
from .configuration import ModelVariant
from .device import RegisterBus
from .discovery import ControllerChoice, discover_controllers
from .runtime import asset_directory, setup_directory, setup_lock
from .storage import NativeSetupStore, SetupStore


class ActionModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ConnectAction(ActionModel):
    action: Literal["connect"]
    serial: str = Field(pattern=r"^[A-Za-z0-9_-]+$")


class SimpleAction(ActionModel):
    action: Literal["capture_reference", "finish_sweep", "disconnect", "restart"]


class ReverseAction(ActionModel):
    action: Literal["reverse_joint"]
    index: int = Field(ge=0, le=5)


class GripperEndpointAction(ActionModel):
    action: Literal["capture_gripper_endpoint"]
    endpoint: Literal["closed", "open"]


class SaveAction(ActionModel):
    action: Literal["save"]
    model_verified: Literal[True]


Action = Annotated[
    ConnectAction | SimpleAction | ReverseAction | GripperEndpointAction | SaveAction,
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
    gripper_opening: float = 0.5
    reference_joints_radians: list[float] = Field(default_factory=lambda: list(REFERENCE_JOINTS))
    reference_gripper_radians: float = REFERENCE_GRIPPER_RADIANS
    message: str = ""
    command: str = ""


class CalibrationApplication:
    def __init__(
        self,
        config_directory: Path,
        calibration_root: Path,
        store: SetupStore | None = None,
        saved_command: Callable[[str], str] | None = None,
    ) -> None:
        self.config_directory = config_directory.resolve()
        self.store = NativeSetupStore() if store is None else store
        self.saved_command = saved_command
        self.calibration_root = calibration_root
        self.session: CalibrationSession | None = None
        self.auto_connect = True
        self.lock = threading.RLock()

    def controllers(self) -> list[ControllerChoice]:
        return discover_controllers(self.config_directory, self.store)

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
        bus = RegisterBus(controller.port)
        try:
            self.session = CalibrationSession(
                setup, directory, self.calibration_root, bus, self.store
            )
        except Exception:
            bus.close()
            raise

    def view(self) -> ApplicationView:
        with self.lock:
            if self.session is None:
                controllers = self.controllers()
                registered = [controller for controller in controllers if controller.registered]
                if self.auto_connect and len(registered) == 1:
                    self.connect(registered[0].serial)
                else:
                    return ApplicationView(
                        stage="disconnected",
                        controllers=controllers,
                        message="Plug in an arm, or select its USB controller below. New controllers need explicit selection.",
                    )
            assert self.session is not None
            view = ApplicationView.model_validate(self.session.view().model_dump())
            view.model_variant = self.session.setup.model_variant
            if view.stage == "saved" and self.saved_command is not None:
                view.command = self.saved_command(view.serial)
            return view

    def act(self, action: Action) -> ApplicationView:
        with self.lock:
            if isinstance(action, ConnectAction):
                self.connect(action.serial)
            elif isinstance(action, SimpleAction) and action.action == "disconnect":
                self.close()
                self.auto_connect = False
            else:
                if self.session is None:
                    raise ValueError("Connect an arm first")
                match action:
                    case SimpleAction(action="restart"):
                        if self.session.view().stage in {"saved", "error"}:
                            raise ValueError("Disconnect and reconnect before creating a new draft")
                        self.session.state = Reference()
                    case SimpleAction(action="capture_reference"):
                        self.session.capture_reference()
                    case SimpleAction(action="finish_sweep"):
                        self.session.finish_sweep()
                    case ReverseAction(index=index):
                        self.session.reverse_joint(index)
                    case GripperEndpointAction(endpoint=endpoint):
                        self.session.capture_gripper_endpoint(endpoint)
                    case SaveAction(model_verified=verified):
                        self.session.save(model_verified=verified)
            return self.view()

    def close(self) -> None:
        if self.session is not None:
            self.session.bus.close()
            self.session = None


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


def request_handler(
    application: CalibrationApplication, asset_directory: Path, token: str
) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def setup(self) -> None:
            super().setup()
            self.connection.settimeout(5)

        def reply(
            self,
            status: HTTPStatus,
            content: bytes,
            content_type: str = "application/json",
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:",
            )
            self.end_headers()
            self.wfile.write(content)

        def authenticated(self) -> bool:
            provided = self.headers.get("Authorization", "")
            return secrets.compare_digest(provided, f"Bearer {token}")

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/api/state":
                if not self.authenticated():
                    self.reply(
                        HTTPStatus.UNAUTHORIZED,
                        b'{"message":"Open the URL printed by the calibration command."}',
                    )
                    return
                try:
                    self.reply(HTTPStatus.OK, application.view().model_dump_json().encode())
                except Exception as error:
                    self.reply(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        json.dumps({"message": str(error)}).encode(),
                    )
                return
            requested = (asset_directory / unquote(path).lstrip("/")).resolve()
            if path == "/":
                requested = asset_directory / "index.html"
            if not requested.is_relative_to(asset_directory) or not requested.is_file():
                self.reply(HTTPStatus.NOT_FOUND, b"Not found", "text/plain")
                return
            self.reply(
                HTTPStatus.OK,
                requested.read_bytes(),
                mimetypes.guess_type(requested.name)[0] or "application/octet-stream",
            )

        def do_POST(self) -> None:
            if urlsplit(self.path).path != "/api/action":
                self.reply(HTTPStatus.NOT_FOUND, b"{}")
                return
            if not self.authenticated():
                self.reply(HTTPStatus.UNAUTHORIZED, b"{}")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("Action body must be 1-4096 bytes")
                action = ACTION_ADAPTER.validate_json(self.rfile.read(length))
                self.reply(HTTPStatus.OK, application.act(action).model_dump_json().encode())
            except Exception as error:
                self.reply(HTTPStatus.BAD_REQUEST, json.dumps({"message": str(error)}).encode())

        def log_message(self, format: str, *args: object) -> None:
            pass

    return Handler


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
            f"SO-101 calibration: http://127.0.0.1:{server.server_port}/#{token}",
            flush=True,
        )
        with contextlib.suppress(KeyboardInterrupt):
            server.serve_forever()


def command_line() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-directory", type=Path, default=setup_directory())
    parser.add_argument("--assets", type=Path, default=asset_directory())
    parser.add_argument("--port", type=int, default=8795)
    arguments = parser.parse_args()
    application = CalibrationApplication(arguments.setup_directory, calibration_root())
    serve_application(application, arguments.assets, arguments.port, setup_lock())


if __name__ == "__main__":
    command_line()
