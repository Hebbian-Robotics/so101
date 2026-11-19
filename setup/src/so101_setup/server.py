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
    RESOLUTION,
    CalibrationSession,
    Failed,
    JointView,
    Reference,
    Saved,
    SweepRecording,
    remap_full_sweep,
)
from .configuration import (
    DEFAULT_NEW_ARM_MODEL,
    MOTOR_IDS,
    ArmSetup,
    ModelVariant,
    MotorIds,
    motor_id_mapping,
)
from .device import RegisterBus, RegisterReadError
from .discovery import ControllerChoice, discover_controllers
from .motor_ids import FOLLOWER_MOTORS, program_motor_id, programming_ports
from .restore import restore_parameters
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
    action: Literal[
        "finish_sweep",
        "disconnect",
        "restart",
        "reopen",
        "reset_servos",
        "skip_servo_reset",
        "check_motor_order",
        "apply_detected_motor_order",
    ]


class MapMotorIdsAction(ActionModel):
    action: Literal["map_motor_ids"]
    motor_ids: MotorIds


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


class SaveAction(ActionModel):
    action: Literal["save"]
    model_verified: Literal[True]


Action = Annotated[
    ConnectAction
    | SimpleAction
    | MapMotorIdsAction
    | StartSweepAction
    | ReverseAction
    | AdjustAction
    | SaveAction,
    Field(discriminator="action"),
]
ACTION_ADAPTER = TypeAdapter(Action)


class ApplicationView(BaseModel):
    model_config = ConfigDict(extra="forbid")
    stage: Literal["disconnected", "reference", "sweep", "review", "saved", "error"]
    controllers: list[ControllerChoice] = Field(default_factory=list)
    serial: str = ""
    model_variant: ModelVariant = DEFAULT_NEW_ARM_MODEL
    joints: list[JointView] = Field(default_factory=list)
    estimated: bool = False
    can_recalibrate_joint: bool = False
    offer_servo_reset: bool = False
    motor_ids: dict[str, int] = Field(default_factory=lambda: motor_id_mapping(MOTOR_IDS))
    checking_motor_order: bool = False
    detected_base_motor: Literal[1, 6] | None = None
    servo_reset_incomplete: bool = False
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
    ) -> None:
        self.config_directory = config_directory.resolve()
        self.store = NativeSetupStore() if store is None else store
        self.saved_command = saved_command
        self.calibration_root = calibration_root
        self.session: CalibrationSession | None = None
        self.auto_connect = True
        self.servo_reset_decided = False
        self.motor_order_sample: list[int] | None = None
        self.detected_base_motor: Literal[1, 6] | None = None
        self.incomplete_servo_resets: set[str] = set()
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
        return discover_controllers(self.config_directory, self.store)

    @staticmethod
    def read_calibration_registers(bus: RegisterBus) -> dict[str, list[int]]:
        return {
            name: bus.read_registers(name)
            for name in ("id", "homing_offset", "min_position_limit", "max_position_limit")
        }

    def connect(self, serial: str, motor_ids: dict[str, int] | None = None) -> None:
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
        if motor_ids is not None:
            setup = self.with_motor_ids(setup, motor_ids)
        bus = RegisterBus(controller.port, setup.motor_ids_in_joint_order())
        try:
            session = CalibrationSession(setup, directory, self.calibration_root, bus, self.store)
            self.calibration_registers = self.read_calibration_registers(bus)
            self.session = session
            self.servo_reset_decided = False
            self.motor_order_sample = None
            self.detected_base_motor = None
        except Exception:
            bus.close()
            raise

    @staticmethod
    def with_motor_ids(setup: ArmSetup, motor_ids: dict[str, int]) -> ArmSetup:
        remapped = ArmSetup.model_validate(setup.model_dump() | {"motor_ids": motor_ids})
        old_gains = setup.servos.expected_position_gains_by_motor
        gains = {
            remapped.motor_ids[joint]: old_gains[old_id]
            for joint, old_id in setup.motor_ids.items()
            if old_id in old_gains
        }
        return remapped.model_copy(
            update={
                "servos": setup.servos.model_copy(
                    update={"expected_position_gains_by_motor": gains}
                )
            }
        )

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
                    self.motor_order_sample = None
                    self.detected_base_motor = None
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
            view.servo_reset_incomplete = session.serial in self.incomplete_servo_resets
            view.offer_servo_reset = view.stage == "reference" and not self.servo_reset_decided
            view.motor_ids = session.setup.motor_ids
            view.checking_motor_order = self.motor_order_sample is not None
            view.detected_base_motor = self.detected_base_motor
            view.model_variant = self.session.setup.model_variant
            if view.stage == "saved" and self.saved_command is not None:
                view.command = self.saved_command(view.serial)
            return view

    def reset_servos(self, session: CalibrationSession) -> None:
        serial = session.serial
        controller = next((item for item in self.controllers() if item.serial == serial), None)
        if controller is None:
            raise ValueError("Reconnect this arm before resetting its servos")
        session.require_torque_off()
        self.close_session()
        # Reopen the exact board afterward, even when another controller is present.
        self.auto_connect = False
        self.incomplete_servo_resets.add(serial)
        try:
            for motor_id in session.setup.motor_ids_in_joint_order():
                try:
                    restore_parameters(
                        controller.port, motor_id, session.directory / "backups" / "servo-restore"
                    )
                except Exception as error:
                    raise RuntimeError(f"Reset stopped on motor {motor_id}: {error}") from error
        finally:
            self.connect(serial, session.setup.motor_ids)
            if self.session is not None:
                self.session.restart()
        self.incomplete_servo_resets.discard(serial)
        self.servo_reset_decided = True

    def handle_reset_choice(self, session: CalibrationSession, action: Action) -> bool:
        action_name = action.action
        choosing_reset = action_name in {"reset_servos", "skip_servo_reset"}
        awaiting_choice = isinstance(session.state, Reference) and not self.servo_reset_decided
        if not choosing_reset:
            if awaiting_choice and action_name == "start_sweep":
                raise ValueError("Choose whether to reset servos before the full sweep")
            return False
        if not awaiting_choice:
            raise ValueError("Choose whether to reset servos before the full sweep")
        if action_name == "reset_servos":
            self.reset_servos(session)
        elif session.serial in self.incomplete_servo_resets:
            raise ValueError("An incomplete reset must be retried before calibration")
        else:
            self.servo_reset_decided = True
        return True

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
                if self.handle_reset_choice(self.session, action):
                    return self.view()
                if isinstance(action, SimpleAction) and action.action == "check_motor_order":
                    self.check_motor_order(self.session)
                elif (
                    isinstance(action, SimpleAction)
                    and action.action == "apply_detected_motor_order"
                ):
                    self.apply_detected_motor_order(self.session)
                else:
                    self.apply_session_action(self.session, action)
            return self.view()

    def check_motor_order(self, session: CalibrationSession) -> None:
        self.require_full_sweep(session)
        positions = session.read_positions()
        if self.motor_order_sample is None:
            self.motor_order_sample = positions
            self.detected_base_motor = None
            return
        moved_motors = {
            motor_id
            for motor_id, before, after in zip(
                session.setup.motor_ids_in_joint_order(),
                self.motor_order_sample,
                positions,
                strict=True,
            )
            if min(delta := abs(after - before), RESOLUTION - delta) >= 16
        }
        self.motor_order_sample = None
        if moved_motors == {1}:
            self.detected_base_motor = 1
        elif moved_motors == {6}:
            self.detected_base_motor = 6
        else:
            raise ValueError("Motor order is unclear. Try again and move only the base joint.")

    def apply_detected_motor_order(self, session: CalibrationSession) -> None:
        if self.detected_base_motor is None:
            raise ValueError("Check which motor moves the base first")
        ids = MOTOR_IDS if self.detected_base_motor == 1 else MOTOR_IDS[::-1]
        self.apply_motor_mapping(session, motor_id_mapping(ids))

    @staticmethod
    def require_full_sweep(session: CalibrationSession) -> SweepRecording:
        state = session.state
        if not isinstance(state, SweepRecording) or state.recalibration is not None:
            raise ValueError("Motor mapping is available during a full sweep")
        return state

    def apply_motor_mapping(self, session: CalibrationSession, motor_ids: dict[str, int]) -> None:
        state = self.require_full_sweep(session)
        self.with_motor_ids(session.setup, motor_ids)
        session.require_torque_off()
        if motor_ids == session.setup.motor_ids:
            self.detected_base_motor = None
            return
        reset_decided = self.servo_reset_decided
        old_ids = session.setup.motor_ids_in_joint_order()
        old_registers = self.calibration_registers
        self.close_session()
        self.auto_connect = False
        self.connect(session.serial, motor_ids)
        assert self.session is not None
        self.session.restart()
        new_ids = self.session.setup.motor_ids_in_joint_order()
        if any(
            dict(zip(old_ids, values, strict=True))
            != dict(zip(new_ids, self.calibration_registers[name], strict=True))
            for name, values in old_registers.items()
        ):
            raise ValueError("Motor registers changed. Start a fresh sweep before saving.")
        self.session.state = remap_full_sweep(state, old_ids, self.session.setup)
        self.servo_reset_decided = reset_decided

    def apply_session_action(self, session: CalibrationSession, action: Action) -> None:
        match action:
            case MapMotorIdsAction(motor_ids=motor_ids):
                self.apply_motor_mapping(session, motor_ids)
            case SimpleAction(action="reopen"):
                if session.view().stage != "saved":
                    raise ValueError("Reopen is available after saving")
                serial = session.serial
                self.session = None
                self.connect(serial)
            case SimpleAction(action="restart"):
                session.restart()
                self.servo_reset_decided = False
            case StartSweepAction(joint_index=joint_index):
                session.start_sweep(joint_index)
            case SimpleAction(action="finish_sweep"):
                session.finish_sweep()
            case ReverseAction(index=index):
                session.reverse_joint(index)
            case AdjustAction(index=index, delta_radians=delta):
                session.adjust_joint(index, delta)
            case SaveAction(model_verified=verified):
                session.save(model_verified=verified)

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
        default=DEFAULT_NEW_ARM_MODEL,
        help="Model for new arms (default: soft-gripper-camera). Saved arms keep their model.",
    )
    parser.add_argument("--assets", type=Path, default=asset_directory())
    parser.add_argument("--port", type=int, default=8795)
    arguments = parser.parse_args()
    application = CalibrationApplication(
        arguments.setup_directory,
        calibration_root(),
        NativeSetupStore(arguments.model_variant),
    )
    serve_application(application, arguments.assets, arguments.port, setup_lock())


if __name__ == "__main__":
    command_line()
