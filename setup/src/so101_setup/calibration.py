"""SO-101 calibration drafts. Physical motion stays with the operator.

Reference capture and range recording change only the draft. Motor registers
are written only by save, with torque off and backups already on disk.
"""

import json
import math
import os
import tempfile
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from .configuration import MOTOR_IDS, SO101_JOINT_NAMES, ArmSetup
from .device import RegisterBus, apply_position_gains
from .storage import NativeSetupStore, SetupStore

MOTOR_NAMES = (*SO101_JOINT_NAMES, "gripper")
RESOLUTION = 4096
HOMING_STEPS = 2047
REFERENCE_JOINTS = [0.0] * 5
REFERENCE_GRIPPER_RADIANS = 0.75
CALIBRATION_REGISTERS = (
    "homing_offset",
    "min_position_limit",
    "max_position_limit",
    "operating_mode",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "return_delay_time",
    "lock",
)


class MotorCalibration(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    id: int = Field(ge=1, le=6)
    drive_mode: Literal[0] = 0
    homing_offset: int = Field(ge=-2047, le=2047)
    range_min: int = Field(ge=0, lt=RESOLUTION)
    range_max: int = Field(ge=0, lt=RESOLUTION)

    @model_validator(mode="after")
    def require_nonempty_range(self) -> "MotorCalibration":
        if self.range_min >= self.range_max:
            raise ValueError("Each motor needs a nonempty increasing range")
        return self


Calibration = dict[str, MotorCalibration]
CALIBRATION_ADAPTER = TypeAdapter(Calibration)


def read_calibration(path: Path) -> Calibration:
    calibration = CALIBRATION_ADAPTER.validate_json(path.read_bytes())
    if set(calibration) != set(MOTOR_NAMES) or any(
        calibration[name].id != motor_id
        for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS, strict=True)
    ):
        raise ValueError("Calibration must name the six SO-101 motors with IDs 1-6")
    return calibration


def joint_radians(steps: int, sign: int, offset_radians: float) -> float:
    return sign * (steps - HOMING_STEPS) * math.tau / RESOLUTION + offset_radians


def rehomed_steps(position: int, previous_homing: int, new_homing: int) -> int:
    # Feetech reports (encoder - homing_offset) modulo the encoder resolution.
    return (position + previous_homing - new_homing) % RESOLUTION


@dataclass
class DraftCalibration:
    homing_offsets: list[int]
    reference_steps: list[int]
    minimums: list[int]
    maximums: list[int]
    signs: list[int]
    offsets: list[float]
    previous_steps: list[int] = field(init=False)

    def __post_init__(self) -> None:
        self.previous_steps = list(self.reference_steps)

    def positions(self, positions: list[int], old_homing: list[int]) -> list[int]:
        return [
            rehomed_steps(position, previous, new)
            for position, previous, new in zip(
                positions, old_homing, self.homing_offsets, strict=True
            )
        ]

    def record(self, positions: list[int]) -> None:
        if any(
            abs(position - previous) > RESOLUTION // 2
            for index, (position, previous) in enumerate(
                zip(positions, self.previous_steps, strict=True)
            )
            if index != 4
        ):
            raise ValueError(
                "A joint crossed the encoder boundary. Return to the middle reference pose and capture again before sweeping."
            )
        self.previous_steps = list(positions)
        for index, position in enumerate(positions):
            self.minimums[index] = min(self.minimums[index], position)
            self.maximums[index] = max(self.maximums[index], position)

    def entries(self) -> Calibration:
        return {
            name: MotorCalibration(
                id=motor_id,
                homing_offset=self.homing_offsets[index],
                range_min=0 if name == "wrist_roll" else self.minimums[index],
                range_max=4095 if name == "wrist_roll" else self.maximums[index],
            )
            for index, (name, motor_id) in enumerate(zip(MOTOR_NAMES, MOTOR_IDS, strict=True))
        }


@dataclass(frozen=True)
class Reference:
    message: str = ""


@dataclass
class Recording:
    draft: DraftCalibration
    closed_gripper_steps: int | None = None
    open_gripper_steps: int | None = None


@dataclass
class Review:
    draft: DraftCalibration
    gripper_endpoints: tuple[float, float]


@dataclass(frozen=True)
class Saved:
    configuration: Path
    calibration: Path
    backup: Path


@dataclass(frozen=True)
class Failed:
    message: str


CalibrationState = Reference | Recording | Review | Saved | Failed


class JointView(BaseModel):
    name: str
    position_radians: float
    range_min: int
    range_max: int
    sign: int


class CalibrationView(BaseModel):
    stage: Literal["reference", "sweep", "review", "saved", "error"]
    serial: str
    joints: list[JointView]
    gripper_opening: float
    gripper_closed_recorded: bool = False
    gripper_open_recorded: bool = False
    message: str = ""
    command: str = ""


def atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(dir=path.parent, prefix=".calibration-")
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass
class CalibrationSession:
    setup: ArmSetup
    directory: Path
    calibration_root: Path
    bus: RegisterBus
    store: SetupStore = field(default_factory=NativeSetupStore)
    state: CalibrationState = field(default_factory=Reference)
    original_homing: list[int] = field(init=False)
    last_view: CalibrationView = field(init=False)

    @property
    def serial(self) -> str:
        return self.setup.serial

    def __post_init__(self) -> None:
        if self.bus.driven_motor_ids != MOTOR_IDS or self.bus.all_motor_ids != MOTOR_IDS:
            raise ValueError("Calibrator requires the standard SO-101 motor layout (IDs 1-6)")
        self.require_torque_off()
        self.original_homing = self.bus.read_registers("homing_offset")
        self.state = self.initial_state()
        self.last_view = self.view()

    def initial_state(self) -> Reference | Review:
        calibration_path = self.directory / "calibration.json"
        if not calibration_path.exists():
            return Reference()
        try:
            entries = read_calibration(calibration_path)
        except (ValueError, OSError) as error:
            return Reference(f"Saved calibration could not be reused: {error}")
        minimums = self.bus.read_registers("min_position_limit")
        maximums = self.bus.read_registers("max_position_limit")
        if not all(
            (
                entries[name].homing_offset,
                entries[name].range_min,
                entries[name].range_max,
            )
            == (self.original_homing[index], minimums[index], maximums[index])
            for index, name in enumerate(MOTOR_NAMES)
        ):
            return Reference(
                "Saved calibration differs from the motor registers. Capture a new reference to recalibrate this arm."
            )
        settings = self.setup.alignment
        gripper = self.setup.gripper
        return Review(
            DraftCalibration(
                homing_offsets=list(self.original_homing),
                reference_steps=[HOMING_STEPS] * 6,
                minimums=minimums,
                maximums=maximums,
                signs=[*settings.joint_signs, settings.gripper_sign],
                offsets=[
                    *settings.joint_offsets_radians,
                    settings.gripper_offset_radians,
                ],
            ),
            (gripper.closed_position_radians, gripper.open_position_radians),
        )

    def require_torque_off(self) -> None:
        if any(self.bus.read_registers("torque_enable", self.bus.all_motor_ids)):
            raise RuntimeError("Support and disable the arm before calibration; torque is enabled")

    def read_positions(self) -> list[int]:
        self.require_torque_off()
        positions = self.bus.read_registers("present_position")
        if len(positions) != 6 or any(not 0 <= position < RESOLUTION for position in positions):
            raise RuntimeError("Expected six valid SO-101 encoder readings")
        return positions

    def capture_reference(self) -> None:
        if isinstance(self.state, (Saved, Failed)):
            raise ValueError("Restart calibration to create another draft")
        positions = self.read_positions()
        raw_positions = [
            (position + previous) % RESOLUTION
            for position, previous in zip(positions, self.original_homing, strict=True)
        ]
        homing_offsets = [
            max(-2047, min(2047, position - HOMING_STEPS)) for position in raw_positions
        ]
        reference_steps = [
            rehomed_steps(position, old, new)
            for position, old, new in zip(
                positions, self.original_homing, homing_offsets, strict=True
            )
        ]
        settings = self.setup.alignment
        signs: list[int] = [*settings.joint_signs, settings.gripper_sign]
        targets = [*REFERENCE_JOINTS, REFERENCE_GRIPPER_RADIANS]
        offsets = [
            target - joint_radians(position, sign, 0.0)
            for target, position, sign in zip(targets, reference_steps, signs, strict=True)
        ]
        self.state = Recording(
            DraftCalibration(
                homing_offsets,
                reference_steps,
                list(reference_steps),
                list(reference_steps),
                signs,
                offsets,
            )
        )

    def reverse_joint(self, index: int) -> None:
        if not isinstance(self.state, Recording) or not 0 <= index < 6:
            raise ValueError("Directions can be changed only while recording a new calibration")
        draft = self.state.draft
        target = REFERENCE_JOINTS[index] if index < 5 else REFERENCE_GRIPPER_RADIANS
        draft.signs[index] *= -1
        draft.offsets[index] = target - joint_radians(
            draft.reference_steps[index], draft.signs[index], 0.0
        )

    def capture_gripper_endpoint(self, endpoint: Literal["closed", "open"]) -> None:
        if not isinstance(self.state, Recording):
            raise ValueError("Record gripper endpoints during the travel sweep")
        positions = self.state.draft.positions(self.read_positions(), self.original_homing)
        self.state.draft.record(positions)
        steps = positions[5]
        if endpoint == "closed":
            self.state.closed_gripper_steps = steps
        else:
            self.state.open_gripper_steps = steps

    def finish_sweep(self) -> None:
        if not isinstance(self.state, Recording):
            raise ValueError("Capture a reference pose before recording travel")
        self.view()
        # MotorCalibration checks that every swept joint/gripper has moved.
        self.state.draft.entries()
        draft = self.state.draft
        closed_steps = self.state.closed_gripper_steps
        open_steps = self.state.open_gripper_steps
        if closed_steps is None or open_steps is None or closed_steps == open_steps:
            raise ValueError("Record distinct closed and open gripper positions before review")
        endpoints = (
            joint_radians(closed_steps, draft.signs[5], draft.offsets[5]),
            joint_radians(open_steps, draft.signs[5], draft.offsets[5]),
        )
        self.state = Review(draft, endpoints)

    def view(self) -> CalibrationView:
        if isinstance(self.state, (Saved, Failed)):
            return self.last_view.model_copy(
                update={
                    "stage": "saved" if isinstance(self.state, Saved) else "error",
                    "message": f"Saved. Backups: {self.state.backup}"
                    if isinstance(self.state, Saved)
                    else self.state.message,
                    "command": "",
                }
            )
        positions = self.read_positions()
        settings = self.setup.alignment
        if isinstance(self.state, (Recording, Review)):
            draft = self.state.draft
            positions = draft.positions(positions, self.original_homing)
            if isinstance(self.state, Recording):
                draft.record(positions)
            signs, offsets = draft.signs, draft.offsets
            minimums, maximums = draft.minimums, draft.maximums
            stage = "sweep" if isinstance(self.state, Recording) else "review"
        else:
            signs: list[int] = [*settings.joint_signs, settings.gripper_sign]
            offsets = [*settings.joint_offsets_radians, settings.gripper_offset_radians]
            minimums, maximums = positions, positions
            stage = "reference"
        angles = [
            joint_radians(position, sign, offset)
            for position, sign, offset in zip(positions, signs, offsets, strict=True)
        ]
        closed = joint_radians(minimums[5], signs[5], offsets[5])
        opened = joint_radians(maximums[5], signs[5], offsets[5])
        if isinstance(self.state, Review):
            closed, opened = self.state.gripper_endpoints
        opening = (angles[5] - closed) / (opened - closed) if opened != closed else 0.5
        view = CalibrationView(
            stage=stage,
            serial=self.serial,
            message=self.state.message if isinstance(self.state, Reference) else "",
            joints=[
                JointView(
                    name=name,
                    position_radians=angles[index],
                    range_min=minimums[index],
                    range_max=maximums[index],
                    sign=signs[index],
                )
                for index, name in enumerate(MOTOR_NAMES)
            ],
            gripper_opening=max(0.0, min(1.0, opening)),
            gripper_closed_recorded=isinstance(self.state, Review)
            or (isinstance(self.state, Recording) and self.state.closed_gripper_steps is not None),
            gripper_open_recorded=isinstance(self.state, Review)
            or (isinstance(self.state, Recording) and self.state.open_gripper_steps is not None),
        )
        self.last_view = view
        return view

    def save(self, *, model_verified: bool) -> None:
        if not isinstance(self.state, Review) or not model_verified:
            raise ValueError("Review the live model and confirm it matches before saving")
        self.read_positions()
        draft = self.state.draft
        entries = draft.entries()
        calibration_bytes = CALIBRATION_ADAPTER.dump_json(entries, indent=2)
        configuration_path = self.store.configuration_path(self.directory)
        result = ArmSetup.model_validate(
            {
                "serial": self.serial,
                "alignment": {
                    "joint_signs": draft.signs[:5],
                    "joint_offsets_radians": draft.offsets[:5],
                    "gripper_sign": draft.signs[5],
                    "gripper_offset_radians": draft.offsets[5],
                },
                "gripper": {
                    "closed_position_radians": self.state.gripper_endpoints[0],
                    "open_position_radians": self.state.gripper_endpoints[1],
                },
                "servos": self.setup.servos.model_dump(),
            }
        )
        configuration_bytes = self.store.encode(self.directory, result)
        live_path = self.calibration_root / "robots/so_follower" / f"so101_{self.serial}.json"
        backup = self.directory / "backups" / datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
        backup.mkdir(parents=True)
        for source, filename in [
            (configuration_path, configuration_path.name),
            (self.directory / "calibration.json", "calibration.json"),
            (live_path, "live-calibration.json"),
        ]:
            if source.exists():
                atomic_write(backup / filename, source.read_bytes())
        registers = {
            register: self.bus.read_registers(register) for register in CALIBRATION_REGISTERS
        }
        atomic_write(backup / "registers.json", json.dumps(registers, indent=2).encode())
        try:
            self.require_torque_off()
            updates = {
                "homing_offset": [entries[name].homing_offset for name in MOTOR_NAMES],
                "min_position_limit": [entries[name].range_min for name in MOTOR_NAMES],
                "max_position_limit": [entries[name].range_max for name in MOTOR_NAMES],
                "operating_mode": [self.bus.definition.operating_modes["position"]] * 6,
            }
            if self.setup.servos.expected_return_delay_time is not None:
                updates["return_delay_time"] = [self.setup.servos.expected_return_delay_time] * 6
            for motor_id, locked in zip(MOTOR_IDS, registers["lock"], strict=True):
                if locked:
                    self.bus.write_acknowledged(motor_id, "lock", 0)
            for register, desired in updates.items():
                for motor_id, current, value in zip(
                    MOTOR_IDS, registers[register], desired, strict=True
                ):
                    if current != value:
                        self.bus.write_acknowledged(motor_id, register, value)
                if self.bus.read_registers(register) != desired:
                    raise RuntimeError(f"Calibration register verification failed: {register}")
            apply_position_gains(self.bus, self.setup.servos)
            self.require_torque_off()
            atomic_write(self.directory / "calibration.json", calibration_bytes)
            atomic_write(configuration_path, configuration_bytes)
            atomic_write(live_path, calibration_bytes)
        except Exception as error:
            self.state = Failed(
                f"Save failed: {error}. Backups and previous registers: {backup}. Restart calibration to inspect the arm before continuing."
            )
            self.bus.close()
            raise RuntimeError(self.state.message) from error
        self.state = Saved(configuration_path, live_path, backup)
        self.bus.close()
