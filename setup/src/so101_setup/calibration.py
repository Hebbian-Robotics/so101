"""SO-101 calibration drafts. Physical motion stays with the operator.

Reference capture and range recording change only the draft. Motor registers
are written only by save, with torque off and backups already on disk.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from .configuration import MOTOR_IDS, SO101_JOINT_NAMES, ArmSetup
from .device import RegisterBus, apply_position_gains
from .storage import NativeSetupStore, SetupStore
from .sweep_estimation import EncoderSweep, estimate_sweep

MOTOR_NAMES = (*SO101_JOINT_NAMES, "gripper")
RESOLUTION = 4096
HOMING_STEPS = 2047
HOME_JOINTS = [-0.0037, -1.8435, 1.5765, 1.3296, -1.5536]
HOME_GRIPPER_RADIANS = -0.12105
# Generous lower bounds based on the observed soft-gripper SO-101 assembly.
# These detect a missed sweep, not physical or commanded joint limits.
MINIMUM_SWEEP_DEGREES = (130, 130, 120, 130, 220, 75)
CALIBRATION_REGISTERS = (
    "homing_offset",
    "min_position_limit",
    "max_position_limit",
    "operating_mode",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "return_delay_time",
    "maximum_acceleration",
    "max_torque_limit",
    "protection_current",
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
    def require_nonempty_range(self) -> MotorCalibration:
        if self.range_min >= self.range_max:
            raise ValueError("Each motor needs a nonempty increasing range")
        return self


Calibration = dict[str, MotorCalibration]
CALIBRATION_ADAPTER = TypeAdapter(Calibration)


def read_calibration(path: Path, motor_ids: list[int] = MOTOR_IDS) -> Calibration:
    calibration = CALIBRATION_ADAPTER.validate_json(path.read_bytes())
    if set(calibration) != set(MOTOR_NAMES) or any(
        calibration[name].id != motor_id
        for name, motor_id in zip(MOTOR_NAMES, motor_ids, strict=True)
    ):
        raise ValueError(
            "Calibration joint IDs do not match the selected motor ID order. Capture a new calibration."
        )
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

    def positions(self, positions: list[int], old_homing: list[int]) -> list[int]:
        return [
            rehomed_steps(position, previous, new)
            for position, previous, new in zip(
                positions, old_homing, self.homing_offsets, strict=True
            )
        ]

    def entries(self, motor_ids: list[int] = MOTOR_IDS) -> Calibration:
        return {
            name: MotorCalibration(
                id=motor_id,
                homing_offset=self.homing_offsets[index],
                range_min=self.minimums[index],
                range_max=self.maximums[index],
            )
            for index, (name, motor_id) in enumerate(zip(MOTOR_NAMES, motor_ids, strict=True))
        }


@dataclass(frozen=True)
class Reference:
    message: str = ""


@dataclass
class SweepRecording:
    sweeps: dict[int, EncoderSweep]
    signs: list[int]
    home_positions: list[int]
    recalibration: JointRecalibration | None = None


def remap_full_sweep(
    recording: SweepRecording,
    old_motor_ids: list[int],
    setup: ArmSetup,
) -> SweepRecording:
    """Keep encoder data with its physical motor and alignment with its joint."""
    old_indices = [old_motor_ids.index(motor_id) for motor_id in setup.motor_ids_in_joint_order()]
    return SweepRecording(
        {index: recording.sweeps[old] for index, old in enumerate(old_indices)},
        list(recording.signs),
        [recording.home_positions[old] for old in old_indices],
    )


@dataclass
class Review:
    draft: DraftCalibration
    gripper_endpoints: tuple[float, float]
    estimated: bool = False
    saved_baseline: bool = False
    recalibration: JointRecalibration | None = None


@dataclass(frozen=True)
class JointRecalibration:
    index: int
    baseline: Review


@dataclass(frozen=True)
class Saved:
    configuration: Path
    calibration: Path
    backup: Path


@dataclass(frozen=True)
class Failed:
    message: str


CalibrationState = Reference | SweepRecording | Review | Saved | Failed


class JointView(BaseModel):
    name: str
    position_radians: float
    range_min: int
    range_max: int
    sign: int
    moving: bool = False


class CalibrationView(BaseModel):
    stage: Literal["reference", "sweep", "review", "saved", "error"]
    serial: str
    joints: list[JointView]
    gripper_opening: float
    estimated: bool = False
    can_recalibrate_joint: bool = False
    recalibrating_joint: int | None = None
    message: str = ""
    command: str = ""
    configuration_path: str = ""
    calibration_path: str = ""


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
    previous_encoder_positions: list[int] | None = field(default=None, init=False)
    moving_until: list[float] = field(default_factory=lambda: [0.0] * 6, init=False)

    @property
    def serial(self) -> str:
        return self.setup.serial

    def __post_init__(self) -> None:
        expected_ids = self.setup.motor_ids_in_joint_order()
        if self.bus.driven_motor_ids != expected_ids or self.bus.all_motor_ids != expected_ids:
            raise ValueError("Calibration bus IDs must match the selected SO-101 motor order")
        self.require_torque_off()
        self.original_homing = self.bus.read_registers("homing_offset")
        self.state = self.initial_state()
        self.last_view = self.view()

    def initial_state(self) -> Reference | Review:
        calibration_path = self.directory / "calibration.json"
        if not calibration_path.exists():
            return Reference()
        try:
            entries = read_calibration(calibration_path, self.bus.driven_motor_ids)
        except (ValueError, OSError) as error:
            return Reference(f"Saved calibration could not be reused: {error}")
        minimums = self.bus.read_registers("min_position_limit")
        maximums = self.bus.read_registers("max_position_limit")
        if self.bus.read_registers("homing_offset") != self.original_homing or not all(
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
            saved_baseline=self.store.configuration_path(self.directory).is_file(),
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

    def restart(self) -> None:
        if isinstance(self.state, (Saved, Failed)):
            raise ValueError("Disconnect and reconnect before creating a new draft")
        if isinstance(self.state, (SweepRecording, Review)) and self.state.recalibration:
            self.state = self.initial_state()
        else:
            self.state = Reference()

    def start_sweep(self, joint_index: int | None = None) -> None:
        """Capture the supported home pose and start a read-only sweep."""
        if isinstance(self.state, (Saved, Failed)):
            raise ValueError("Reconnect before creating a new draft")
        recalibration = None
        if joint_index is None:
            indices = range(len(MOTOR_NAMES))
        else:
            if not 0 <= joint_index < len(MOTOR_NAMES):
                raise ValueError("Unknown joint")
            if not isinstance(self.state, Review) or not self.state.saved_baseline:
                raise ValueError("Single-joint recalibration requires a saved, matching arm setup")
            baseline = self.initial_state()
            if not isinstance(baseline, Review) or not baseline.saved_baseline:
                raise ValueError(
                    "Saved calibration no longer matches the arm. Reconnect to inspect it."
                )
            self.require_saved_setup()
            recalibration = JointRecalibration(joint_index, baseline)
            indices = range(joint_index, joint_index + 1)
        positions = self.read_positions()
        signs: list[int] = [*self.setup.alignment.joint_signs, self.setup.alignment.gripper_sign]
        self.state = SweepRecording(
            {index: EncoderSweep.start(positions[index]) for index in indices},
            signs,
            list(positions),
            recalibration,
        )

    def require_saved_setup(self) -> None:
        if (
            not self.store.configuration_path(self.directory).is_file()
            or self.store.load(self.directory, self.serial) != self.setup
        ):
            raise ValueError("Saved setup changed. Reconnect before recalibrating a joint.")

    def finish_sweep(self) -> None:
        if not isinstance(self.state, SweepRecording):
            raise ValueError("Start a sweep before estimating calibration")
        self.view()
        recording = self.state
        incomplete = [
            f"{MOTOR_NAMES[index].replace('_', ' ')}: {travel:.0f}° recorded (at least {minimum}° expected)"
            for index, sweep in recording.sweeps.items()
            if (travel := (sweep.maximum - sweep.minimum) * 360 / RESOLUTION)
            < (minimum := MINIMUM_SWEEP_DEGREES[index])
        ]
        if incomplete:
            raise ValueError(
                "Incomplete sweep — "
                + "; ".join(incomplete)
                + ". Gently sweep the listed joints further, then finish again; do not force the arm."
            )
        recalibration = recording.recalibration
        draft = (
            deepcopy(recalibration.baseline.draft)
            if recalibration
            else DraftCalibration(
                [0] * 6,
                [0] * 6,
                [0] * 6,
                [0] * 6,
                recording.signs,
                [0.0] * 6,
            )
        )
        reference_angles = [*HOME_JOINTS, HOME_GRIPPER_RADIANS]
        for index, sweep in recording.sweeps.items():
            sign = recording.signs[index]
            old_homing = self.original_homing[index]
            estimate = estimate_sweep(sweep, old_homing, sign, 0.0)
            home_steps = rehomed_steps(
                recording.home_positions[index], old_homing, estimate.homing_offset
            )
            draft.homing_offsets[index] = estimate.homing_offset
            draft.reference_steps[index] = (estimate.range_min + estimate.range_max) // 2
            draft.minimums[index] = estimate.range_min
            draft.maximums[index] = estimate.range_max
            draft.offsets[index] = reference_angles[index] - joint_radians(home_steps, sign, 0.0)
        draft.entries(self.bus.driven_motor_ids)
        if recalibration and recalibration.index != 5:
            endpoints = recalibration.baseline.gripper_endpoints
        else:
            gripper_angles = [
                joint_radians(position, draft.signs[5], draft.offsets[5])
                for position in (draft.minimums[5], draft.maximums[5])
            ]
            # The home sample was captured with the physical gripper closed.
            endpoints = tuple(
                sorted(gripper_angles, key=lambda angle: abs(angle - HOME_GRIPPER_RADIANS))
            )
            endpoints = (endpoints[0], endpoints[1])
        self.state = Review(draft, endpoints, estimated=True, recalibration=recalibration)

    def reverse_joint(self, index: int) -> None:
        if not 0 <= index < 6:
            raise ValueError("Unknown joint")
        if isinstance(self.state, Review):
            self.require_editable_joint(self.state, index)
            draft = self.state.draft
            center = draft.reference_steps[index]
            reference_angle = joint_radians(center, draft.signs[index], draft.offsets[index])
            draft.signs[index] *= -1
            draft.offsets[index] = reference_angle - joint_radians(center, draft.signs[index], 0.0)
            if index == 5:
                closed, opened = self.state.gripper_endpoints
                self.state.gripper_endpoints = (
                    2 * reference_angle - closed,
                    2 * reference_angle - opened,
                )
            return
        raise ValueError("Directions can be changed during review")

    def adjust_joint(self, index: int, delta_radians: float) -> None:
        if not isinstance(self.state, Review):
            raise ValueError("Joint alignment can be adjusted only during review")
        if not 0 <= index < 6 or not math.isfinite(delta_radians) or abs(delta_radians) > 0.1:
            raise ValueError("Invalid joint alignment adjustment")
        self.require_editable_joint(self.state, index)
        self.state.draft.offsets[index] += delta_radians
        if index == 5:
            closed, opened = self.state.gripper_endpoints
            self.state.gripper_endpoints = (closed + delta_radians, opened + delta_radians)

    def require_editable_joint(self, review: Review, index: int) -> None:
        if review.recalibration and index != review.recalibration.index:
            raise ValueError("Only the selected joint can be adjusted during single-joint review")
        review.saved_baseline = False

    def moving_motors(self, positions: list[int]) -> list[bool]:
        previous = self.previous_encoder_positions
        if previous is None or not isinstance(self.state, SweepRecording):
            self.previous_encoder_positions = list(positions)
            self.moving_until = [0.0] * len(positions)
            return [False] * len(positions)
        now = time.monotonic()
        for index, (before, after) in enumerate(zip(previous, positions, strict=True)):
            delta = abs(after - before)
            if min(delta, RESOLUTION - delta) >= 8:
                previous[index] = after
                self.moving_until[index] = now + 0.6
        return [until > now for until in self.moving_until]

    def view(self) -> CalibrationView:
        if isinstance(self.state, (Saved, Failed)):
            return self.last_view.model_copy(
                update={
                    "stage": "saved" if isinstance(self.state, Saved) else "error",
                    "message": f"Saved. Backups: {self.state.backup}"
                    if isinstance(self.state, Saved)
                    else self.state.message,
                    "command": "",
                    "configuration_path": str(self.state.configuration)
                    if isinstance(self.state, Saved)
                    else "",
                    "calibration_path": str(self.directory / "calibration.json")
                    if isinstance(self.state, Saved)
                    else "",
                }
            )
        positions = self.read_positions()
        moving_motors = self.moving_motors(positions)
        settings = self.setup.alignment
        if isinstance(self.state, SweepRecording):
            recording = self.state
            for index, sweep in recording.sweeps.items():
                sweep.record(positions[index])
            view = CalibrationView(
                stage="sweep",
                serial=self.serial,
                recalibrating_joint=recording.recalibration.index
                if recording.recalibration
                else None,
                gripper_opening=0.5,
                joints=[
                    JointView(
                        name=MOTOR_NAMES[index],
                        position_radians=0.0,
                        range_min=sweep.minimum,
                        range_max=sweep.maximum,
                        sign=recording.signs[index],
                        moving=moving_motors[index],
                    )
                    for index, sweep in recording.sweeps.items()
                ],
            )
            self.last_view = view
            return view
        if isinstance(self.state, Review):
            draft = self.state.draft
            positions = draft.positions(positions, self.original_homing)
            signs, offsets = draft.signs, draft.offsets
            minimums, maximums = draft.minimums, draft.maximums
            stage = "review"
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
        review = self.state if isinstance(self.state, Review) else None
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
                    moving=moving_motors[index],
                    sign=signs[index],
                )
                for index, name in enumerate(MOTOR_NAMES)
            ],
            gripper_opening=max(0.0, min(1.0, opening)),
            estimated=review is not None and review.estimated,
            can_recalibrate_joint=review is not None and review.saved_baseline,
            recalibrating_joint=review.recalibration.index
            if review and review.recalibration
            else None,
        )
        self.last_view = view
        return view

    def save(self, *, model_verified: bool) -> None:
        if not isinstance(self.state, Review) or not model_verified:
            raise ValueError("Review the live model and confirm it matches before saving")
        self.read_positions()
        draft = self.state.draft
        motor_ids = self.bus.driven_motor_ids
        entries = draft.entries(motor_ids)
        recalibration = self.state.recalibration
        calibration_path = self.directory / "calibration.json"
        updates = {
            "homing_offset": [entries[name].homing_offset for name in MOTOR_NAMES],
            "min_position_limit": [entries[name].range_min for name in MOTOR_NAMES],
            "max_position_limit": [entries[name].range_max for name in MOTOR_NAMES],
        }
        registers = {
            register: self.bus.read_registers(register) for register in CALIBRATION_REGISTERS
        }
        if recalibration:
            self.require_saved_setup()
            baseline = recalibration.baseline.draft
            if (
                read_calibration(calibration_path, motor_ids) != baseline.entries(motor_ids)
                or registers["homing_offset"] != baseline.homing_offsets
                or registers["min_position_limit"] != baseline.minimums
                or registers["max_position_limit"] != baseline.maximums
            ):
                raise ValueError(
                    "Saved calibration changed. Reconnect before recalibrating a joint."
                )
            target_indices = (recalibration.index,)
        else:
            target_indices = tuple(range(len(MOTOR_NAMES)))
            updates["operating_mode"] = [self.bus.definition.operating_modes["position"]] * 6
            if self.setup.servos.expected_return_delay_time is not None:
                updates["return_delay_time"] = [self.setup.servos.expected_return_delay_time] * 6
            servo_registers = self.setup.servo_registers
            updates["maximum_acceleration"] = [servo_registers.maximum_acceleration] * 6
            # The bus is in joint order, including when the physical IDs are reversed.
            updates["max_torque_limit"] = [
                *registers["max_torque_limit"][:5],
                servo_registers.gripper_max_torque_limit,
            ]
            updates["protection_current"] = [
                *registers["protection_current"][:5],
                servo_registers.gripper_protection_current,
            ]
        calibration_bytes = CALIBRATION_ADAPTER.dump_json(entries, indent=2)
        configuration_path = self.store.configuration_path(self.directory)
        result = ArmSetup.model_validate(
            {
                "serial": self.serial,
                "model_variant": self.setup.model_variant,
                "motor_ids": self.setup.motor_ids,
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
            (calibration_path, "calibration.json"),
            (live_path, "live-calibration.json"),
        ]:
            if source.exists():
                atomic_write(backup / filename, source.read_bytes())
        atomic_write(
            backup / "registers.json",
            json.dumps({"motor_ids": self.bus.driven_motor_ids, **registers}, indent=2).encode(),
        )
        try:
            self.require_torque_off()
            self.write_calibration_registers(
                registers, updates, target_indices, preserve_lock=recalibration is not None
            )
            if recalibration is None:
                apply_position_gains(self.bus, self.setup.servos)
            self.require_torque_off()
            atomic_write(calibration_path, calibration_bytes)
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

    def write_calibration_registers(
        self,
        registers: dict[str, list[int]],
        updates: dict[str, list[int]],
        target_indices: tuple[int, ...],
        *,
        preserve_lock: bool,
    ) -> None:
        motor_ids = self.bus.driven_motor_ids
        locked_indices = [index for index in target_indices if registers["lock"][index]]
        try:
            for index in locked_indices:
                self.bus.write_acknowledged(motor_ids[index], "lock", 0)
            for register, desired in updates.items():
                for index in target_indices:
                    if registers[register][index] != desired[index]:
                        self.bus.write_acknowledged(motor_ids[index], register, desired[index])
                if self.bus.read_registers(register) != desired:
                    raise RuntimeError(f"Calibration register verification failed: {register}")
        finally:
            if preserve_lock and locked_indices:
                for index in locked_indices:
                    self.bus.write_acknowledged(motor_ids[index], "lock", registers["lock"][index])
                if self.bus.read_registers("lock") != registers["lock"]:
                    raise RuntimeError("Calibration register verification failed: lock")
