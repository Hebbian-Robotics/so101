"""Calibration outcomes across encoder mapping, register writes and saved files."""

import json
import tomllib
from pathlib import Path

import pytest
import rustypot
import tomli_w

from so101_setup.calibration import (
    CalibrationSession,
    Failed,
    Recording,
    Reference,
    Review,
    Saved,
    read_calibration,
)
from so101_setup.configuration import ArmSetup, ServoSettings
from so101_setup.device import RegisterBus
from so101_setup.storage import NativeSetupStore


class MemoryServoBus(RegisterBus):
    """An arm's registers at the serial boundary; no third-party behavior mocked."""

    def __init__(self) -> None:
        self.driven_motor_ids = [1, 2, 3, 4, 5, 6]
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
            "lock": [0] * 6,
        }
        self.writes: list[tuple[int, str, int]] = []
        self.closed = False
        self.drop_register: str | None = None
        self.pending_positions: list[list[int]] = []

    def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
        if name == "present_position" and self.pending_positions:
            self.registers[name] = self.pending_positions.pop(0)
        return list(self.registers[name])

    def write_acknowledged(self, motor_id: int, register: str, value: int) -> None:
        self.writes.append((motor_id, register, value))
        if register != self.drop_register:
            self.registers[register][motor_id - 1] = value

    def close(self) -> None:
        self.closed = True


def session_at(tmp_path: Path, bus: MemoryServoBus) -> CalibrationSession:
    setup = ArmSetup(
        serial="ARM_A",
        servos=ServoSettings(expected_position_gains_by_motor={2: [32, 0, 32], 3: [32, 0, 32]}),
    )
    return CalibrationSession(setup, tmp_path / "arms/ARM_A", tmp_path / "live", bus)


def record_travel(session: CalibrationSession, bus: MemoryServoBus) -> None:
    session.capture_reference()
    bus.registers["present_position"] = [1800] * 6
    session.view()
    session.capture_gripper_endpoint("closed")
    bus.registers["present_position"] = [2300] * 6
    session.view()
    session.capture_gripper_endpoint("open")
    session.finish_sweep()


def test_draft_is_read_only_and_save_creates_a_standalone_configuration(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    # A nonzero existing homing must be removed before deriving the new one.
    bus.registers["homing_offset"] = [100] * 6
    session = session_at(tmp_path, bus)
    assert isinstance(session.state, Reference)
    record_travel(session, bus)
    assert isinstance(session.state, Review)
    assert bus.writes == []
    assert not session.directory.exists()
    session.save(model_verified=True)
    assert isinstance(session.state, Saved)
    calibration = read_calibration(session.directory / "calibration.json")
    assert calibration["shoulder_pan"].homing_offset == 100
    assert calibration["shoulder_pan"].range_min == 1800
    assert calibration["shoulder_pan"].range_max == 2300
    assert calibration["wrist_roll"].range_min == 0
    assert calibration["wrist_roll"].range_max == 4095
    setup = NativeSetupStore().load(session.directory, "ARM_A")
    assert setup.serial == "ARM_A"
    assert setup.gripper.open_position_radians > setup.gripper.closed_position_radians
    assert read_calibration(session.state.calibration) == calibration
    assert (session.state.backup / "registers.json").is_file()
    assert bus.registers["p_coefficient"] == [16, 32, 32, 16, 16, 16]
    assert bus.registers["return_delay_time"] == [0] * 6
    assert bus.registers["torque_enable"] == [0] * 6
    assert not any(register in {"torque_enable", "goal_position"} for _, register, _ in bus.writes)
    assert bus.closed


@pytest.mark.parametrize("reference_steps", [2047, 4095])
def test_reversing_a_joint_preserves_its_reference_and_changes_the_live_direction(
    tmp_path: Path,
    reference_steps: int,
) -> None:
    bus = MemoryServoBus()
    bus.registers["present_position"][1] = reference_steps
    session = session_at(tmp_path, bus)
    session.capture_reference()
    session.reverse_joint(1)
    reference = session.view()
    assert reference.joints[1].position_radians == 0.0
    bus.registers["present_position"][1] = (reference_steps + 100) % 4096
    assert session.view().joints[1].position_radians < 0
    assert bus.writes == []


def test_sweeps_require_movement_and_refuse_an_encoder_wrap(tmp_path: Path) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    session.capture_reference()
    with pytest.raises(ValueError, match="nonempty"):
        session.finish_sweep()
    assert isinstance(session.state, Recording)
    for position in [1800, 2300]:
        bus.registers["present_position"] = [position] * 6
        session.view()
    with pytest.raises(ValueError, match="Record distinct"):
        session.finish_sweep()
    # Gradual physical motion crosses zero between consecutive samples.
    for position in [1000, 100, 4090]:
        bus.registers["present_position"][0] = position
        if position == 4090:
            with pytest.raises(ValueError, match="encoder boundary"):
                session.view()
        else:
            session.view()
    assert bus.writes == []


def test_torque_and_operator_confirmation_gate_every_save(tmp_path: Path) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    record_travel(session, bus)
    with pytest.raises(ValueError, match="confirm"):
        session.save(model_verified=False)
    bus.registers["torque_enable"][2] = 1
    with pytest.raises(RuntimeError, match="torque is enabled"):
        session.save(model_verified=True)
    assert bus.writes == []
    assert not session.directory.exists()


def test_failed_register_verification_keeps_files_and_records_recovery_information(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    record_travel(session, bus)
    session.directory.mkdir(parents=True)
    previous_profile = 'serial = "ARM_A"\n'
    (session.directory / "setup.toml").write_text(previous_profile)
    bus.drop_register = "min_position_limit"
    with pytest.raises(RuntimeError, match="verification failed"):
        session.save(model_verified=True)
    assert isinstance(session.state, Failed)
    assert (session.directory / "setup.toml").read_text() == previous_profile
    assert not (session.directory / "calibration.json").exists()
    backups = list((session.directory / "backups").iterdir())
    assert len(backups) == 1
    assert (backups[0] / "setup.toml").read_text() == previous_profile
    assert json.loads((backups[0] / "registers.json").read_text())["min_position_limit"] == [0] * 6
    assert bus.registers["torque_enable"] == [0] * 6
    assert bus.closed


def test_matching_saved_calibration_skips_manual_steps_and_preserves_tuned_endpoints(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    record_travel(session, bus)
    session.save(model_verified=True)
    document = tomllib.loads((session.directory / "setup.toml").read_text())
    document["gripper"]["closed_position_radians"] = 0.5
    document["gripper"]["open_position_radians"] = 1.0
    (session.directory / "setup.toml").write_text(tomli_w.dumps(document))
    setup = NativeSetupStore().load(session.directory, "ARM_A")
    writes_before_reconnect = list(bus.writes)
    reconnected = CalibrationSession(setup, session.directory, session.calibration_root, bus)
    assert isinstance(reconnected.state, Review)
    assert bus.writes == writes_before_reconnect
    reconnected.save(model_verified=True)
    saved_setup = NativeSetupStore().load(session.directory, "ARM_A")
    assert saved_setup.gripper.closed_position_radians == 0.5
    assert saved_setup.gripper.open_position_radians == 1.0


def test_explicit_gripper_endpoints_can_be_inside_travel_and_in_reverse_encoder_order(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    session.capture_reference()
    session.reverse_joint(5)
    for position in [1800, 2300]:
        bus.registers["present_position"] = [position] * 6
        session.view()
    bus.registers["present_position"][5] = 2200
    session.capture_gripper_endpoint("closed")
    bus.registers["present_position"][5] = 1900
    session.capture_gripper_endpoint("open")
    session.finish_sweep()
    session.save(model_verified=True)
    setup = NativeSetupStore().load(session.directory, "ARM_A")
    assert setup.gripper.closed_position_radians == pytest.approx(0.5153, abs=0.0001)
    assert setup.gripper.open_position_radians == pytest.approx(0.9755, abs=0.0001)
    entry = read_calibration(session.directory / "calibration.json")["gripper"]
    assert (entry.range_min, entry.range_max) == (1800, 2300)


def test_corrupt_saved_calibration_enters_a_read_only_repair_flow(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    directory = tmp_path / "arms/ARM_A"
    directory.mkdir(parents=True)
    (directory / "calibration.json").write_text("{}")
    session = session_at(tmp_path, bus)
    assert isinstance(session.state, Reference)
    assert "could not be reused" in session.view().message
    assert bus.writes == []


def test_an_endpoint_captured_while_the_hand_moves_is_inside_its_recorded_limits(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus)
    session.capture_reference()
    bus.pending_positions = [[1810] * 6, [1800] * 6]
    session.capture_gripper_endpoint("closed")
    assert isinstance(session.state, Recording)
    endpoint = session.state.closed_gripper_steps
    assert endpoint is not None
    assert session.state.draft.minimums[5] <= endpoint <= session.state.draft.maximums[5]
