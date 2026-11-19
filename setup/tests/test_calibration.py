"""Calibration outcomes across encoder mapping, register writes and saved files."""

import json
import tomllib
from pathlib import Path

import pytest
import rustypot
import tomli_w

from so101_setup.calibration import (
    HOME_GRIPPER_RADIANS,
    HOME_JOINTS,
    CalibrationSession,
    Failed,
    Recording,
    Reference,
    Review,
    Saved,
    SweepRecording,
    joint_radians,
    read_calibration,
)
from so101_setup.configuration import ArmSetup, ModelVariant, ServoSettings
from so101_setup.device import RegisterBus
from so101_setup.storage import NativeSetupStore


class MemoryServoBus(RegisterBus):
    """An arm's registers at the serial boundary; no third-party behavior mocked."""

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
        self.closed = False
        self.drop_register: str | None = None
        self.pending_positions: list[list[int]] = []

    def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
        if name == "present_position" and self.pending_positions:
            self.registers[name] = self.pending_positions.pop(0)
        ids = self.driven_motor_ids if motor_ids is None else motor_ids
        return [self.registers[name][motor_id - 1] for motor_id in ids]

    def write_acknowledged(self, motor_id: int, register: str, value: int) -> None:
        self.writes.append((motor_id, register, value))
        if register != self.drop_register:
            self.registers[register][motor_id - 1] = value

    def close(self) -> None:
        self.closed = True


def session_at(
    tmp_path: Path, bus: MemoryServoBus, model_variant: ModelVariant = "stock"
) -> CalibrationSession:
    setup = ArmSetup(
        serial="ARM_A",
        model_variant=model_variant,
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


@pytest.mark.parametrize("model_variant", ["stock", "soft-gripper-camera"])
def test_draft_is_read_only_and_save_creates_a_standalone_configuration(
    tmp_path: Path,
    model_variant: ModelVariant,
) -> None:
    bus = MemoryServoBus()
    # A nonzero existing homing must be removed before deriving the new one.
    bus.registers["homing_offset"] = [100] * 6
    session = session_at(tmp_path, bus, model_variant)
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
    assert setup.model_variant == model_variant
    assert setup.gripper.open_position_radians > setup.gripper.closed_position_radians
    assert read_calibration(session.state.calibration) == calibration
    assert (session.state.backup / "registers.json").is_file()
    assert bus.registers["p_coefficient"] == [16, 32, 32, 16, 16, 16]
    assert bus.registers["return_delay_time"] == [0] * 6
    assert bus.registers["maximum_acceleration"] == [254] * 6
    assert bus.registers["max_torque_limit"] == [1000] * 5 + [500]
    assert bus.registers["protection_current"] == [310] * 5 + [250]
    assert not (session.directory / "servo-overrides.toml").exists()
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
    assert "servos" not in document
    document["servos"] = {"expected_position_gains_by_motor": {"2": [24, 0, 32]}}
    (session.directory / "setup.toml").write_text(tomli_w.dumps(document))
    (session.directory / "servo-overrides.toml").write_text("maximum_acceleration = 100\n")
    setup = NativeSetupStore().load(session.directory, "ARM_A")
    writes_before_reconnect = list(bus.writes)
    reconnected = CalibrationSession(setup, session.directory, session.calibration_root, bus)
    assert isinstance(reconnected.state, Review)
    assert bus.writes == writes_before_reconnect
    reconnected.save(model_verified=True)
    saved_setup = NativeSetupStore().load(session.directory, "ARM_A")
    assert saved_setup.gripper.closed_position_radians == 0.5
    assert saved_setup.gripper.open_position_radians == 1.0
    assert saved_setup.servos.expected_position_gains_by_motor[2] == [32, 0, 32]
    assert bus.registers["p_coefficient"][1] == 32
    assert bus.registers["maximum_acceleration"] == [100] * 6
    assert "servos" not in tomllib.loads((session.directory / "setup.toml").read_text())


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


def test_sweep_estimates_wrapped_wrist_and_gripper_without_writing_registers(
    tmp_path: Path,
) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus, "soft-gripper-camera")
    bus.registers["present_position"][5] = 1800
    session.start_sweep()
    for wrist in (1500, 900, 300, 3796, 3196, 2596, 2296):
        bus.registers["present_position"] = [900, 900, 900, 900, wrist, 1800]
        session.view()
    bus.registers["present_position"] = [2047, 2047, 2047, 2047, 2296, 2500]
    session.view()
    for wrist in (2896, 3496, 0, 600, 1200, 1800, 2047):
        bus.registers["present_position"] = [3200, 3200, 3200, 3200, wrist, 3200]
        session.view()
    session.finish_sweep()
    assert isinstance(session.state, Review)
    assert session.state.estimated
    home = session.state.draft.positions(
        [2047, 2047, 2047, 2047, 2047, 1800], session.original_homing
    )
    angles = [
        joint_radians(position, sign, offset)
        for position, sign, offset in zip(
            home, session.state.draft.signs, session.state.draft.offsets, strict=True
        )
    ]
    assert angles == pytest.approx([*HOME_JOINTS, HOME_GRIPPER_RADIANS])
    wrist_calibration = session.state.draft.entries()["wrist_roll"]
    assert 0 < wrist_calibration.range_min < wrist_calibration.range_max < 4095
    assert wrist_calibration.range_max - wrist_calibration.range_min > 3600
    assert session.state.gripper_endpoints[0] == pytest.approx(-0.12105)
    assert bus.writes == []
    session.adjust_joint(5, 0.01)
    assert session.state.gripper_endpoints[0] == pytest.approx(-0.11105)
    assert bus.writes == []


def test_saved_calibration_can_be_adjusted_without_losing_wrist_limits(tmp_path: Path) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus, "soft-gripper-camera")
    session.start_sweep()
    for position in (600, 2000, 3400):
        bus.registers["present_position"] = [position] * 6
        session.view()
    session.finish_sweep()
    session.save(model_verified=True)
    saved = read_calibration(session.directory / "calibration.json")
    saved_setup = NativeSetupStore().load(session.directory, "ARM_A")
    reopened = CalibrationSession(saved_setup, session.directory, session.calibration_root, bus)
    assert isinstance(reopened.state, Review)
    reopened.adjust_joint(1, 0.01)
    assert reopened.state.draft.entries()["wrist_roll"] == saved["wrist_roll"]
    assert reopened.state.draft.offsets[1] == pytest.approx(
        saved_setup.alignment.joint_offsets_radians[1] + 0.01
    )


def test_recalibrating_one_joint_preserves_other_calibration_and_servo_settings(
    tmp_path: Path,
) -> None:
    motor_ids = [6, 5, 4, 3, 2, 1]
    bus = MemoryServoBus(motor_ids)
    setup = ArmSetup(serial="ARM_A", motor_id_order="descending")
    directory = tmp_path / "arms/ARM_A"
    session = CalibrationSession(setup, directory, tmp_path / "live", bus)
    record_travel(session, bus)
    session.save(model_verified=True)
    calibration_path = directory / "calibration.json"
    saved = read_calibration(calibration_path, motor_ids)
    saved_setup = NativeSetupStore().load(directory, "ARM_A")
    bus.registers["p_coefficient"][0] = 27
    bus.registers["return_delay_time"][2] = 7
    bus.registers["lock"] = [1] * 6
    saved_registers = {name: values.copy() for name, values in bus.registers.items()}
    bus.writes.clear()

    reopened = CalibrationSession(saved_setup, directory, session.calibration_root, bus)
    assert reopened.view().can_recalibrate_joint
    reopened.start_sweep(joint_index=3)
    with pytest.raises(ValueError, match="wrist flex: 0° recorded"):
        reopened.finish_sweep()
    # Joint index 3 maps to motor ID 3 on this descending arm.
    for position in (750, 2100, 3250):
        bus.registers["present_position"][2] = position
        reopened.view()
    reopened.finish_sweep()
    assert reopened.view().recalibrating_joint == 3
    with pytest.raises(ValueError, match="Only the selected joint"):
        reopened.adjust_joint(0, 0.01)
    assert bus.writes == []
    reopened.save(model_verified=True)

    updated = read_calibration(calibration_path, motor_ids)
    wrist = updated.pop("wrist_flex")
    assert (wrist.id, wrist.homing_offset, wrist.range_min, wrist.range_max) == (3, -47, 797, 3297)
    saved.pop("wrist_flex")
    assert updated == saved
    updated_setup = NativeSetupStore().load(directory, "ARM_A")
    assert updated_setup.gripper == saved_setup.gripper
    assert updated_setup.servos == saved_setup.servos
    assert (
        updated_setup.alignment.joint_offsets_radians[:3]
        == saved_setup.alignment.joint_offsets_radians[:3]
    )
    assert (
        updated_setup.alignment.joint_offsets_radians[4]
        == saved_setup.alignment.joint_offsets_radians[4]
    )
    assert {motor_id for motor_id, _, _ in bus.writes} == {3}
    for name, expected in (
        ("homing_offset", -47),
        ("min_position_limit", 797),
        ("max_position_limit", 3297),
    ):
        saved_registers[name][2] = expected
    saved_registers.pop("present_position")
    assert {name: bus.registers[name] for name in saved_registers} == saved_registers


def test_incomplete_gripper_sweep_can_be_completed_before_review(tmp_path: Path) -> None:
    bus = MemoryServoBus()
    session = session_at(tmp_path, bus, "soft-gripper-camera")
    session.start_sweep()
    for position in (600, 2000, 3400):
        bus.registers["present_position"] = [position] * 5 + [2047]
        session.view()
    with pytest.raises(ValueError, match="gripper: 0° recorded"):
        session.finish_sweep()
    assert isinstance(session.state, SweepRecording)
    assert bus.writes == []
    bus.registers["present_position"][5] = 3400
    session.view()
    session.finish_sweep()
    assert isinstance(session.state, Review)
    assert bus.writes == []


def test_reverse_calibration_writes_physical_joint_ranges_and_reopens(tmp_path: Path) -> None:
    bus = MemoryServoBus([6, 5, 4, 3, 2, 1])
    setup = ArmSetup(serial="ARM_A", motor_id_order="descending")
    directory = tmp_path / "arms/ARM_A"
    session = CalibrationSession(setup, directory, tmp_path / "live", bus)
    session.capture_reference()
    bus.registers["present_position"] = [1750, 1760, 1770, 1780, 1790, 1800]
    session.view()
    session.capture_gripper_endpoint("closed")
    bus.registers["present_position"] = [2250, 2260, 2270, 2280, 2290, 2300]
    session.view()
    session.capture_gripper_endpoint("open")
    session.finish_sweep()
    session.save(model_verified=True)

    saved_setup = NativeSetupStore().load(directory, "ARM_A")
    assert saved_setup.motor_id_order == "descending"
    calibration_path = directory / "calibration.json"
    entries = read_calibration(calibration_path, [6, 5, 4, 3, 2, 1])
    assert (entries["shoulder_pan"].id, entries["shoulder_pan"].range_min) == (6, 1800)
    assert (entries["gripper"].id, entries["gripper"].range_min) == (1, 1750)
    assert bus.registers["min_position_limit"] == [1750, 0, 1770, 1780, 1790, 1800]

    reopened = CalibrationSession(saved_setup, directory, tmp_path / "live", bus)
    assert isinstance(reopened.state, Review)
    bus.driven_motor_ids = bus.all_motor_ids = [1, 2, 3, 4, 5, 6]
    changed_order = CalibrationSession(ArmSetup(serial="ARM_A"), directory, tmp_path / "live", bus)
    assert isinstance(changed_order.state, Reference)


def test_unplugged_calibration_resumes_its_draft_only_on_the_original_board(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from so101_setup import server
    from so101_setup.device import RegisterReadError
    from so101_setup.discovery import ControllerChoice

    class DisconnectableBus(MemoryServoBus):
        unplugged = False

        def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
            if self.unplugged:
                raise RegisterReadError("No such device or address")
            if name == "id":
                return self.driven_motor_ids
            return super().read_registers(name, motor_ids)

    original_bus = DisconnectableBus()
    reconnected_bus = DisconnectableBus()
    available = [ControllerChoice(serial="ARM_A", port="original", registered=False)]
    buses = {"original": original_bus, "new-port": reconnected_bus}
    monkeypatch.setattr(server, "discover_controllers", lambda *_: available)
    monkeypatch.setattr(server, "RegisterBus", lambda port, _motor_ids: buses[port])
    application = server.CalibrationApplication(tmp_path, tmp_path / "live")
    assert application.view().stage == "reference"
    session = application.session
    assert session is not None
    session.capture_reference()
    original_bus.registers["present_position"] = [1800] * 6
    assert application.view().stage == "sweep"
    draft = session.state

    original_bus.unplugged = True
    assert application.view().stage == "error"
    assert original_bus.closed
    available[:] = [ControllerChoice(serial="ARM_B", port="other", registered=False)]
    assert "ARM_A" in application.view().message

    available[:] = [ControllerChoice(serial="ARM_A", port="new-port", registered=False)]
    reconnected_bus.registers["present_position"] = [2300] * 6
    assert application.view().stage == "sweep"
    assert application.session is session
    assert session.state is draft
    assert session.bus is reconnected_bus
    assert not original_bus.writes
    assert not reconnected_bus.writes
