"""Program a single loose SO-101 follower servo without running calibration."""

import rustypot
from pydantic import BaseModel
from serial.tools.list_ports import comports

from .configuration import MOTOR_IDS, SO101_JOINT_NAMES
from .device import BAUD_RATE, READ_RETRIES, TIMEOUT_SECONDS


class FollowerMotor(BaseModel):
    id: int
    joint: str


FOLLOWER_MOTORS = [
    FollowerMotor(id=motor_id, joint=joint)
    for motor_id, joint in zip(MOTOR_IDS, [*SO101_JOINT_NAMES, "gripper"], strict=True)
]


class ProgrammingPort(BaseModel):
    port: str
    description: str


def programming_ports() -> list[ProgrammingPort]:
    return [
        ProgrammingPort(port=port.device, description=port.description)
        for port in sorted(comports())
        if port.vid is not None
    ]


def program_motor_id(port: str, target_id: int) -> int:
    if target_id not in MOTOR_IDS:
        raise ValueError("Select a valid follower motor ID")
    controller = rustypot.Sts3215PyController
    definition = controller.definition()

    with rustypot.Bus(
        port, BAUD_RATE, TIMEOUT_SECONDS, dict.fromkeys(range(254), definition)
    ) as bus:
        for baud_rate in sorted(definition.baudrates, reverse=True):
            bus.set_baudrate(baud_rate)
            answers = bus.broadcast_scan(definition)
            if answers:
                break
        else:
            raise ValueError("No motor responded. Check motor power and the servo cable.")

        if len(answers) != 1:
            raise ValueError("More than one motor responded. Connect only the loose motor.")
        [(current_id, model)] = answers.items()
        if model != controller.models()["STS3215"]:
            raise ValueError(f"Expected an STS3215 servo, found model {model}")

        if baud_rate != BAUD_RATE:
            bus.change_baudrate(definition, current_id, BAUD_RATE)
            bus.set_baudrate(BAUD_RATE)
            if bus.read_register(current_id, "baudrate", retries=READ_RETRIES) != 0:
                raise RuntimeError("Could not verify the servo's new baud rate")

        if current_id != target_id:
            bus.change_id(definition, current_id, target_id)

        if bus.read_register(target_id, "id", retries=READ_RETRIES) != target_id:
            raise RuntimeError("Could not verify the new motor ID")
        if bus.read_register(target_id, "baudrate", retries=READ_RETRIES) != 0:
            raise RuntimeError("Could not verify the motor at 1 Mbps")
        if bus.read_register(target_id, "lock", retries=READ_RETRIES) != 1:
            status = bus.write_register_with_error(target_id, "lock", 1)
            if definition.refuses(status) or bus.read_register(target_id, "lock") != 1:
                raise RuntimeError("Could not lock the servo's settings")
    return target_id
