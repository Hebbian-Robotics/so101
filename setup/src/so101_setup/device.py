"""Acknowledged Feetech I/O and gain verification shared by setup applications."""

import rustypot

from .configuration import MOTOR_IDS, ServoSettings

BAUD_RATE = 1_000_000
# Longer than a USB serial adapter's 16 ms latency timer, as LeRobot uses.
TIMEOUT_SECONDS = 0.05
READ_RETRIES = 2


class RegisterBus:
    """Acknowledged register reads and writes on one arm's Feetech bus."""

    def __init__(
        self,
        port: str,
        motor_ids: list[int] | None = None,
        passive_motor_ids: list[int] | None = None,
    ) -> None:
        self.definition = rustypot.Sts3215PyController.definition()
        self.driven_motor_ids = list(MOTOR_IDS if motor_ids is None else motor_ids)
        self.all_motor_ids = [*self.driven_motor_ids, *(passive_motor_ids or [])]
        self.bus = rustypot.Bus(
            port,
            BAUD_RATE,
            TIMEOUT_SECONDS,
            dict.fromkeys(self.all_motor_ids, self.definition),
        )

    def close(self) -> None:
        """Release the exclusive serial port when commissioning is finished."""
        self.bus.close()

    def read_registers(self, name: str, motor_ids: list[int] | None = None) -> list[int]:
        ids = self.driven_motor_ids if motor_ids is None else motor_ids
        answers = self.bus.sync_read_register_with_error(ids, name, retries=READ_RETRIES)
        refused = [
            motor_id
            for motor_id, (_, status) in zip(ids, answers, strict=True)
            if self.definition.refuses(status)
        ]
        if refused:
            raise RuntimeError(f"Motors {refused} refused reading {name}")
        return [value for value, _ in answers]

    def write_acknowledged(self, motor_id: int, register: str, value: int) -> None:
        status = self.bus.write_register_with_error(motor_id, register, value)
        if self.definition.refuses(status):
            raise RuntimeError(f"Motor {motor_id} refused writing {register}: {status}")


def apply_position_gains(bus: RegisterBus, settings: ServoSettings) -> None:
    if any(bus.read_registers("torque_enable", bus.all_motor_ids)):
        raise RuntimeError("Support and disable every motor before applying gains")
    for register, index in (
        ("p_coefficient", 0),
        ("i_coefficient", 1),
        ("d_coefficient", 2),
    ):
        actual = bus.read_registers(register)
        expected = [
            settings.expected_position_gains_by_motor.get(
                motor_id, settings.expected_position_gains
            )[index]
            for motor_id in bus.driven_motor_ids
        ]
        for motor_id, current, desired in zip(bus.driven_motor_ids, actual, expected, strict=True):
            if current != desired:
                bus.write_acknowledged(motor_id, register, desired)
        if bus.read_registers(register) != expected:
            raise RuntimeError(f"Position gain verification failed: {register}")
    if any(bus.read_registers("torque_enable", bus.all_motor_ids)):
        raise RuntimeError("Unexpected enabled torque after gain setup")
