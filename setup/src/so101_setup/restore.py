"""Restore one STS3215 servo's stored parameters with torque off."""

import argparse
import json
import time
from datetime import UTC, datetime
from pathlib import Path

from serial import Serial

from .configuration import MOTOR_IDS
from .device import BAUD_RATE, RegisterBus
from .discovery import discover_controllers
from .runtime import setup_directory, setup_lock
from .storage import NativeSetupStore

SETTINGS = (
    "id",
    "model_number",
    "firmware_major_version",
    "firmware_minor_version",
    "baud_rate",
    "response_status_level",
    "return_delay_time",
    "homing_offset",
    "min_position_limit",
    "max_position_limit",
    "operating_mode",
    "phase",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "maximum_acceleration",
    "maximum_velocity_limit",
    "acceleration_multiplier",
    "max_torque_limit",
    "protection_current",
    "minimum_startup_force",
    "cw_dead_zone",
    "ccw_dead_zone",
    "max_temperature_limit",
    "max_voltage_limit",
    "min_voltage_limit",
    "protective_torque",
    "protection_time",
    "overload_torque",
    "lock",
)


def require_disabled(bus: RegisterBus) -> None:
    if bus.read_registers("id") != list(MOTOR_IDS):
        raise RuntimeError("Could not verify the six motor IDs")
    if any(bus.read_registers("torque_enable")):
        raise RuntimeError("Every servo must have torque off")


def restore_parameters(port: str, motor_id: int, backup_directory: Path) -> Path:
    if motor_id not in MOTOR_IDS:
        raise ValueError("Specify one SO-101 motor ID (1-6)")
    bus = RegisterBus(port)
    try:
        require_disabled(bus)
        before = {name: bus.read_registers(name, [motor_id])[0] for name in SETTINGS}
        if (
            before["model_number"],
            before["firmware_major_version"],
            before["firmware_minor_version"],
        ) != (777, 3, 10):
            raise RuntimeError(
                "Parameter restore is only verified on STS3215 model 777 firmware 3.10"
            )
        backup_directory.mkdir(parents=True, exist_ok=True)
        backup = backup_directory / f"motor-{motor_id}-{datetime.now(UTC):%Y%m%dT%H%M%S%fZ}.json"
        with backup.open("x") as output:
            json.dump(before, output, indent=2)
        print(f"Saved motor settings to {backup}", flush=True)
        if before["lock"]:
            bus.write_acknowledged(motor_id, "lock", 0)
        if bus.read_registers("lock", [motor_id]) != [0]:
            raise RuntimeError("Motor EEPROM did not unlock")
    finally:
        bus.close()

    response = b""
    try:
        # Feetech parameter restore, addressed to one motor. No broadcast.
        body = bytes((motor_id, 2, 0x06))
        frame = b"\xff\xff" + body + bytes((~sum(body) & 0xFF,))
        with Serial(port, BAUD_RATE, timeout=0.3, write_timeout=0.3, exclusive=True) as link:
            link.reset_input_buffer()
            link.write(frame)
            link.flush()
            response = link.read(6)
    finally:
        time.sleep(0.5)
        bus = RegisterBus(port)
        try:
            require_disabled(bus)
            if bus.read_registers("lock", [motor_id]) != [before["lock"]]:
                bus.write_acknowledged(motor_id, "lock", before["lock"])
            after = {name: bus.read_registers(name, [motor_id])[0] for name in SETTINGS}
        finally:
            bus.close()
    backup.with_suffix(".after.json").write_text(json.dumps(after, indent=2))
    expected = b"\xff\xff" + bytes((motor_id, 2, 0, ~(motor_id + 2) & 0xFF))
    if response != expected:
        raise RuntimeError(f"Restore outcome unknown: unexpected response {response.hex(' ')}")
    return backup


def command_line() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True, help="USB serial of the SO-101 controller")
    parser.add_argument("--motor-id", required=True, type=int, choices=MOTOR_IDS)
    parser.add_argument("--setup-directory", type=Path, default=setup_directory())
    parser.add_argument("--confirm-restore", action="store_true")
    arguments = parser.parse_args()
    if not arguments.confirm_restore:
        parser.error("Stop the motion station, disable torque, then pass --confirm-restore")
    try:
        with setup_lock():
            controllers = discover_controllers(arguments.setup_directory, NativeSetupStore())
            controller = next(
                (item for item in controllers if item.serial == arguments.serial), None
            )
            if controller is None:
                raise RuntimeError(f"USB controller {arguments.serial} is absent or ambiguous")
            backup_directory = (
                arguments.setup_directory / "arms" / controller.serial / "backups" / "servo-restore"
            )
            restore_parameters(controller.port, arguments.motor_id, backup_directory)
    except Exception as error:
        parser.exit(1, f"Servo restore failed: {error}\n")
    print(
        f"Motor {arguments.motor_id} acknowledged restore. Restore its calibration before enabling."
    )
