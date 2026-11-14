"""USB metadata discovery without opening serial ports or launching a station."""

import logging
from collections import defaultdict
from pathlib import Path

from pydantic import BaseModel, Field, ValidationError
from serial.tools.list_ports import comports

from .configuration import ControllerSerial
from .storage import SetupStore


class UsbController(BaseModel):
    serial: ControllerSerial
    port: str = Field(min_length=1)


class ControllerChoice(UsbController):
    registered: bool


def match_controllers(
    controllers: list[UsbController], directory: Path, store: SetupStore
) -> list[ControllerChoice]:
    by_serial: dict[str, set[str]] = defaultdict(set)
    for controller in controllers:
        by_serial[controller.serial].add(controller.port)
    matches = []
    for serial, ports in sorted(by_serial.items()):
        if len(ports) != 1:
            logging.warning("Ignoring ambiguous USB serial %s on ports %s", serial, sorted(ports))
            continue
        matches.append(
            ControllerChoice(
                serial=serial,
                port=next(iter(ports)),
                registered=store.configuration_path(directory / "arms" / serial).is_file(),
            )
        )
    return matches


def discover_controllers(directory: Path, store: SetupStore) -> list[ControllerChoice]:
    controllers = []
    for port in comports():
        if port.vid is None or port.serial_number is None:
            continue
        try:
            controllers.append(UsbController(serial=port.serial_number, port=port.device))
        except ValidationError:
            logging.warning("Ignoring USB port %s with invalid serial identity", port.device)
    return match_controllers(controllers, directory, store)
