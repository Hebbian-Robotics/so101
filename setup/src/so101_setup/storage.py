"""Native setup files and the small persistence boundary for host applications."""

import tomllib
from pathlib import Path
from typing import Protocol

import tomli_w

from .configuration import (
    DEFAULT_NEW_ARM_MODEL,
    ArmSetup,
    ModelVariant,
    ServoRegisterSettings,
    ServoSettings,
)


class SetupStore(Protocol):
    def configuration_path(self, directory: Path) -> Path: ...
    def load(self, directory: Path, serial: str) -> ArmSetup: ...
    def encode(self, directory: Path, setup: ArmSetup) -> bytes: ...


class NativeSetupStore:
    """One setup.toml and LeRobot calibration.json per controller directory."""

    def __init__(self, default_model_variant: ModelVariant = DEFAULT_NEW_ARM_MODEL) -> None:
        self.default_model_variant = default_model_variant

    def configuration_path(self, directory: Path) -> Path:
        return directory / "setup.toml"

    def load(self, directory: Path, serial: str) -> ArmSetup:
        path = self.configuration_path(directory)
        if not path.exists():
            if (directory / "profile.toml").exists():
                raise ValueError(
                    "This directory contains a host profile. Use its setup adapter or select a new standalone setup directory."
                )
            setup = ArmSetup(serial=serial, model_variant=self.default_model_variant)
            return self.with_servo_defaults(directory, setup)
        document = tomllib.loads(path.read_text())
        setup = ArmSetup.model_validate(document)
        if setup.serial != serial:
            raise ValueError(f"Setup serial {setup.serial} does not match USB controller {serial}")
        return self.with_servo_defaults(directory, setup)

    @staticmethod
    def with_servo_defaults(directory: Path, setup: ArmSetup) -> ArmSetup:
        shoulder_id, elbow_id = (2, 3) if setup.motor_id_order == "ascending" else (5, 4)
        override_path = directory / "servo-overrides.toml"
        register_settings = (
            ServoRegisterSettings.model_validate(tomllib.loads(override_path.read_text()))
            if override_path.exists()
            else ServoRegisterSettings()
        )
        return setup.model_copy(
            update={
                "servo_registers": register_settings,
                "servos": ServoSettings(
                    expected_position_gains_by_motor={
                        shoulder_id: [32, 0, 32],
                        elbow_id: [32, 0, 32],
                    }
                ),
            }
        )

    def encode(self, directory: Path, setup: ArmSetup) -> bytes:
        document = setup.model_dump(mode="json", exclude={"servos", "servo_registers"})
        return tomli_w.dumps(document).encode()
