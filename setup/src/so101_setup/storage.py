"""Native setup files and the small persistence boundary for host applications."""

import tomllib
from pathlib import Path
from typing import Protocol

import tomli_w

from .configuration import ArmSetup


class SetupStore(Protocol):
    def configuration_path(self, directory: Path) -> Path: ...
    def load(self, directory: Path, serial: str) -> ArmSetup: ...
    def encode(self, directory: Path, setup: ArmSetup) -> bytes: ...


class NativeSetupStore:
    """One setup.toml and LeRobot calibration.json per controller directory."""

    def configuration_path(self, directory: Path) -> Path:
        return directory / "setup.toml"

    def load(self, directory: Path, serial: str) -> ArmSetup:
        path = self.configuration_path(directory)
        if not path.exists():
            if (directory / "profile.toml").exists():
                raise ValueError(
                    "This directory contains a host profile. Use its setup adapter or select a new standalone setup directory."
                )
            return ArmSetup(serial=serial)
        setup = ArmSetup.model_validate(tomllib.loads(path.read_text()))
        if setup.serial != serial:
            raise ValueError(f"Setup serial {setup.serial} does not match USB controller {serial}")
        return setup

    def encode(self, directory: Path, setup: ArmSetup) -> bytes:
        return tomli_w.dumps(setup.model_dump(mode="json")).encode()
