"""Validate the profile at the point where an arm's saved settings are loaded."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from so101_setup.storage import NativeSetupStore


@pytest.mark.parametrize(
    "servo_settings",
    [
        "expected_position_gains = [256, 0, 32]",
        'expected_position_gains_by_motor = { "1" = [16, -1, 32] }',
    ],
)
def test_setup_rejects_position_gains_that_do_not_fit_servo_registers(
    tmp_path: Path, servo_settings: str
) -> None:
    store = NativeSetupStore()
    store.configuration_path(tmp_path).write_text(f'serial = "ARM_A"\n[servos]\n{servo_settings}\n')

    with pytest.raises(ValidationError, match="expected_position_gains"):
        store.load(tmp_path, "ARM_A")
