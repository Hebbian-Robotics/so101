"""USB identity matching is deterministic and does not guess duplicate serials."""

from pathlib import Path

from so101_setup.configuration import ArmSetup
from so101_setup.discovery import UsbController, match_controllers
from so101_setup.storage import NativeSetupStore


def test_new_arm_model_selection_preserves_existing_setup(tmp_path: Path) -> None:
    store = NativeSetupStore("soft-gripper-camera")
    setup = store.load(tmp_path, "ARM_A")
    assert setup.model_variant == "soft-gripper-camera"
    (tmp_path / "setup.toml").write_bytes(NativeSetupStore().encode(tmp_path, setup))
    assert NativeSetupStore("stock").load(tmp_path, "ARM_A").model_variant == "soft-gripper-camera"


def test_registered_unknown_and_duplicate_controller_identities(tmp_path: Path) -> None:
    store = NativeSetupStore()
    directory = tmp_path / "arms/KNOWN"
    directory.mkdir(parents=True)
    store.configuration_path(directory).write_bytes(
        store.encode(directory, ArmSetup(serial="KNOWN"))
    )
    controllers = [
        UsbController(serial=serial, port=port)
        for serial, port in [
            ("KNOWN", "/dev/ttyUSB3"),
            ("OTHER", "/dev/ttyUSB1"),
            ("DUPLICATE", "/dev/ttyUSB2"),
            ("DUPLICATE", "/dev/ttyUSB4"),
        ]
    ]
    matches = match_controllers(controllers, tmp_path, store)
    assert [
        (controller.serial, controller.port, controller.registered) for controller in matches
    ] == [
        ("KNOWN", "/dev/ttyUSB3", True),
        ("OTHER", "/dev/ttyUSB1", False),
    ]
