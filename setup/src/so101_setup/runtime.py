"""Private application files and the packaged browser/model resources."""

import os
import tempfile
from pathlib import Path

from filelock import FileLock


def asset_directory() -> Path:
    return Path(__file__).parent / "assets/web"


def setup_directory() -> Path:
    config_home = Path(os.environ.get("XDG_CONFIG_HOME") or "~/.config").expanduser()
    return Path(os.environ.get("SO101_SETUP_DIRECTORY") or config_home / "so101-setup")


def setup_lock() -> FileLock:
    if os.name == "nt":
        directory = Path(os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()) / "so101-setup"
        directory.mkdir(parents=True, exist_ok=True)
        return FileLock(directory / "setup.lock", timeout=0)
    runtime_root = os.environ.get("XDG_RUNTIME_DIR")
    directory = (
        Path(runtime_root) / "so101-setup"
        if runtime_root
        else Path(tempfile.gettempdir()) / f"so101-setup-{os.getuid()}"
    )
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    status = directory.stat()
    if status.st_uid != os.getuid():
        raise PermissionError(f"{directory} belongs to another user")
    if status.st_mode & 0o077:
        directory.chmod(0o700)
    # Serial devices are also opened exclusively by rustypot. This lock prevents
    # multiple browser sessions from selecting arms in one setup application.
    return FileLock(directory / "setup.lock", timeout=0)
