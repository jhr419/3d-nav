import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def _valid_workspace(path: Path) -> bool:
    return (path / "src").is_dir() and (path / "maps").is_dir()


def find_workspace_root() -> Path:
    env_value = os.environ.get("NAV3D_WS", "")
    if env_value:
        env_path = Path(env_value).expanduser()
        if _valid_workspace(env_path):
            return env_path

    cwd = Path.cwd()
    for candidate in (cwd, *cwd.parents):
        if _valid_workspace(candidate):
            return candidate

    share = Path(get_package_share_directory("nav3d_bringup")).resolve()
    for candidate in (share, *share.parents):
        if _valid_workspace(candidate):
            return candidate

    # Installed packages live under <ws>/install/<pkg>/share/<pkg>.
    if len(share.parents) >= 4:
        return share.parents[3]
    return cwd


def ws_path(*parts: str) -> str:
    return str(find_workspace_root().joinpath(*parts))


def package_file(package: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package)).joinpath(*parts))


def ensure_dir(*parts: str) -> str:
    path = find_workspace_root().joinpath(*parts)
    path.mkdir(parents=True, exist_ok=True)
    return str(path)
