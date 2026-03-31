import json
from pathlib import Path

Import("env")


def _has_current_version_flag(flags):
    for flag in flags:
        if str(flag).startswith("-DCURRENT_VERSION="):
            return True
    return False


project_dir = Path(env["PROJECT_DIR"])
version_path = project_dir / "version.json"

if version_path.exists() and not _has_current_version_flag(env.get("BUILD_FLAGS", [])):
    data = json.loads(version_path.read_text(encoding="utf-8-sig"))
    version = str(data.get("version", "")).strip()
    if version:
        env.Append(BUILD_FLAGS=[f'-DCURRENT_VERSION=\\"{version}\\"'])
