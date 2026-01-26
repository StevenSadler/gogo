Import("env")  # type: ignore

import json
from pathlib import Path
from datetime import datetime, timezone
from SCons.Script import COMMAND_LINE_TARGETS # type: ignore

def log(msg):
    print(f"[extra_post] {msg}")

# ----------------------------------------------------------------------
# Paths
# ----------------------------------------------------------------------
PROJECT_DIR = Path(env["PROJECT_DIR"])  # type: ignore
BUILD_LOG_DIR = PROJECT_DIR / ".pio/build/build_logs/src"


# ----------------------------------------------------------------------
# Detect build intent
# ----------------------------------------------------------------------
IS_UPLOAD = "upload" in COMMAND_LINE_TARGETS
ACTION_LABEL = "build+upload" if IS_UPLOAD else "build-only"


# ----------------------------------------------------------------------
# Post-build action — runs ONLY on successful firmware build
# ----------------------------------------------------------------------
def on_firmware_built(source, target, env):
    log("Firmware built successfully — running post-build finalization")

    current_build_file = BUILD_LOG_DIR / "CURRENTBUILD"
    if not current_build_file.exists():
        log("CURRENTBUILD not found — skipping post-build finalization")
        return

    build_hash = current_build_file.read_text().strip()
    hash_dir = BUILD_LOG_DIR / build_hash
    info_pre = hash_dir / "info_pre.json"

    if not info_pre.exists():
        log(f"info_pre.json missing for hash {build_hash} — skipping")
        return

    metadata = json.loads(info_pre.read_text())

    timestamp_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    metadata["post_build_timestamp_utc"] = timestamp_utc
    metadata["post_build_notes"] = "Firmware built successfully"

    (hash_dir / "info_post.json").write_text(json.dumps(metadata, indent=2))

    with open(BUILD_LOG_DIR / "build_history_post.txt", "a") as f:
        f.write(
            f"{timestamp_utc} - "
            f"Build Hash: {build_hash} ({ACTION_LABEL})\n"
        )

    log(f"Post-build metadata written for hash {build_hash} ({ACTION_LABEL})")

# ----------------------------------------------------------------------
# Attach to firmware target
# ----------------------------------------------------------------------
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", on_firmware_built) # type: ignore
