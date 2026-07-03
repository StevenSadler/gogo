Import("env")  # type: ignore

import sys
import hashlib
import json
from pathlib import Path
from datetime import datetime, timezone
from SCons.Script import COMMAND_LINE_TARGETS  # type: ignore

def log(msg):
    print(f"[extra_pre] {msg}")

def is_real_build():
    return any(t in ("build", "upload", "program") for t in COMMAND_LINE_TARGETS)

# Detect if running clean
is_clean = any("clean" in arg.lower() for arg in sys.argv)
log(f"Check sys.argv for is_clean: {is_clean}")
log(f"Command line targets: {COMMAND_LINE_TARGETS}")

# if is_clean or not is_real_build():
if is_clean:
    log("No real build detected — skipping pre-build steps")

else:
    log("Real build detected — running pre-build steps")
    log("STEP 1: extra_pre.py loaded")

    # ----------------------------------------------------------------------
    # Paths
    # ----------------------------------------------------------------------
    PROJECT_DIR = Path(env["PROJECT_DIR"])  # type: ignore
    CONTRACT_PATH = PROJECT_DIR / "config" / "contract" / "control_contract.json"
    SRC_DIR = PROJECT_DIR / "src"
    INCLUDE_DIR = PROJECT_DIR / "include"
    GENERATED_DIR = PROJECT_DIR / "include/generated"

    GENERATED_HEADER_DIR = GENERATED_DIR
    BUILD_LOG_DIR = PROJECT_DIR / ".pio/build/build_logs/src"
    LIB_DEPS_DIR = Path(env["PROJECT_LIBDEPS_DIR"])  # type: ignore

    GENERATED_HEADER_DIR.mkdir(parents=True, exist_ok=True)
    BUILD_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log(f"Generated headers dir: {GENERATED_HEADER_DIR}")

    # ----------------------------------------------------------------------
    # Load control contract
    # ----------------------------------------------------------------------
    log(f"Loading contract: {CONTRACT_PATH}")

    if not CONTRACT_PATH.exists():
        raise FileNotFoundError(f"Contract not found: {CONTRACT_PATH}")

    contract_raw = json.loads(CONTRACT_PATH.read_text())

    # Canonical form (critical for stable hashing)
    contract_canonical = json.dumps(
        contract_raw,
        sort_keys=True,
        separators=(",", ":")
    ).encode()

    # ------------------------------------------------------------------
    # Collect source + include files for hashing
    # ------------------------------------------------------------------
    src_files = [f for f in SRC_DIR.rglob("*") if f.is_file()]
    inc_files = [f for f in INCLUDE_DIR.rglob("*")
                 if f.is_file() and GENERATED_DIR not in f.parents]

    all_files = sorted(src_files + inc_files)
    log(f"Found {len(all_files)} source+include files for hashing")

    # ------------------------------------------------------------------
    # Compute contract hash
    # ------------------------------------------------------------------
    contract_hasher = hashlib.sha1()
    contract_hasher.update(contract_canonical)
    contract_hash = contract_hasher.hexdigest()
    log(f"CONTRACT_HASH = {contract_hash}")

    # ------------------------------------------------------------------
    # Compute build hash
    # ------------------------------------------------------------------
    build_hasher = hashlib.sha1()

    # contract MUST affect firmware identity
    build_hasher.update(contract_canonical)

    for f in all_files:
        build_hasher.update(str(f.relative_to(PROJECT_DIR)).encode())
        build_hasher.update(f.read_bytes())

    build_hash = build_hasher.hexdigest()
    log(f"BUILD_HASH = {build_hash}")


    # ----------------------------------------------------------------------
    # Compute library hash
    # ----------------------------------------------------------------------
    lib_files = []

    for lib_dir in LIB_DEPS_DIR.glob("*/*"):
        if lib_dir.is_dir():
            lib_files.extend(lib_dir.glob("**/*.cpp"))
            lib_files.extend(lib_dir.glob("**/*.h"))
            lib_files.extend(lib_dir.glob("**/*.c"))
            lib_files.extend(lib_dir.glob("**/*.S"))

    lib_hasher = hashlib.sha1()
    for f in sorted(lib_files):
        lib_hasher.update(f.read_bytes())
    libs_hash = lib_hasher.hexdigest()
    log(f"LIBS_HASH = {libs_hash} ({len(lib_files)} files)")

    # ------------------------------------------------------------------
    # Prepare build hash directory
    # ------------------------------------------------------------------
    hash_dir = BUILD_LOG_DIR / build_hash
    hash_dir.mkdir(parents=True, exist_ok=True)
    log(f"Using snapshot: {hash_dir}")

    # ----------------------------------------------------------------------
    # Archive materialization (append-only, no shutil, no overwrite)
    # ----------------------------------------------------------------------

    # ---- Contract snapshot ----
    contract_dir = hash_dir / "contract_snapshot"
    contract_dir.mkdir(parents=True, exist_ok=True)

    contract_out = contract_dir / "control_contract.json"
    if not contract_out.exists():
        contract_out.write_bytes(CONTRACT_PATH.read_bytes())
        log("Wrote contract snapshot")

    # ---- Source snapshot ----
    src_out_dir = hash_dir / "src_snapshot"
    src_out_dir.mkdir(parents=True, exist_ok=True)

    for f in SRC_DIR.rglob("*"):
        if f.is_file():
            rel = f.relative_to(SRC_DIR)
            out = src_out_dir / rel
            out.parent.mkdir(parents=True, exist_ok=True)

            if not out.exists():
                out.write_bytes(f.read_bytes())

    log("Source snapshot materialized")

    # ---- Include snapshot ----
    inc_out_dir = hash_dir / "include_snapshot"
    inc_out_dir.mkdir(parents=True, exist_ok=True)

    for f in INCLUDE_DIR.rglob("*"):
        if f.is_file() and GENERATED_DIR not in f.parents:
            rel = f.relative_to(INCLUDE_DIR)
            out = inc_out_dir / rel
            out.parent.mkdir(parents=True, exist_ok=True)

            if not out.exists():
                out.write_bytes(f.read_bytes())

    log("Include snapshot materialized")

    # ------------------------------------------------------------------
    # Record library commit SHAs
    # ------------------------------------------------------------------
    LIB_COMMITS_FILE = hash_dir / "lib_commits.json"
    if not LIB_COMMITS_FILE.exists():
        log("Recording library commit SHAs")
        lib_commits = {}

        for lib_dir in LIB_DEPS_DIR.glob("*/*"):
            lib_name = lib_dir.name
            git_dir = lib_dir / ".git"
            commit = "unknown"

            if git_dir.exists():
                head = (git_dir / "HEAD").read_text().strip()
                if head.startswith("ref:"):
                    ref = git_dir / head.split(" ", 1)[1]
                    if ref.exists():
                        commit = ref.read_text().strip()
                else:
                    commit = head
            else:
                commit = "not-a-git-repo"

            lib_commits[lib_name] = {
                "path": str(lib_dir),
                "commit": commit
            }

        LIB_COMMITS_FILE.write_text(json.dumps(lib_commits, indent=2))
    else:
        log("Library commit record already exists — skipping")

    # ----------------------------------------------------------------------
    # Write build_hash.h
    # ----------------------------------------------------------------------
    build_timestamp_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    header_path = GENERATED_HEADER_DIR / "build_hash.h"
    header_path.write_text(
        "// Auto-generated by PlatformIO extra_pre.py\n"
        "#pragma once\n\n"
        f'#define BUILD_HASH "{build_hash}"\n'
        f'#define BUILD_TIMESTAMP_UTC "{build_timestamp_utc}"\n'
        f'#define LIBS_HASH "{libs_hash}"\n'
    )
    log(f"Wrote header: {header_path}")

    # Inject include path
    env.Append(CPPPATH=[str(GENERATED_HEADER_DIR)])  # type: ignore
    log("Include path injected")

    # ----------------------------------------------------------------------
    # Write contract.h
    # ----------------------------------------------------------------------
    contract_header = GENERATED_HEADER_DIR / "contract.h"
    contract_header.write_text(
    "// Auto-generated by PlatformIO extra_pre.py\n"
    "#pragma once\n\n"
    f'static constexpr const char* CONTRACT_HASH = "{contract_hash}";\n'
    f'static constexpr unsigned long CONTROL_PERIOD_MS = {contract_raw["control_loop"]["period_ms"]};\n'
    f'static constexpr unsigned long CONTROL_FREQUENCY_HZ = {contract_raw["control_loop"]["frequency_hz"]};\n'
    f'static constexpr unsigned long ODOM_PERIOD_MS = {contract_raw["odometry_loop"]["period_ms"]};\n'
    f'static constexpr unsigned long ODOM_FREQUENCY_HZ = {contract_raw["odometry_loop"]["frequency_hz"]};\n'
    f'static constexpr int CMD_MIN = {contract_raw["motor"]["cmd_min"]};\n'
    f'static constexpr int CMD_MAX = {contract_raw["motor"]["cmd_max"]};\n'
    f'static constexpr int MAX_ACCEL_TPS_PER_SECOND = {contract_raw["motor"]["max_accel_tps_per_second"]};\n'
)
    log(f"Wrote contract header: {contract_header}")

    # ----------------------------------------------------------------------
    # Write pre-build metadata JSON
    # ----------------------------------------------------------------------
    metadata = {
        "files": [str(f) for f in src_files],
        "build_hash": build_hash,
        "build_timestamp_utc": build_timestamp_utc,
        "libs_hash": libs_hash,
        "contract_hash": contract_hash,
        "library_files": [str(f) for f in lib_files],
        "contract_file": str(CONTRACT_PATH)
    }
    metadata_file = hash_dir / "info_pre.json"
    if not metadata_file.exists():
        metadata_file.write_text(json.dumps(metadata, indent=2))
    log(f"Wrote pre-build metadata JSON for hash {build_hash}")

    # ----------------------------------------------------------------------
    # Write CURRENTBUILD pointer
    # ----------------------------------------------------------------------
    current_build_file = BUILD_LOG_DIR / "CURRENTBUILD"
    current_build_file.write_text(build_hash)
    log(f"Wrote CURRENTBUILD pointer: {current_build_file}")

    log("ALL STEPS COMPLETE — pre-build finished successfully")
