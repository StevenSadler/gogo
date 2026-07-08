import os
import json
from dataclasses import dataclass


# ============================================================
# Path resolution (firmware-owned identity)
# ============================================================

PROJECT_ROOT = os.environ["GOGO_ROOT"]

IDENTITY_PATH = os.path.join(
    PROJECT_ROOT,
    "firmware",
    "config",
    "generated",
    "firmware_identity.json"
)

IDENTITY_PATH = os.path.normpath(IDENTITY_PATH)


# ============================================================
# Identity data structure
# ============================================================

@dataclass
class FirmwareIdentity:
    build_hash: str
    contract_hash: str


# ============================================================
# Loader
# ============================================================

def load_identity() -> FirmwareIdentity:
    if not os.path.exists(IDENTITY_PATH):
        raise FileNotFoundError(
            f"Firmware identity not found: {IDENTITY_PATH}"
        )

    with open(IDENTITY_PATH, "r") as f:
        data = json.load(f)

    return FirmwareIdentity(
        build_hash=data["build_hash"],
        contract_hash=data["contract_hash"]
    )