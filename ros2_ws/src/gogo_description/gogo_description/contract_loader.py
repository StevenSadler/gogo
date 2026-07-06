from __future__ import annotations

import json
import os
from dataclasses import dataclass


# ============================================================
# Path resolution (firmware-owned contract)
# ============================================================

PROJECT_ROOT = os.environ["GOGO_ROOT"]
CONTRACT_PATH = os.path.join(
    PROJECT_ROOT,
    "firmware",
    "config",
    "contract",
    "control_contract.json"
)

CONTRACT_PATH = os.path.normpath(CONTRACT_PATH)


# ============================================================
# Contract data model (immutable)
# ============================================================

@dataclass(frozen=True)
class ControlLoop:
    period_ms: int

    def frequency_hz(self) -> float:
        return 1000.0 / self.period_ms


@dataclass(frozen=True)
class OdometryLoop:
    period_ms: int

    def frequency_hz(self) -> float:
        return 1000.0 / self.period_ms


@dataclass(frozen=True)
class Safety:
    watchdog_timeout_ms: int

    def watchdog_timeout_sec(self) -> float:
        return self.watchdog_timeout_ms / 1000.0


@dataclass(frozen=True)
class Serial:
    baudrate: int


@dataclass(frozen=True)
class Motor:
    cmd_max: int
    max_accel_tps_per_second: int
    cmd_unit: str


@dataclass(frozen=True)
class Contract:
    control_loop: ControlLoop
    odometry_loop: OdometryLoop
    safety: Safety
    serial: Serial
    motor: Motor


# ============================================================
# Loader (pure function, no caching)
# ============================================================

def load_contract() -> Contract:
    if not os.path.exists(CONTRACT_PATH):
        raise FileNotFoundError(
            f"[contract_loader] Contract not found:\n{CONTRACT_PATH}"
        )

    with open(CONTRACT_PATH, "r") as f:
        data = json.load(f)

    return Contract(
        control_loop=ControlLoop(**data["control_loop"]),
        odometry_loop=OdometryLoop(**data["odometry_loop"]),
        safety=Safety(**data["safety"]),
        serial=Serial(**data["serial"]),
        motor=Motor(**data["motor"]),
    )