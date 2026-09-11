"""Validate the measurement set needed for a trustworthy dynamics simulator."""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field, ValidationError, model_validator

SCHEMA_VERSION = 1
EXPECTED_D2_CONTROL_RATE_HZ = 60


class Units(BaseModel):
    length: str = "m"
    mass: str = "kg"
    inertia: str = "kg*m^2"
    torque: str = "N*m"
    voltage: str = "V"


class Inertia(BaseModel):
    ixx: float | None = Field(default=None, gt=0)
    iyy: float | None = Field(default=None, gt=0)
    izz: float | None = Field(default=None, gt=0)


class DynamicsParams(BaseModel):
    schema_version: int = SCHEMA_VERSION
    robot_name: str = "DogV3"
    units: Units = Field(default_factory=Units)
    geometry_source: str
    recommended_engine: str = "MuJoCo"
    simulation_rate_hz: float = Field(gt=0)
    control_rate_hz: float = Field(gt=0)
    masses_kg: dict[str, float | None]
    link_inertia_kg_m2: dict[str, Inertia]
    actuator: dict[str, float | str | None]
    joint_friction: dict[str, float | None]
    foot_contact: dict[str, float | str | None]
    battery: dict[str, float | str | None]
    validation_targets: dict[str, float | str | list[int] | None] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _schema_supported(self) -> "DynamicsParams":
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"schema_version {self.schema_version} != supported {SCHEMA_VERSION}")
        return self


@dataclass(frozen=True)
class DynamicsReadiness:
    ready: bool
    missing: list[str]
    warnings: list[str]

    def to_dict(self) -> dict:
        return {"ready": self.ready, "missing": self.missing, "warnings": self.warnings}


def load_dynamics_params(path: str | Path) -> DynamicsParams:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return DynamicsParams.model_validate(raw)


def evaluate_readiness(params: DynamicsParams) -> DynamicsReadiness:
    data = params.model_dump()
    missing = sorted(_missing_paths(data))
    warnings: list[str] = []
    if params.recommended_engine.lower() != "mujoco":
        warnings.append("recommended_engine is not MuJoCo; document why another dynamics engine is better")
    if params.control_rate_hz != EXPECTED_D2_CONTROL_RATE_HZ:
        warnings.append(
            f"control_rate_hz should match deployed D2 Home-PC control rate "
            f"{EXPECTED_D2_CONTROL_RATE_HZ} Hz unless intentionally changed"
        )
    if params.simulation_rate_hz < 500:
        warnings.append("simulation_rate_hz below 500 Hz may hide contact instability")
    return DynamicsReadiness(ready=not missing, missing=missing, warnings=warnings)


def _missing_paths(value: Any, prefix: str = "") -> list[str]:
    out: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            out.extend(_missing_paths(child, child_prefix))
    elif isinstance(value, list):
        for i, child in enumerate(value):
            out.extend(_missing_paths(child, f"{prefix}[{i}]"))
    elif value is None:
        out.append(prefix)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Validate DogV3 dynamics-simulation measurement parameters")
    ap.add_argument("--params", default="simulation_params.example.json")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--allow-incomplete", action="store_true", help="return zero even when measurements are missing")
    args = ap.parse_args(argv)

    try:
        params = load_dynamics_params(args.params)
        readiness = evaluate_readiness(params)
    except (OSError, json.JSONDecodeError, ValidationError, ValueError) as e:
        print(f"dogv3-sim-params: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(readiness.to_dict(), indent=2))
    else:
        print(f"DogV3 dynamics params: {'READY' if readiness.ready else 'INCOMPLETE'}")
        if readiness.missing:
            print("Missing measurements:")
            for path in readiness.missing:
                print(f"  - {path}")
        if readiness.warnings:
            print("Warnings:")
            for warning in readiness.warnings:
                print(f"  - {warning}")

    if readiness.ready or args.allow_incomplete:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
