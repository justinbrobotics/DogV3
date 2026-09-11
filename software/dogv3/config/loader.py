"""Load / validate / save ``robot_config.json`` with fail-fast semantics."""
from __future__ import annotations

import json
from pathlib import Path

from pydantic import ValidationError

from .schema import RobotConfig


class ConfigError(Exception):
    """Raised on any config load/validation failure (fail-fast)."""


def load_config(path: str | Path, *, require_commissioned: bool = False) -> RobotConfig:
    """Load and validate the config. If *require_commissioned* (Runtime), also
    refuse configs with unassigned slots, inconsistent mappings, or unverified
    servos."""
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"robot_config not found: {p}")
    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise ConfigError(f"robot_config is not valid JSON: {e}") from e
    try:
        cfg = RobotConfig.model_validate(raw)
    except ValidationError as e:
        raise ConfigError(f"robot_config failed schema validation:\n{e}") from e

    if require_commissioned and not cfg.fully_commissioned():
        missing = []
        if not cfg.all_slots_assigned():
            missing.append("not all 12 slots assigned")
        if len(cfg.servos) != 12:
            missing.append(f"{len(cfg.servos)} servos defined (need 12)")
        unverified = cfg.unverified_servos()
        if unverified:
            missing.append(f"unverified servos: {unverified}")
        integrity = cfg.commissioning_integrity_errors()
        if integrity:
            missing.append("mapping integrity: " + "; ".join(integrity))
        raise ConfigError(
            "Runtime refuses config — commissioning incomplete: " + "; ".join(missing)
        )
    return cfg


def save_config(cfg: RobotConfig, path: str | Path) -> None:
    """Atomically write the validated config (Setup Program only)."""
    p = Path(path)
    # Re-validate before persisting.
    cfg = RobotConfig.model_validate(cfg.model_dump())
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(json.dumps(cfg.model_dump(), indent=2), encoding="utf-8")
    tmp.replace(p)


def default_config() -> RobotConfig:
    """Runnable seed config with example link lengths.

    The operator must replace these measurements during commissioning.
    """
    return RobotConfig.model_validate(
        {
            "schema_version": 1,
            "firmware_expected": "DOGV3-MUX v1.0",
            "frame": {"lateral": "X", "longitudinal": "Y", "up": "Z", "forward_sign": "+Y"},
            "buses": {
                "A": {"role": "rear", "esp32_uart": 1, "rx": 16, "tx": 17},
                "B": {"role": "front", "esp32_uart": 2, "rx": 26, "tx": 25},
            },
            "links_mm": {"L1_hip": 50, "L2_femur": 110, "L3_tibia": 130},
            "body_mm": {"shoulder_lateral": 190, "fore_aft": 270},
            "joint_order": ["hip", "femur", "tibia"],
            "legs": {
                "FL": {"bus": "B", "side": "left", "knee_sign": 1},
                "FR": {"bus": "B", "side": "right", "knee_sign": 1},
                "BL": {"bus": "A", "side": "left", "knee_sign": 1},
                "BR": {"bus": "A", "side": "right", "knee_sign": 1},
            },
            "servos": {},
            "gait_seed": {
                "gait_type": "trot",
                "body_height": 160,
                "step_length": 40,
                "step_height": 30,
                "cycle_period": 0.6,
                "duty_factor": 0.6,
                "stance_width_offset": 10,
                "body_sway_amp": 0,
                "body_offset_x": 0,
                "body_offset_y": 0,
                "turn_gain": 0.5,
                "max_fwd_speed": 80,
                "max_yaw": 0.6,
                "swing_shape": "parabola",
            },
        }
    )
