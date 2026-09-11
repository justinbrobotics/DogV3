"""Create or update a measured, uncommissioned ``robot_config.json`` seed."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from ..simulation.candidates import load_trot_candidates
from ..simulation.trot_check import parse_gait_overrides
from .loader import ConfigError, default_config, load_config, save_config
from .schema import RobotConfig

GEOMETRY_FIELDS = {
    "links_mm.L1_hip": ("links_mm", "L1_hip"),
    "links_mm.L2_femur": ("links_mm", "L2_femur"),
    "links_mm.L3_tibia": ("links_mm", "L3_tibia"),
    "body_mm.shoulder_lateral": ("body_mm", "shoulder_lateral"),
    "body_mm.fore_aft": ("body_mm", "fore_aft"),
}

GEOMETRY_ALIASES = {
    "L1_hip": "links_mm.L1_hip",
    "L2_femur": "links_mm.L2_femur",
    "L3_tibia": "links_mm.L3_tibia",
    "shoulder_lateral": "body_mm.shoulder_lateral",
    "fore_aft": "body_mm.fore_aft",
}


def init_config(
    *,
    source_path: str | Path | None = "robot_config.example.json",
    out_path: str | Path = "robot_config.json",
    overrides: list[str] | None = None,
    candidates_path: str | Path | None = None,
    candidate_name: str | None = None,
    preserve_commissioning: bool = False,
    force: bool = False,
) -> RobotConfig:
    out = Path(out_path)
    if out.exists() and not force:
        raise ConfigError(f"{out} already exists; pass --force to overwrite")
    cfg = _load_source_config(source_path)
    if not preserve_commissioning:
        cfg = _clear_commissioning(cfg)
    data = cfg.model_dump(mode="json")
    if candidate_name:
        if candidates_path is None:
            candidates_path = "trot_candidates.example.json"
        _apply_candidate(data, candidates_path, candidate_name)
    for item in overrides or []:
        _apply_override(data, item)
    cfg = RobotConfig.model_validate(data)
    save_config(cfg, out)
    return cfg


def _load_source_config(source_path: str | Path | None) -> RobotConfig:
    if source_path is None:
        return default_config()
    source = Path(source_path)
    if source.exists():
        return load_config(source, require_commissioned=False)
    if str(source_path) == "robot_config.example.json":
        return default_config()
    raise ConfigError(f"source config not found: {source}")


def _clear_commissioning(config: RobotConfig) -> RobotConfig:
    data = config.model_dump(mode="json")
    for leg in data["legs"].values():
        leg["ids"] = {"hip": None, "femur": None, "tibia": None}
    data["servos"] = {}
    return RobotConfig.model_validate(data)


def _apply_candidate(data: dict[str, Any], candidates_path: str | Path, candidate_name: str) -> None:
    candidates, _ = load_trot_candidates(candidates_path)
    selected = next((candidate for candidate in candidates if candidate.name == candidate_name), None)
    if selected is None:
        names = ", ".join(candidate.name for candidate in candidates)
        raise ValueError(f"candidate {candidate_name!r} not found; valid names: {names}")
    data["gait_seed"].update(selected.overrides)


def _apply_override(data: dict[str, Any], item: str) -> None:
    key, sep, raw_value = item.partition("=")
    if not sep:
        raise ValueError(f"expected FIELD=VALUE, got {item!r}")
    key = GEOMETRY_ALIASES.get(key, key)
    if key in GEOMETRY_FIELDS:
        section, field = GEOMETRY_FIELDS[key]
        data[section][field] = float(raw_value)
        return
    if key.startswith("gait_seed."):
        gait_key = key.split(".", 1)[1]
        parsed = parse_gait_overrides([f"{gait_key}={raw_value}"])
        data["gait_seed"].update(parsed)
        return
    parsed = parse_gait_overrides([f"{key}={raw_value}"])
    data["gait_seed"].update(parsed)


def _summary(config: RobotConfig, out_path: str | Path) -> dict[str, Any]:
    return {
        "wrote": str(out_path),
        "fully_commissioned": config.fully_commissioned(),
        "links_mm": config.links_mm.model_dump(mode="json"),
        "body_mm": config.body_mm.model_dump(mode="json"),
        "gait_seed": config.gait_seed.model_dump(mode="json"),
    }


def print_summary(config: RobotConfig, out_path: str | Path) -> None:
    summary = _summary(config, out_path)
    print(f"wrote measured config seed: {summary['wrote']}")
    print(f"commissioning: {'ready' if summary['fully_commissioned'] else 'not commissioned'}")
    print("geometry:")
    for section in ("links_mm", "body_mm"):
        for key, value in summary[section].items():
            print(f"  {section}.{key}: {value}")
    print("gait_seed:")
    for key, value in summary["gait_seed"].items():
        print(f"  {key}: {value}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Create a measured robot_config.json seed without pretending servo commissioning is complete"
    )
    ap.add_argument("--source", default="robot_config.example.json", help="source config seed")
    ap.add_argument("--out", default="robot_config.json", help="output robot_config path")
    ap.add_argument(
        "--set",
        dest="sets",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help=(
            "set geometry or gait field, e.g. --set links_mm.L2_femur=112 "
            "--set body_mm.fore_aft=272 --set gait_seed.step_length=20"
        ),
    )
    ap.add_argument("--candidates", default="trot_candidates.example.json")
    ap.add_argument("--candidate", default=None, help="apply named trot candidate before --set overrides")
    ap.add_argument("--preserve-commissioning", action="store_true", help="do not clear existing IDs/servo records")
    ap.add_argument("--force", action="store_true", help="overwrite --out if it exists")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    try:
        cfg = init_config(
            source_path=args.source,
            out_path=args.out,
            overrides=args.sets,
            candidates_path=args.candidates,
            candidate_name=args.candidate,
            preserve_commissioning=args.preserve_commissioning,
            force=args.force,
        )
    except (ConfigError, OSError, json.JSONDecodeError, ValidationError, ValueError) as e:
        print(f"dogv3-config-init: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(_summary(cfg, args.out), indent=2))
    else:
        print_summary(cfg, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
