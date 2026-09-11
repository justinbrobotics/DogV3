"""A small fixed experiment set; no new gait generator or hardware authority.

The three trot families share diagonal footfalls. The two crawl families use
the existing BL/FL/BR/FR order and differ by lateral body sway. Planned speed
is matched within trot and crawl groups; no physical ranking is implied.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from ..config.loader import load_config
from ..config.schema import GaitSeed, RobotConfig
from .guided_tuner import Candidate, screen_trajectory

REFERENCE_NAME = "turn-good-trot"
PACK_VERSION = 1
# id, family, gait, period, duty, stride, sway, lift pair, purpose
STYLES = [
    ("relaxed", "Relaxed trot", "trot", 1.2, .65, 35., 0., (18., 22.),
     "Longer swing time gives the servos more time to follow."),
    ("compact", "Compact trot", "trot", 1.0, .65, 35./1.2, 0., (14., 18.),
     "Shorter, quicker steps at the same planned speed as relaxed trot."),
    ("overlap", "More-support trot", "trot", 1.6, .75, 35./(.65*1.2)*(.75*1.6), 0., (18., 22.),
     "More time with four feet down; longer steps maintain the planned pace."),
    ("deliberate", "Deliberate crawl", "crawl", 3., .82, 30., 0., (18., 22.),
     "One leg swings at a time, with a pause on all four feet and no added body sway."),
    ("weight-shift", "Weight-shift crawl", "crawl", 3., .82, 30., 12., (18., 22.),
     "Adds 12 mm of phase-linked lateral body sway to the same crawl. Its balance benefit is unproven."),
]


def catalog() -> list[dict]:
    return [{"id": f"{key}-{variant}", "name": f"{family} / {lift:g} mm lift",
             "family": family, "why": why + (" Start with this version." if variant == "low"
                 else " Try this extra clearance only if the lower-lift version scuffs."),
             "profile_name": f"test-{key}-{variant}", "variant": variant}
            for key, family, gait, period, duty, stride, sway, lifts, why in STYLES
            for variant, lift in zip(("low", "clear"), lifts)]


def build_test_pack(config: RobotConfig, reference: GaitSeed, hz: float = 60, *, use_saved=False) -> list[Candidate]:
    base = reference.model_dump()
    entries = iter(catalog())
    out = []
    for key, family, gait, period, duty, stride, sway, lifts, why in STYLES:
        for lift in lifts:
            meta = next(entries)
            # Keep the operator's commissioned stance height, width and static
            # trims. Joint directions, calibration, servo registers are untouched.
            seed = GaitSeed.model_validate({**base, "gait_type": gait,
                "cycle_period": period, "duty_factor": duty, "step_length": round(stride, 6),
                "step_height": lift, "body_sway_amp": sway, "swing_shape": "cycloid",
                "swing_retract": 1., "stance_dip": 0., "accel_limit": 2.,
                "max_fwd_speed": 60. if gait == "trot" else 20., "max_yaw": .3 if gait == "trot" else .1})
            # On the robot, use the exact transferred examples. Regenerating
            # from a different Pi reference would silently change their stance.
            if use_saved:
                saved = config.gait_profiles.get(meta["profile_name"])
                if saved is not None:
                    seed = saved.model_copy(deep=True)
                    if seed.gait_type != gait:
                        raise ValueError(f"{meta['profile_name']} has the wrong footfall type.")
            screen = screen_trajectory(config, seed, hz, sample_hz=600)
            out.append(Candidate(id=meta["id"], name=meta["name"], family=family,
                profile_name=meta["profile_name"], why=meta["why"], seed=seed, screen=screen,
                changes={k: {"before": base[k], "after": v} for k, v in seed.model_dump().items()
                         if v != base[k]}, status="proposed" if screen["passed"] else "screened_out"))
    return out


def install_local_library(path: Path, *, hz=60) -> dict:
    """Explicit offline maintenance: archive first, then replace only profiles.

    Never called at server startup or implicitly by a GET. The caller must stop
    a server using this file before running maintenance. All old profiles are
    preserved verbatim, including the kept reference; no credentials are copied.
    """
    original = path.read_bytes()
    raw = json.loads(original)
    cfg = load_config(path)
    reference = cfg.gait_profiles.get(REFERENCE_NAME)
    if reference is None:
        raise ValueError(f"Keep an explicit {REFERENCE_NAME!r} reference before replacing this library.")
    candidates = build_test_pack(cfg, reference, hz)
    blocked = [c.name for c in candidates if not c.screen["passed"]]
    if blocked:
        raise ValueError("Library unchanged; these candidates failed screening: " + ", ".join(blocked))
    old = raw.get("gait_profiles", {})
    profiles = {REFERENCE_NAME: old[REFERENCE_NAME],
                **{c.profile_name: c.seed.model_dump() for c in candidates}}
    if profiles == old:
        return {"changed": False, "profiles": len(profiles), "candidates": [c.model_dump() for c in candidates]}
    archive_dir = path.with_name(path.stem + ".gait-archive")
    archive_dir.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(json.dumps(old, sort_keys=True).encode()).hexdigest()[:16]
    archive = archive_dir / f"profiles-{digest}.json"
    archive_data = {"version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "reason": "Archived legacy library for a smaller physical test set; not a physical bad-gait classification.",
        "gait_profiles": old}
    if not archive.exists():
        with archive.open("x", encoding="utf-8") as f:
            json.dump(archive_data, f, indent=2, allow_nan=False)
    if json.loads(archive.read_text(encoding="utf-8")).get("gait_profiles") != old:
        raise ValueError("Archive verification failed; library unchanged.")
    if path.read_bytes() != original:
        raise ValueError("Config changed while checking candidates; library unchanged.")
    raw["gait_profiles"] = profiles
    tmp = path.with_name(path.name + ".library-tmp")
    tmp.write_text(json.dumps(raw, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    # Validate before replacing the user's config, preserving all other keys.
    load_config(tmp)
    tmp.replace(path)
    return {"changed": True, "archived_profiles": len(old), "profiles": len(profiles),
            "archive": str(archive.resolve()), "candidates": [c.model_dump() for c in candidates]}
