"""Measured robot_config seed creation."""

import json
from pathlib import Path

from dogv3.config.init_config import init_config, main
from dogv3.config.loader import default_config, load_config, save_config
from dogv3.config.schema import ServoSpec


ROOT = Path(__file__).resolve().parents[1]


def test_init_config_applies_geometry_and_gait_without_commissioning(tmp_path):
    out = tmp_path / "robot_config.json"
    cfg = init_config(
        source_path=ROOT / "robot_config.example.json",
        out_path=out,
        overrides=[
            "links_mm.L1_hip=52",
            "body_mm.fore_aft=275",
            "gait_seed.step_length=22",
            "swing_shape=sine",
        ],
    )
    loaded = load_config(out)

    assert cfg.links_mm.L1_hip == 52
    assert loaded.body_mm.fore_aft == 275
    assert loaded.gait_seed.step_length == 22
    assert loaded.gait_seed.swing_shape == "sine"
    assert not loaded.fully_commissioned()
    assert loaded.servos == {}
    assert loaded.legs["FL"].ids.hip is None


def test_init_config_applies_named_candidate(tmp_path):
    out = tmp_path / "robot_config.json"
    cfg = init_config(
        source_path=ROOT / "robot_config.example.json",
        out_path=out,
        candidates_path=ROOT / "trot_candidates.example.json",
        candidate_name="bench-conservative",
    )

    assert cfg.gait_seed.step_length == 20
    assert cfg.gait_seed.cycle_period == 1.0
    assert cfg.gait_seed.duty_factor == 0.7


def test_init_config_refuses_overwrite_without_force(tmp_path):
    out = tmp_path / "robot_config.json"
    save_config(default_config(), out)

    assert main(["--source", str(ROOT / "robot_config.example.json"), "--out", str(out)]) == 2
    assert main(["--source", str(ROOT / "robot_config.example.json"), "--out", str(out), "--force"]) == 0


def test_init_config_can_preserve_commissioning_when_requested(tmp_path):
    source = tmp_path / "source.json"
    out = tmp_path / "robot_config.json"
    cfg = default_config()
    sid = 1
    for leg in ("FL", "FR", "BL", "BR"):
        for joint in ("hip", "femur", "tibia"):
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}",
                verified={"assigned": True, "center": True, "direction": True, "range": True},
            )
            sid += 1
    save_config(cfg, source)

    cleared = init_config(source_path=source, out_path=out)
    assert not cleared.fully_commissioned()

    preserved_out = tmp_path / "preserved.json"
    preserved = init_config(source_path=source, out_path=preserved_out, preserve_commissioning=True)
    assert preserved.fully_commissioned()


def test_config_init_cli_json(tmp_path, capsys):
    out = tmp_path / "robot_config.json"
    assert main([
        "--source", str(ROOT / "robot_config.example.json"),
        "--out", str(out),
        "--candidate", "bench-conservative",
        "--set", "L2_femur=112",
        "--json",
    ]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["wrote"] == str(out)
    assert payload["links_mm"]["L2_femur"] == 112
    assert payload["gait_seed"]["step_length"] == 20
    assert not payload["fully_commissioned"]
