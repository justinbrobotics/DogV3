"""Deliverable readiness matrix."""

import json

from dogv3.config.loader import default_config, save_config
from dogv3.config.schema import ServoSpec
from dogv3.setup_program import readiness


def _commissioned_config_path(tmp_path):
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
    path = tmp_path / "robot_config.json"
    save_config(cfg, path)
    return path


def _filled_params_path(tmp_path):
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    data = json.loads((root / "simulation_params.example.json").read_text(encoding="utf-8"))

    def fill_nulls(value):
        if isinstance(value, dict):
            return {key: fill_nulls(child) for key, child in value.items()}
        if isinstance(value, list):
            return [fill_nulls(child) for child in value]
        return 1.0 if value is None else value

    path = tmp_path / "simulation_params.json"
    path.write_text(json.dumps(fill_nulls(data)), encoding="utf-8")
    return path


def test_readiness_reports_uncommissioned_example_as_warnings(monkeypatch):
    monkeypatch.setattr(readiness.preflight, "list_serial_ports", lambda: [])
    report = readiness.run_readiness(run_tests=False, run_build=False)
    rows = {(item.deliverable, item.name): item for item in report.items}

    assert rows[("D1", "joint-distance config")].status == "PASS"
    assert rows[("D1", "commissioned config")].status == "WARN"
    assert rows[("D2", "trot tuning knobs")].status == "PASS"
    assert rows[("D3", "MuJoCo dynamics params")].status == "WARN"
    assert not report.failed
    assert report.exit_code() == 0
    assert report.exit_code(strict_warnings=True) == 1


def test_readiness_require_commissioned_fails_uncommissioned_example(monkeypatch):
    monkeypatch.setattr(readiness.preflight, "list_serial_ports", lambda: [])
    report = readiness.run_readiness(
        run_tests=False,
        run_build=False,
        require_commissioned=True,
    )
    rows = {(item.deliverable, item.name): item for item in report.items}

    assert rows[("D1", "commissioned config")].status == "FAIL"
    assert rows[("D1", "config blob")].status == "FAIL"
    assert report.exit_code() == 1


def test_readiness_can_pass_strict_config_and_dynamics_gates(monkeypatch, tmp_path):
    monkeypatch.setattr(
        readiness.preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    config_path = _commissioned_config_path(tmp_path)
    params_path = _filled_params_path(tmp_path)
    report = readiness.run_readiness(
        config_path=str(config_path),
        params_path=str(params_path),
        run_tests=False,
        run_build=False,
        require_commissioned=True,
        require_dynamics_ready=True,
    )
    rows = {(item.deliverable, item.name): item for item in report.items}

    assert rows[("D1", "commissioned config")].status == "PASS"
    assert rows[("D1", "config blob")].status == "PASS"
    assert rows[("D3", "MuJoCo dynamics params")].status == "PASS"
    assert not report.failed


def test_readiness_cli_json(monkeypatch, capsys):
    monkeypatch.setattr(readiness.preflight, "list_serial_ports", lambda: [])
    assert readiness.main(["--skip-tests", "--skip-build", "--json"]) == 0
    captured = capsys.readouterr()
    assert '"items"' in captured.out
    assert '"deliverable": "D1"' in captured.out
