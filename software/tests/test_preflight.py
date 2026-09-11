"""Host preflight checks for flash-and-test readiness."""

from dogv3.config.loader import default_config, save_config
from dogv3.config.schema import ServoSpec
from dogv3.setup_program import preflight


def _commissioned_config_path(tmp_path):
    cfg = default_config()
    sid = 1
    for leg in ("BL", "BR", "FL", "FR"):
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


def test_preflight_safe_dry_run_warns_but_does_not_fail(monkeypatch):
    monkeypatch.setattr(
        preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    report = preflight.run_preflight(
        config_path="robot_config.example.json",
        firmware_dir="firmware/dogv3_mux",
        env="esp32dev",
        run_tests=False,
        run_build=False,
    )
    statuses = {check.name: check.status for check in report.checks}
    assert statuses["config schema"] == "PASS"
    assert statuses["commissioning"] == "WARN"
    assert statuses["config blob"] == "WARN"
    assert statuses["trot simulation"] == "WARN"
    assert statuses["serial ports"] == "PASS"
    assert statuses["selected port"] == "PASS"
    assert statuses["python tests"] == "SKIP"
    assert statuses["firmware build esp32dev"] == "SKIP"
    assert not report.failed
    assert report.exit_code() == 0


def test_preflight_strict_warnings_exit_nonzero(monkeypatch):
    monkeypatch.setattr(preflight, "list_serial_ports", lambda: [])
    report = preflight.run_preflight(
        config_path="robot_config.example.json",
        firmware_dir="firmware/dogv3_mux",
        env="esp32dev",
        run_tests=False,
        run_build=False,
    )
    assert report.warned
    assert report.exit_code(strict_warnings=True) == 1


def test_preflight_requires_commissioned_config(monkeypatch):
    monkeypatch.setattr(preflight, "list_serial_ports", lambda: [])
    report = preflight.run_preflight(
        config_path="robot_config.example.json",
        firmware_dir="firmware/dogv3_mux",
        env="esp32dev",
        run_tests=False,
        run_build=False,
        require_commissioned=True,
    )
    statuses = {check.name: check.status for check in report.checks}
    assert statuses["commissioning"] == "FAIL"
    assert statuses["config blob"] == "FAIL"
    assert report.failed


def test_preflight_config_blob_passes_for_commissioned_config(monkeypatch, tmp_path):
    monkeypatch.setattr(preflight, "list_serial_ports", lambda: [])
    report = preflight.run_preflight(
        config_path=str(_commissioned_config_path(tmp_path)),
        firmware_dir="firmware/dogv3_mux",
        env="esp32dev",
        run_tests=False,
        run_build=False,
        require_commissioned=True,
    )
    statuses = {check.name: check.status for check in report.checks}
    assert statuses["commissioning"] == "PASS"
    assert statuses["config blob"] == "PASS"
    assert not report.failed


def test_select_serial_port_warns_on_multiple_without_requested_port():
    selected, check = preflight.select_serial_port(
        None,
        [
            {"device": "COM5", "description": "USB Serial", "hwid": "A"},
            {"device": "COM9", "description": "Bluetooth", "hwid": "B"},
        ],
    )
    assert selected is None
    assert check.status == "WARN"
    assert "multiple ports" in check.detail


def test_select_serial_port_warns_when_requested_port_not_detected():
    selected, check = preflight.select_serial_port(
        "COM7",
        [{"device": "COM5", "description": "USB Serial", "hwid": "A"}],
    )
    assert selected == "COM7"
    assert check.status == "WARN"


def test_preflight_missing_config_fails(monkeypatch):
    monkeypatch.setattr(preflight, "list_serial_ports", lambda: [])
    report = preflight.run_preflight(
        config_path="missing_robot_config.json",
        firmware_dir="firmware/dogv3_mux",
        env="esp32dev",
        run_tests=False,
        run_build=False,
    )
    assert report.failed
    assert report.exit_code() == 1
