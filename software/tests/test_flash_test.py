"""One-command flash/test orchestration."""

import pytest

from dogv3.config.loader import default_config
from dogv3.driver.mux import Bus
from dogv3.setup_program import flash_test, preflight
from dogv3.setup_program.bench_smoke import SmokeReport


class FakeTransport:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def test_flash_test_stops_before_smoke_when_preflight_fails(monkeypatch):
    def fake_preflight(**kwargs):
        return preflight.PreflightReport([preflight.Check("firmware upload", "FAIL", "no port")])

    monkeypatch.setattr(flash_test.preflight, "list_serial_ports", lambda: [])
    monkeypatch.setattr(flash_test.preflight, "run_preflight", fake_preflight)
    monkeypatch.setattr(
        flash_test,
        "run_bench_smoke",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("smoke should not run")),
    )

    report = flash_test.run_flash_test(config_path="robot_config.example.json", run_tests=False)
    assert report.selected_port is None
    assert report.smoke_report is None
    assert report.exit_code() == 1


def test_flash_test_runs_bench_smoke_after_success(monkeypatch):
    fake_transport = FakeTransport()
    calls = []

    def fake_preflight(**kwargs):
        calls.append(kwargs)
        if len(calls) == 1:
            assert kwargs["upload"] is False
            assert kwargs["verify"] is False
            assert kwargs["run_tests"] is False
            assert kwargs["run_build"] is True
            return preflight.PreflightReport([preflight.Check("firmware build esp32dev", "PASS", "ok")])
        assert kwargs["upload"] is True
        assert kwargs["verify"] is True
        assert kwargs["scan"] is True
        assert kwargs["run_tests"] is False
        assert kwargs["run_build"] is False
        return preflight.PreflightReport([preflight.Check("firmware verify", "PASS", "ok")])

    def fake_smoke(transport, **kwargs):
        assert transport is fake_transport
        assert kwargs["verify"] is True
        assert kwargs["torque_off"] is True
        return SmokeReport(
            firmware_ok=True,
            found={Bus.A: [], Bus.B: []},
            expected={Bus.A: [], Bus.B: []},
            missing={Bus.A: [], Bus.B: []},
            unexpected={Bus.A: [], Bus.B: []},
            torque_off_failed={Bus.A: [], Bus.B: []},
        )

    monkeypatch.setattr(
        flash_test.preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    monkeypatch.setattr(flash_test.preflight, "run_preflight", fake_preflight)
    monkeypatch.setattr(flash_test, "load_config", lambda *args, **kwargs: default_config())
    monkeypatch.setattr(flash_test, "make_transport", lambda **kwargs: fake_transport)
    monkeypatch.setattr(flash_test, "run_bench_smoke", fake_smoke)

    report = flash_test.run_flash_test(config_path="robot_config.example.json", run_tests=False)
    assert report.selected_port == "COM5"
    assert report.ok
    assert report.exit_code() == 0
    assert fake_transport.closed
    assert len(calls) == 2


def test_flash_test_can_skip_id_compare_and_smoke(monkeypatch):
    calls = []

    def fake_preflight(**kwargs):
        calls.append(kwargs)
        return preflight.PreflightReport([preflight.Check("firmware verify", "PASS", "ok")])

    monkeypatch.setattr(
        flash_test.preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    monkeypatch.setattr(flash_test.preflight, "run_preflight", fake_preflight)
    monkeypatch.setattr(
        flash_test,
        "load_config",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("config should not load")),
    )

    report = flash_test.run_flash_test(
        config_path="robot_config.example.json",
        run_tests=False,
        run_smoke=False,
        compare_config_ids=False,
    )
    assert report.selected_port == "COM5"
    assert report.smoke_report is None
    assert report.exit_code() == 0
    assert len(calls) == 2


def test_first_flash_uses_example_config_and_skips_id_compare(monkeypatch, tmp_path):
    # Hermetic: exercise the "robot_config.json absent -> example" branch from a
    # clean cwd, independent of whether a real robot_config.json exists in the repo.
    monkeypatch.chdir(tmp_path)
    calls = []

    def fake_preflight(**kwargs):
        calls.append(kwargs)
        return preflight.PreflightReport([preflight.Check("firmware verify", "PASS", "ok")])

    monkeypatch.setattr(
        flash_test.preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    monkeypatch.setattr(flash_test.preflight, "run_preflight", fake_preflight)
    monkeypatch.setattr(
        flash_test,
        "load_config",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("config should not load")),
    )

    report = flash_test.run_flash_test(
        run_tests=False,
        run_smoke=False,
        first_flash=True,
    )

    assert report.config_path == "robot_config.example.json"
    assert report.selected_port == "COM5"
    assert report.exit_code() == 0
    assert [call["config_path"] for call in calls] == [
        "robot_config.example.json",
        "robot_config.example.json",
    ]


def test_first_flash_cannot_require_commissioned_config():
    with pytest.raises(ValueError, match="first-flash"):
        flash_test.run_flash_test(first_flash=True, require_commissioned=True, run_tests=False)


def test_flash_test_strict_warnings_stops_before_upload(monkeypatch):
    calls = []

    def fake_preflight(**kwargs):
        calls.append(kwargs)
        return preflight.PreflightReport([preflight.Check("commissioning", "WARN", "not ready")])

    monkeypatch.setattr(
        flash_test.preflight,
        "list_serial_ports",
        lambda: [{"device": "COM5", "description": "USB Serial", "hwid": "TEST"}],
    )
    monkeypatch.setattr(flash_test.preflight, "run_preflight", fake_preflight)
    report = flash_test.run_flash_test(
        config_path="robot_config.example.json",
        run_tests=False,
        strict_warnings=True,
    )
    assert report.selected_port == "COM5"
    assert report.smoke_report is None
    assert report.exit_code(strict_warnings=True) == 1
    assert len(calls) == 1
    assert calls[0]["upload"] is False
