"""Local release regression checks. Never connect to a robot."""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('public_start', ROOT / 'start.py')
start = importlib.util.module_from_spec(spec)
spec.loader.exec_module(start)


def test_init_generates_distinct_credentials_without_commissioning(tmp_path):
    from dogv3.config.schema import RobotConfig
    template = (ROOT / 'robot_config.example.json').read_text()
    generated = []
    for name in ('builder_a', 'builder_b'):
        directory = tmp_path / name
        directory.mkdir()
        (directory / 'robot_config.example.json').write_text(template)
        assert start.initialize(directory)
        data = json.loads((directory / 'robot_config.json').read_text())
        assert not RobotConfig.model_validate(data).fully_commissioned()
        assert len(data['network']['token']) >= 32
        generated.append(data['network']['token'])
    assert generated[0] != generated[1]


def test_existing_configuration_is_never_overwritten(tmp_path):
    target = tmp_path / 'robot_config.json'
    before = b'{"existing_builder_calibration": true}\n'
    target.write_bytes(before)
    assert not start.initialize(tmp_path)
    assert target.read_bytes() == before


def test_operate_defaults_to_dry_without_hardware_arguments(monkeypatch):
    calls = []
    monkeypatch.setattr(start, 'initialize', lambda: False)
    monkeypatch.setattr('builtins.input', lambda prompt: '')
    monkeypatch.setattr(start, 'run_module', lambda *args: calls.append(args) or 0)
    assert start.main(['operate']) == 0
    assert calls == [('dogv3.runtime.operator_server', '--dry', '--config', 'robot_config.json')]


def test_flash_cancel_does_not_run_a_module(monkeypatch):
    answers = iter(['COM5', 'CANCEL'])
    calls = []
    monkeypatch.setattr(start, 'initialize', lambda: False)
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    monkeypatch.setattr(start, 'run_module', lambda *args: calls.append(args) or 0)
    assert start.main(['flash']) == 0
    assert calls == []
