"""Versioned config blob compiler for onboard firmware input."""

import pytest

from dogv3.config.blob import (
    BLOB_MAGIC,
    BLOB_VERSION,
    ConfigBlobError,
    build_config_blob,
    compile_config_blob,
    parse_config_blob,
)
from dogv3.config.loader import ConfigError, default_config, save_config
from dogv3.config.schema import ServoSpec


def commissioned_config():
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
    return cfg


def test_config_blob_roundtrip_commissioned_config():
    cfg = commissioned_config()
    blob = build_config_blob(cfg)
    assert blob.startswith(BLOB_MAGIC)
    info, parsed = parse_config_blob(blob)
    assert info.blob_version == BLOB_VERSION
    assert info.schema_version == 1
    assert info.firmware_expected == "DOGV3-MUX v1.0"
    assert info.fully_commissioned
    assert parsed.model_dump() == cfg.model_dump()


def test_config_blob_is_deterministic():
    cfg = commissioned_config()
    assert build_config_blob(cfg) == build_config_blob(cfg)


def test_compile_config_blob_requires_commissioned_by_default(tmp_path):
    p = tmp_path / "robot_config.json"
    save_config(default_config(), p)
    with pytest.raises(ConfigError):
        compile_config_blob(p)
    blob = compile_config_blob(p, require_commissioned=False)
    info, _ = parse_config_blob(blob)
    assert not info.fully_commissioned


def test_config_blob_rejects_bad_crc():
    blob = bytearray(build_config_blob(commissioned_config()))
    blob[-1] ^= 0x01
    with pytest.raises(ConfigBlobError, match="crc32"):
        parse_config_blob(bytes(blob))


def test_config_blob_cli_writes_and_inspects(tmp_path, capsys):
    from dogv3.config.blob import main

    cfg_path = tmp_path / "robot_config.json"
    out = tmp_path / "robot_config.blob"
    save_config(commissioned_config(), cfg_path)
    assert main(["--config", str(cfg_path), "--out", str(out)]) == 0
    assert out.exists()
    assert main(["--inspect", str(out), "--json"]) == 0
    captured = capsys.readouterr()
    assert '"fully_commissioned": true' in captured.out
