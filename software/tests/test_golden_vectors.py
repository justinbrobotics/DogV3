"""Golden-vector export and firmware C++ core guardrails."""

import re
from pathlib import Path

from dogv3.kinematics.golden import build_golden_vectors


ROOT = Path(__file__).resolve().parents[1]


def test_golden_vectors_are_deterministic():
    data = build_golden_vectors()
    assert data["schema"] == "dogv3-golden-v1"
    assert data["constants"]["neutral_count"] == 2048
    assert data["constants"]["counts_per_rad"] == 651.898647
    assert data["count_cases"] == [
        {"theta": 0.0, "invert": False, "count": 2048},
        {"theta": 0.5, "invert": False, "count": 2374},
        {"theta": 0.5, "invert": True, "count": 1722},
        {"theta": -1.0, "invert": False, "count": 1396},
    ]
    right_mid = data["ik_cases"][0]
    assert right_mid["angles_rad"] == [0.050274, -0.492933, 1.176005]
    assert right_mid["fk_mm"] == [40.0, 30.0, -200.0]
    phase_zero = data["gait_cases"][0]["targets"]
    assert phase_zero["FL"] == {"x": -10.0, "y": 20.0, "z": -160.0, "phase": 0.0, "in_stance": True}
    assert phase_zero["FR"] == {"x": 10.0, "y": -13.333333, "z": -160.0, "phase": 0.5, "in_stance": True}


def test_firmware_core_uses_float_math_only():
    core_files = [
        ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_core.h",
        ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_core.cpp",
    ]
    text = "\n".join(path.read_text(encoding="utf-8") for path in core_files)
    assert not re.search(r"\bdouble\b", text)
    for symbol in ("sinf", "cosf", "atan2f", "sqrtf", "acosf"):
        assert symbol in text


def test_platformio_native_golden_test_is_configured():
    ini = (ROOT / "firmware" / "dogv3_mux" / "platformio.ini").read_text(encoding="utf-8")
    test_file = ROOT / "firmware" / "dogv3_mux" / "test" / "test_dogv3_core" / "test_main.cpp"
    assert "[env:native]" in ini
    assert "dogv3_core.cpp" in ini
    assert "dogv3_state.cpp" in ini
    assert "dogv3_config_blob.cpp" in ini
    assert test_file.exists()


def test_firmware_state_scaffold_matches_safety_spec():
    header = (ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_state.h").read_text(encoding="utf-8")
    source = (ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_state.cpp").read_text(encoding="utf-8")
    native_test = (
        ROOT / "firmware" / "dogv3_mux" / "test" / "test_dogv3_core" / "test_main.cpp"
    ).read_text(encoding="utf-8")

    assert "INTENT_HOLD_STAND_MS = 300" in header
    assert "INTENT_DISARM_MS = 10000" in header
    assert "MODE_A_MUX" in header and "MODE_B_BRAIN" in header
    assert "canEnterModeB" in header
    assert "state.mode == MODE_A_MUX && !state.armed && !state.estop_latched && state.config_valid" in source
    assert "test_intent_safety_holds_then_disarms" in native_test
    assert "test_mode_b_requires_disarmed_valid_config" in native_test


def test_firmware_config_blob_validator_matches_host_blob_contract():
    header = (ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_config_blob.h").read_text(encoding="utf-8")
    source = (ROOT / "firmware" / "dogv3_mux" / "src" / "dogv3_config_blob.cpp").read_text(encoding="utf-8")
    native_test = (
        ROOT / "firmware" / "dogv3_mux" / "test" / "test_dogv3_core" / "test_main.cpp"
    ).read_text(encoding="utf-8")
    text = header + "\n" + source

    assert "CONFIG_BLOB_HEADER_LEN = 20" in header
    assert "CONFIG_BLOB_BAD_CRC" in header
    assert "validateConfigBlob" in header
    assert "configBlobCrc32" in header
    assert "0xEDB88320u" in source
    assert "readU16LE(data + 8)" in source
    assert "readU32LE(data + 16)" in source
    assert "test_config_blob_validator_accepts_valid_header" in native_test
    assert not re.search(r"\bdouble\b", text)
    assert "#include <vector>" not in text and "new " not in text
