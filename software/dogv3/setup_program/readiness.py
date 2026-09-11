"""Deliverable readiness report for the current DogV3 workspace.

This is a no-motion audit for the current D1-D3 scope:
D1 = Home-PC setup/commissioning, D2 = Home-PC operation and trot tuning,
D3 = digital-twin simulation readiness.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from pydantic import ValidationError

from ..config.blob import ConfigBlobError, compile_config_blob, parse_config_blob
from ..config.loader import ConfigError, load_config
from ..config.report import build_config_report
from ..gait.trot import GaitCommand
from ..simulation.candidates import evaluate_trot_candidates
from ..simulation.dynamics_params import evaluate_readiness, load_dynamics_params
from . import preflight

Status = Literal["PASS", "WARN", "FAIL", "SKIP"]


@dataclass(frozen=True)
class ReadinessItem:
    deliverable: str
    name: str
    status: Status
    detail: str


@dataclass
class ReadinessReport:
    items: list[ReadinessItem]

    @property
    def failed(self) -> bool:
        return any(item.status == "FAIL" for item in self.items)

    @property
    def warned(self) -> bool:
        return any(item.status == "WARN" for item in self.items)

    def exit_code(self, *, strict_warnings: bool = False) -> int:
        if self.failed:
            return 1
        if strict_warnings and self.warned:
            return 1
        return 0

    def to_dict(self) -> dict:
        return {
            "failed": self.failed,
            "warned": self.warned,
            "items": [asdict(item) for item in self.items],
        }


def run_readiness(
    *,
    config_path: str = "robot_config.example.json",
    params_path: str = "simulation_params.example.json",
    candidates_path: str = "trot_candidates.example.json",
    firmware_dir: str = "firmware/dogv3_mux",
    env: str = "esp32dev",
    port: str | None = None,
    run_tests: bool = True,
    run_build: bool = True,
    run_wifi_build: bool = False,
    run_native_test: bool = False,
    require_commissioned: bool = False,
    require_dynamics_ready: bool = False,
) -> ReadinessReport:
    items: list[ReadinessItem] = []
    cfg = None

    try:
        cfg = load_config(config_path, require_commissioned=False)
        config_report = build_config_report(cfg, command=GaitCommand(vy=1.0), samples=64)
        measurement_summary = ", ".join(
            f"{row.field}={row.value_mm:g}mm" for row in config_report.measurements
        )
        items.append(
            ReadinessItem(
                "D1",
                "joint-distance config",
                "PASS",
                measurement_summary,
            )
        )
        gait = cfg.gait_seed
        items.append(
            ReadinessItem(
                "D2",
                "trot tuning knobs",
                "PASS",
                (
                    f"body_height={gait.body_height:g}, step_length={gait.step_length:g}, "
                    f"step_height={gait.step_height:g}, cycle_period={gait.cycle_period:g}, "
                    f"duty_factor={gait.duty_factor:g}, stance_width_offset={gait.stance_width_offset:g}, "
                    f"turn_gain={gait.turn_gain:g}, swing_shape={gait.swing_shape}"
                ),
            )
        )
        sim = config_report.simulation
        items.append(
            ReadinessItem(
                "D2",
                "kinematic trot gate",
                _sim_status(sim["status"]),
                (
                    f"status={sim['status']} issues={','.join(sim['issue_codes']) or '-'} "
                    f"ik_error={sim['max_ik_error_mm']}mm swing_peak={sim['swing_peak_clearance_mm']}mm "
                    f"speed={sim['estimated_forward_speed_mm_s']}mm/s"
                ),
            )
        )
    except (ConfigError, ValidationError, ValueError) as e:
        items.append(ReadinessItem("D1", "robot_config schema", "FAIL", str(e)))

    if cfg is not None:
        items.append(_commissioning_item(cfg.fully_commissioned(), require_commissioned))
        items.append(_config_blob_item(config_path, require_commissioned=require_commissioned))

    items.extend(_candidate_items(config_path, candidates_path))
    items.extend(_dynamics_items(params_path, require_ready=require_dynamics_ready))
    items.extend(
        _preflight_items(
            config_path=config_path,
            firmware_dir=firmware_dir,
            env=env,
            port=port,
            run_tests=run_tests,
            run_build=run_build,
            run_wifi_build=run_wifi_build,
            run_native_test=run_native_test,
            require_commissioned=require_commissioned,
        )
    )
    return ReadinessReport(items)


def _commissioning_item(fully_commissioned: bool, require_commissioned: bool) -> ReadinessItem:
    if fully_commissioned:
        return ReadinessItem("D1", "commissioned config", "PASS", "12 slots assigned and verified")
    status: Status = "FAIL" if require_commissioned else "WARN"
    return ReadinessItem(
        "D1",
        "commissioned config",
        status,
        "not fully commissioned; real motion and --require-commissioned gates are blocked",
    )


def _config_blob_item(config_path: str, *, require_commissioned: bool) -> ReadinessItem:
    try:
        blob = compile_config_blob(config_path, require_commissioned=require_commissioned)
        info, _ = parse_config_blob(blob)
    except (ConfigError, ConfigBlobError) as e:
        return ReadinessItem("D1", "config blob", "FAIL", str(e))
    status: Status = "PASS" if info.fully_commissioned else "WARN"
    detail = f"schema={info.schema_version} payload={info.payload_len} crc32=0x{info.crc32:08x}"
    if not info.fully_commissioned:
        detail = "development blob only; config is not commissioned; " + detail
    return ReadinessItem("D1", "config blob", status, detail)


def _candidate_items(config_path: str, candidates_path: str) -> list[ReadinessItem]:
    try:
        rows = evaluate_trot_candidates(config_path, candidates_path)
    except (OSError, json.JSONDecodeError, ConfigError, ValidationError, ValueError) as e:
        return [ReadinessItem("D2", "named trot candidates", "FAIL", str(e))]
    if not rows:
        return [ReadinessItem("D2", "named trot candidates", "FAIL", "no candidate rows")]
    best = rows[0]
    return [
        ReadinessItem(
            "D2",
            "named trot candidates",
            _sim_status(best.status),
            (
                f"top={best.name} status={best.status} speed={best.estimated_forward_speed_mm_s:.1f}mm/s "
                f"swing={best.swing_peak_clearance_mm:.1f}mm issues={','.join(best.issue_codes) or '-'}"
            ),
        )
    ]


def _dynamics_items(params_path: str, *, require_ready: bool) -> list[ReadinessItem]:
    try:
        params = load_dynamics_params(params_path)
        readiness = evaluate_readiness(params)
    except (OSError, json.JSONDecodeError, ValidationError, ValueError) as e:
        return [ReadinessItem("D3", "MuJoCo dynamics params", "FAIL", str(e))]
    if readiness.ready:
        status: Status = "PASS"
        detail = "measured dynamics params complete"
    else:
        status = "FAIL" if require_ready else "WARN"
        preview = ", ".join(readiness.missing[:6])
        more = f" and {len(readiness.missing) - 6} more" if len(readiness.missing) > 6 else ""
        detail = f"incomplete measurements: {preview}{more}"
    if readiness.warnings:
        detail += "; warnings: " + "; ".join(readiness.warnings)
    return [ReadinessItem("D3", "MuJoCo dynamics params", status, detail)]


def _preflight_items(
    *,
    config_path: str,
    firmware_dir: str,
    env: str,
    port: str | None,
    run_tests: bool,
    run_build: bool,
    run_wifi_build: bool,
    run_native_test: bool,
    require_commissioned: bool,
) -> list[ReadinessItem]:
    report = preflight.run_preflight(
        config_path=config_path,
        firmware_dir=firmware_dir,
        env=env,
        port=port,
        run_tests=run_tests,
        run_build=run_build,
        run_wifi_build=run_wifi_build,
        run_native_test=run_native_test,
        upload=False,
        verify=False,
        scan=False,
        require_commissioned=require_commissioned,
    )
    wanted = {
        "golden vectors": "D2",
        "python tests": "D2",
        f"firmware build {env}": "D2",
        "firmware build esp32-wifi": "D2",
        "native C++ golden test": "D2",
        "serial ports": "D1",
        "selected port": "D1",
        "firmware upload": "D2",
        "firmware verify": "D2",
    }
    return [
        ReadinessItem(wanted[check.name], check.name, check.status, check.detail)
        for check in report.checks
        if check.name in wanted
    ]


def _sim_status(status: str) -> Status:
    if status == "pass":
        return "PASS"
    if status == "warn":
        return "WARN"
    return "FAIL"


def print_report(report: ReadinessReport) -> None:
    print("DogV3 deliverable readiness")
    ordered = ["D1", "D2", "D3"]
    leftovers = sorted({item.deliverable for item in report.items} - set(ordered))
    for deliverable in ordered + leftovers:
        rows = [item for item in report.items if item.deliverable == deliverable]
        if not rows:
            continue
        print(f"\n{deliverable}")
        for item in rows:
            print(f"  [{item.status:4}] {item.name:<24} {item.detail}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Audit DogV3 D1/D2/D3 readiness without moving hardware")
    ap.add_argument("--config", default="robot_config.example.json")
    ap.add_argument("--params", default="simulation_params.example.json")
    ap.add_argument("--candidates", default="trot_candidates.example.json")
    ap.add_argument("--firmware-dir", default="firmware/dogv3_mux")
    ap.add_argument("--env", default="esp32dev")
    ap.add_argument("--port", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--skip-tests", action="store_true")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--wifi-build", action="store_true", help="also build esp32-wifi")
    ap.add_argument("--native-test", action="store_true", help="run native C++ golden test; requires gcc/g++")
    ap.add_argument("--require-commissioned", action="store_true", help="fail if config is not motion-ready")
    ap.add_argument("--require-dynamics-ready", action="store_true", help="fail if MuJoCo measurement params are incomplete")
    ap.add_argument("--strict-warnings", action="store_true", help="return non-zero on warnings")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    report = run_readiness(
        config_path=args.config,
        params_path=args.params,
        candidates_path=args.candidates,
        firmware_dir=args.firmware_dir,
        env=args.env,
        port=args.port,
        run_tests=not args.skip_tests,
        run_build=not args.skip_build,
        run_wifi_build=args.wifi_build,
        run_native_test=args.native_test,
        require_commissioned=args.require_commissioned,
        require_dynamics_ready=args.require_dynamics_ready,
    )
    if args.json:
        print(json.dumps(report.to_dict(), indent=2))
    else:
        print_report(report)
    return report.exit_code(strict_warnings=args.strict_warnings)


if __name__ == "__main__":
    sys.exit(main())
