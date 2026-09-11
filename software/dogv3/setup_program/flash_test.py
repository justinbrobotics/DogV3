"""One-command onboard ESP32 flash and no-motion bench test.

This is the explicit hardware action path. It builds the firmware, uploads to
the selected ESP32, verifies VERSION/PING/SCAN, then runs the bench smoke test
that scans both buses and writes torque-off to discovered servos.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from ..config.loader import load_config
from ..driver.feetech import make_transport
from . import preflight
from .bench_smoke import SmokeReport, print_report as print_smoke_report, run_bench_smoke
from .verify_firmware import DEFAULT_EXPECTED


@dataclass
class FlashTestReport:
    selected_port: str | None
    preflight_report: preflight.PreflightReport
    config_path: str = ""
    smoke_report: SmokeReport | None = None
    smoke_error: str | None = None

    @property
    def ok(self) -> bool:
        return (
            self.preflight_report.exit_code() == 0
            and self.smoke_error is None
            and (self.smoke_report is None or self.smoke_report.ok)
        )

    def exit_code(self, *, strict_warnings: bool = False) -> int:
        preflight_code = self.preflight_report.exit_code(strict_warnings=strict_warnings)
        if preflight_code:
            return preflight_code
        if self.smoke_error:
            return 2
        if self.smoke_report is not None and not self.smoke_report.ok:
            return 1
        return 0


def run_flash_test(
    *,
    config_path: str = "robot_config.json",
    firmware_dir: str = "firmware/dogv3_mux",
    env: str = "esp32dev",
    port: str | None = None,
    run_tests: bool = True,
    run_wifi_build: bool = False,
    run_native_test: bool = False,
    require_commissioned: bool = False,
    strict_warnings: bool = False,
    run_smoke: bool = True,
    compare_config_ids: bool = True,
    first_flash: bool = False,
    expected_firmware: str = DEFAULT_EXPECTED,
    id_min: int = 1,
    id_max: int = 99,
) -> FlashTestReport:
    if first_flash:
        if require_commissioned:
            raise ValueError("--first-flash cannot be combined with --require-commissioned")
        compare_config_ids = False
        if config_path == "robot_config.json" and not Path(config_path).exists():
            config_path = "robot_config.example.json"

    gate_report = preflight.run_preflight(
        config_path=config_path,
        firmware_dir=firmware_dir,
        env=env,
        port=port,
        run_tests=run_tests,
        run_build=True,
        run_wifi_build=run_wifi_build,
        run_native_test=run_native_test,
        upload=False,
        verify=False,
        scan=False,
        require_commissioned=require_commissioned,
    )
    selected_port = _resolve_selected_port(port)
    if gate_report.exit_code(strict_warnings=strict_warnings) != 0:
        return FlashTestReport(selected_port=selected_port, preflight_report=gate_report, config_path=config_path)

    hardware_report = preflight.run_preflight(
        config_path=config_path,
        firmware_dir=firmware_dir,
        env=env,
        port=port,
        run_tests=False,
        run_build=False,
        run_wifi_build=False,
        run_native_test=False,
        upload=True,
        verify=True,
        scan=True,
        require_commissioned=require_commissioned,
    )
    preflight_report = _merge_preflight_reports(gate_report, hardware_report)
    report = FlashTestReport(selected_port=selected_port, preflight_report=preflight_report, config_path=config_path)
    if preflight_report.exit_code() != 0 or not run_smoke:
        return report

    if not selected_port:
        report.smoke_error = "no selected serial port for bench smoke"
        return report

    try:
        cfg = load_config(config_path, require_commissioned=False) if compare_config_ids else None
        transport = make_transport(port_serial=selected_port, read_timeout=0.05)
        if transport is None:
            report.smoke_error = "could not open serial transport"
            return report
        try:
            report.smoke_report = run_bench_smoke(
                transport,
                config=cfg,
                verify=True,
                expected_firmware=expected_firmware,
                id_range=range(id_min, id_max + 1),
                torque_off=True,
            )
        finally:
            close = getattr(transport, "close", None)
            if close:
                close()
    except Exception as e:
        report.smoke_error = str(e)
    return report


def _merge_preflight_reports(
    gate_report: preflight.PreflightReport,
    hardware_report: preflight.PreflightReport,
) -> preflight.PreflightReport:
    hardware_names = {"firmware upload", "firmware verify"}
    gate_checks = [check for check in gate_report.checks if check.name not in hardware_names]
    hardware_checks = [check for check in hardware_report.checks if check.name in hardware_names]
    return preflight.PreflightReport(gate_checks + hardware_checks)


def _resolve_selected_port(requested: str | None) -> str | None:
    ports = preflight.list_serial_ports()
    selected, _ = preflight.select_serial_port(requested, ports)
    return selected


def print_flash_test_report(report: FlashTestReport) -> None:
    print("== preflight / flash / verify ==")
    if report.config_path:
        print(f"config: {report.config_path}")
    preflight.print_report(report.preflight_report)
    if report.smoke_error:
        print("\n== bench smoke ==")
        print(f"bench smoke: ERROR ({report.smoke_error})")
    elif report.smoke_report is not None:
        print("\n== bench smoke ==")
        print_smoke_report(report.smoke_report)
    else:
        print("\n== bench smoke ==")
        print("bench smoke: SKIP")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Flash onboard ESP32 firmware and run no-motion bench smoke checks"
    )
    ap.add_argument("--config", default="robot_config.json")
    ap.add_argument("--firmware-dir", default="firmware/dogv3_mux")
    ap.add_argument("--env", default="esp32dev")
    ap.add_argument("--port", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--skip-tests", action="store_true")
    ap.add_argument("--wifi-build", action="store_true", help="also build esp32-wifi before upload")
    ap.add_argument("--native-test", action="store_true", help="run native C++ Unity test; requires gcc/g++")
    ap.add_argument("--require-commissioned", action="store_true", help="fail if config is not motion-ready")
    ap.add_argument(
        "--first-flash",
        action="store_true",
        help="first no-motion board test: use the example config when robot_config.json is absent and skip ID comparison",
    )
    ap.add_argument("--strict-warnings", action="store_true", help="return non-zero on warnings")
    ap.add_argument("--no-smoke", action="store_true", help="flash/verify only; skip bench smoke")
    ap.add_argument("--no-id-compare", action="store_true", help="scan buses without comparing IDs to config")
    ap.add_argument("--expected", default=DEFAULT_EXPECTED)
    ap.add_argument("--id-min", type=int, default=1)
    ap.add_argument("--id-max", type=int, default=99)
    args = ap.parse_args(argv)
    if args.first_flash and args.require_commissioned:
        ap.error("--first-flash cannot be combined with --require-commissioned")

    report = run_flash_test(
        config_path=args.config,
        firmware_dir=args.firmware_dir,
        env=args.env,
        port=args.port,
        run_tests=not args.skip_tests,
        run_wifi_build=args.wifi_build,
        run_native_test=args.native_test,
        require_commissioned=args.require_commissioned,
        strict_warnings=args.strict_warnings,
        run_smoke=not args.no_smoke,
        compare_config_ids=not args.no_id_compare,
        first_flash=args.first_flash,
        expected_firmware=args.expected,
        id_min=args.id_min,
        id_max=args.id_max,
    )
    print_flash_test_report(report)
    return report.exit_code(strict_warnings=args.strict_warnings)


if __name__ == "__main__":
    sys.exit(main())
