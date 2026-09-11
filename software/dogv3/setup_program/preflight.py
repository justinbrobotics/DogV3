"""Host preflight for flash-and-test readiness.

This command does not move the robot. Upload and firmware verification only run
when the operator passes explicit flags.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from ..config.blob import ConfigBlobError, compile_config_blob, parse_config_blob
from ..config.loader import ConfigError, load_config
from ..gait.trot import GaitCommand
from ..kinematics.golden import build_golden_vectors
from ..simulation.trot_check import simulate_trot

Status = Literal["PASS", "WARN", "FAIL", "SKIP"]


@dataclass
class Check:
    name: str
    status: Status
    detail: str


@dataclass
class PreflightReport:
    checks: list[Check]

    @property
    def failed(self) -> bool:
        return any(c.status == "FAIL" for c in self.checks)

    @property
    def warned(self) -> bool:
        return any(c.status == "WARN" for c in self.checks)

    def exit_code(self, *, strict_warnings: bool = False) -> int:
        if self.failed:
            return 1
        if strict_warnings and self.warned:
            return 1
        return 0


def run_preflight(
    *,
    config_path: str,
    firmware_dir: str,
    env: str,
    port: str | None = None,
    run_tests: bool = True,
    run_build: bool = True,
    run_wifi_build: bool = False,
    run_native_test: bool = False,
    upload: bool = False,
    verify: bool = False,
    scan: bool = False,
    require_commissioned: bool = False,
) -> PreflightReport:
    checks: list[Check] = []
    root = Path(__file__).resolve().parents[2]
    firmware = (root / firmware_dir).resolve() if not Path(firmware_dir).is_absolute() else Path(firmware_dir)
    config = (root / config_path).resolve() if not Path(config_path).is_absolute() else Path(config_path)

    cfg = None
    try:
        cfg = load_config(config, require_commissioned=False)
        checks.append(Check("config schema", "PASS", f"loaded {config} schema_version={cfg.schema_version}"))
        commissioned = cfg.fully_commissioned()
        if commissioned:
            checks.append(Check("commissioning", "PASS", "12 servos assigned and verified"))
        elif require_commissioned:
            checks.append(Check("commissioning", "FAIL", "config is not fully commissioned; real motion is blocked"))
        else:
            checks.append(Check("commissioning", "WARN", "config is not fully commissioned; host-drive will refuse it"))
    except ConfigError as e:
        checks.append(Check("config schema", "FAIL", str(e)))

    if cfg is not None:
        checks.append(_check_config_blob(config, require_commissioned=require_commissioned))
        sim = simulate_trot(cfg, GaitCommand(vy=1.0), samples=64)
        sim_status: Status = "PASS" if sim.status == "pass" else ("WARN" if sim.status == "warn" else "FAIL")
        issues = ", ".join(issue.code for issue in sim.issues) or "no issues"
        checks.append(
            Check(
                "trot simulation",
                sim_status,
                f"{issues}; ik_error={sim.max_ik_error_mm:.3f}mm swing_peak={sim.swing_peak_clearance_mm:.1f}mm",
            )
        )

    vectors = build_golden_vectors()
    checks.append(
        Check(
            "golden vectors",
            "PASS",
            f"{len(vectors['ik_cases'])} IK cases, {len(vectors['gait_cases'])} gait cases, {len(vectors['count_cases'])} count cases",
        )
    )

    ports = list_serial_ports()
    if ports:
        rendered = "; ".join(f"{p['device']} ({p['description']})" for p in ports)
        checks.append(Check("serial ports", "PASS", rendered))
    else:
        checks.append(Check("serial ports", "WARN", "no serial ports detected; plug in ESP32 before upload/verify"))
    selected_port, port_check = select_serial_port(port, ports)
    checks.append(port_check)

    if run_tests:
        checks.append(_run_command("python tests", [sys.executable, "-m", "pytest", "-q"], cwd=root))
    else:
        checks.append(Check("python tests", "SKIP", "skipped by flag"))

    if not firmware.exists():
        checks.append(Check("firmware dir", "FAIL", f"not found: {firmware}"))
    elif run_build:
        checks.append(_run_command(f"firmware build {env}", [sys.executable, "-m", "platformio", "run", "-e", env], cwd=firmware))
        if run_wifi_build:
            checks.append(
                _run_command("firmware build esp32-wifi", [sys.executable, "-m", "platformio", "run", "-e", "esp32-wifi"], cwd=firmware)
            )
    else:
        checks.append(Check(f"firmware build {env}", "SKIP", "skipped by flag"))

    if run_native_test:
        checks.append(_run_command("native C++ golden test", [sys.executable, "-m", "platformio", "test", "-e", "native"], cwd=firmware))
    else:
        checks.append(Check("native C++ golden test", "SKIP", "skipped; requires gcc/g++ on PATH"))

    if upload:
        if not selected_port:
            checks.append(Check("firmware upload", "FAIL", "connect one ESP32 or pass --port COMx with --upload"))
        else:
            checks.append(
                _run_command(
                    "firmware upload",
                    [sys.executable, "-m", "platformio", "run", "-e", env, "-t", "upload", "--upload-port", selected_port],
                    cwd=firmware,
                )
            )
    else:
        checks.append(Check("firmware upload", "SKIP", "skipped; pass --upload --port COMx to flash"))

    if verify:
        if not selected_port:
            checks.append(Check("firmware verify", "FAIL", "connect one ESP32 or pass --port COMx with --verify"))
        else:
            cmd = [sys.executable, "-m", "dogv3.setup_program.verify_firmware", selected_port]
            if scan:
                cmd.append("--scan")
            checks.append(_run_command("firmware verify", cmd, cwd=root))
    else:
        checks.append(Check("firmware verify", "SKIP", "skipped; pass --verify --port COMx after flashing"))

    return PreflightReport(checks)


def list_serial_ports() -> list[dict[str, str]]:
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        return []
    return [
        {
            "device": str(p.device),
            "description": str(p.description),
            "hwid": str(p.hwid),
            "manufacturer": str(getattr(p, "manufacturer", "") or ""),
        }
        for p in list_ports.comports()
    ]


def select_serial_port(requested: str | None, ports: list[dict[str, str]]) -> tuple[str | None, Check]:
    if requested:
        known = {p["device"].upper() for p in ports}
        if known and requested.upper() not in known:
            return requested, Check("selected port", "WARN", f"{requested} was requested but is not in detected ports")
        return requested, Check("selected port", "PASS", f"using requested {requested}")
    if len(ports) == 1:
        device = ports[0]["device"]
        return device, Check("selected port", "PASS", f"auto-selected only detected port {device}")
    if not ports:
        return None, Check("selected port", "WARN", "no port selected")
    devices = ", ".join(p["device"] for p in ports)
    return None, Check("selected port", "WARN", f"multiple ports detected ({devices}); pass --port COMx")


def _run_command(name: str, cmd: list[str], *, cwd: Path) -> Check:
    try:
        proc = subprocess.run(cmd, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    except FileNotFoundError as e:
        return Check(name, "FAIL", f"{e}")
    except subprocess.TimeoutExpired:
        return Check(name, "FAIL", "command timed out after 300s")
    detail = _summarize_output(proc.stdout)
    return Check(name, "PASS" if proc.returncode == 0 else "FAIL", detail)


def _check_config_blob(config: Path, *, require_commissioned: bool) -> Check:
    try:
        blob = compile_config_blob(config, require_commissioned=require_commissioned)
        info, _ = parse_config_blob(blob)
    except (ConfigError, ConfigBlobError) as e:
        return Check("config blob", "FAIL", str(e))

    detail = f"schema={info.schema_version} payload={info.payload_len} crc32=0x{info.crc32:08x}"
    if info.fully_commissioned:
        return Check("config blob", "PASS", detail)
    return Check("config blob", "WARN", f"development blob only; config is not commissioned; {detail}")


def _summarize_output(output: str) -> str:
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    if not lines:
        return "no output"
    interesting = [
        line
        for line in lines
        if "passed" in line
        or "SUCCESS" in line
        or "FAILED" in line
        or "ERROR" in line
        or "not recognized" in line
        or "No module named" in line
    ]
    chosen = interesting[-3:] if interesting else lines[-3:]
    return " | ".join(chosen)


def print_report(report: PreflightReport) -> None:
    width = max(len(c.name) for c in report.checks) if report.checks else 0
    for check in report.checks:
        print(f"[{check.status:4}] {check.name.ljust(width)}  {check.detail}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Check DogV3 host, config, firmware build, and optional flash readiness")
    ap.add_argument("--config", default="robot_config.example.json")
    ap.add_argument("--firmware-dir", default="firmware/dogv3_mux")
    ap.add_argument("--env", default="esp32dev")
    ap.add_argument("--port", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--skip-tests", action="store_true")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--wifi-build", action="store_true", help="also build esp32-wifi")
    ap.add_argument("--native-test", action="store_true", help="run PlatformIO native C++ golden test; requires gcc/g++")
    ap.add_argument("--upload", action="store_true", help="flash firmware to --port")
    ap.add_argument("--verify", action="store_true", help="run verify_firmware against --port")
    ap.add_argument("--scan", action="store_true", help="include servo scan during firmware verify")
    ap.add_argument("--require-commissioned", action="store_true", help="fail if config is not ready for real motion")
    ap.add_argument("--strict-warnings", action="store_true", help="return non-zero on warnings")
    args = ap.parse_args(argv)

    report = run_preflight(
        config_path=args.config,
        firmware_dir=args.firmware_dir,
        env=args.env,
        port=args.port,
        run_tests=not args.skip_tests,
        run_build=not args.skip_build,
        run_wifi_build=args.wifi_build,
        run_native_test=args.native_test,
        upload=args.upload,
        verify=args.verify,
        scan=args.scan,
        require_commissioned=args.require_commissioned,
    )
    print_report(report)
    return report.exit_code(strict_warnings=args.strict_warnings)


if __name__ == "__main__":
    raise SystemExit(main())
