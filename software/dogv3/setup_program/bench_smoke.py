"""Bench smoke test after flashing the onboard ESP32 firmware.

This command verifies the ESP32 mux link, scans both servo buses, optionally
compares discovered IDs against a config, and leaves discovered servos torque
off. It never commands motion.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from ..config.loader import ConfigError, load_config
from ..config.schema import RobotConfig
from ..driver.feetech import FeetechDriver, Transport, make_transport
from ..driver.mux import Bus
from .verify_firmware import DEFAULT_EXPECTED, verify_transport


@dataclass
class SmokeReport:
    firmware_ok: bool | None
    found: dict[Bus, list[int]]
    expected: dict[Bus, list[int]]
    missing: dict[Bus, list[int]]
    unexpected: dict[Bus, list[int]]
    torque_off_failed: dict[Bus, list[int]]

    @property
    def ok(self) -> bool:
        return (
            self.firmware_ok is not False
            and not any(self.missing.values())
            and not any(self.unexpected.values())
            and not any(self.torque_off_failed.values())
        )


def run_bench_smoke(
    transport: Transport,
    *,
    config: RobotConfig | None = None,
    verify: bool = True,
    expected_firmware: str = DEFAULT_EXPECTED,
    id_range: range = range(1, 100),
    torque_off: bool = True,
) -> SmokeReport:
    firmware_ok = verify_transport(transport, expected_firmware, do_scan=False) if verify else None
    driver = FeetechDriver(transport, reply_timeout=0.1)
    found = driver.scan(id_range=id_range)
    expected = expected_ids_by_bus(config) if config is not None else {Bus.A: [], Bus.B: []}
    missing, unexpected = compare_ids(found, expected) if config is not None else ({Bus.A: [], Bus.B: []}, {Bus.A: [], Bus.B: []})
    torque_off_failed = {Bus.A: [], Bus.B: []}
    if torque_off:
        for bus, ids in found.items():
            for sid in ids:
                if not driver.enable_torque(bus, sid, False):
                    torque_off_failed[bus].append(sid)
    return SmokeReport(
        firmware_ok=firmware_ok,
        found=found,
        expected=expected,
        missing=missing,
        unexpected=unexpected,
        torque_off_failed=torque_off_failed,
    )


def expected_ids_by_bus(config: RobotConfig) -> dict[Bus, list[int]]:
    expected: dict[Bus, list[int]] = {Bus.A: [], Bus.B: []}
    for leg, spec in config.legs.items():
        bus = Bus.from_name(spec.bus)
        for joint in config.joint_order:
            sid = config.id_for(leg, joint)
            if sid is not None:
                expected[bus].append(sid)
    return {bus: sorted(ids) for bus, ids in expected.items()}


def compare_ids(found: dict[Bus, list[int]], expected: dict[Bus, list[int]]) -> tuple[dict[Bus, list[int]], dict[Bus, list[int]]]:
    missing: dict[Bus, list[int]] = {Bus.A: [], Bus.B: []}
    unexpected: dict[Bus, list[int]] = {Bus.A: [], Bus.B: []}
    for bus in (Bus.A, Bus.B):
        found_set = set(found.get(bus, []))
        expected_set = set(expected.get(bus, []))
        missing[bus] = sorted(expected_set - found_set)
        unexpected[bus] = sorted(found_set - expected_set)
    return missing, unexpected


def _format_bus_ids(values: dict[Bus, list[int]]) -> str:
    return " ".join(f"{bus.name_letter}:[{','.join(str(i) for i in values.get(bus, []))}]" for bus in (Bus.A, Bus.B))


def print_report(report: SmokeReport) -> None:
    if report.firmware_ok is not None:
        print(f"firmware: {'PASS' if report.firmware_ok else 'FAIL'}")
    print(f"found: {_format_bus_ids(report.found)}")
    if any(report.expected.values()):
        print(f"expected: {_format_bus_ids(report.expected)}")
        print(f"missing: {_format_bus_ids(report.missing)}")
        print(f"unexpected: {_format_bus_ids(report.unexpected)}")
    print(f"torque-off failures: {_format_bus_ids(report.torque_off_failed)}")
    print(f"bench smoke: {'PASS' if report.ok else 'FAIL'}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Verify flashed ESP32 mux firmware and scan both servo buses without motion")
    ap.add_argument("--port-serial", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--tcp", default=None, help="ESP32 WiFi bridge, HOST[:PORT]")
    ap.add_argument("--config", default=None, help="optional robot_config.json for expected ID comparison")
    ap.add_argument("--expected", default=DEFAULT_EXPECTED)
    ap.add_argument("--id-min", type=int, default=1)
    ap.add_argument("--id-max", type=int, default=99)
    ap.add_argument("--no-verify", action="store_true", help="skip VERSION/PING firmware verification")
    ap.add_argument("--no-torque-off", action="store_true", help="do not write torque-off to discovered servos")
    args = ap.parse_args(argv)

    if not args.port_serial and not args.tcp:
        ap.error("give --port-serial COMx or --tcp HOST[:PORT]")
    cfg = None
    if args.config:
        try:
            cfg = load_config(Path(args.config), require_commissioned=False)
        except ConfigError as e:
            print(f"dogv3-bench-smoke: {e}", file=sys.stderr)
            return 2

    try:
        transport = make_transport(port_serial=args.port_serial, tcp=args.tcp, read_timeout=0.05)
        if transport is None:
            print("dogv3-bench-smoke: no transport", file=sys.stderr)
            return 2
        try:
            report = run_bench_smoke(
                transport,
                config=cfg,
                verify=not args.no_verify,
                expected_firmware=args.expected,
                id_range=range(args.id_min, args.id_max + 1),
                torque_off=not args.no_torque_off,
            )
        finally:
            close = getattr(transport, "close", None)
            if close:
                close()
    except Exception as e:
        print(f"dogv3-bench-smoke: {e}", file=sys.stderr)
        return 2

    print_report(report)
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
