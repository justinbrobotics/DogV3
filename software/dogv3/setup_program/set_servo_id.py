"""Give a single servo a unique ID — the D1 prerequisite before commissioning.

Factory STS3215 servos all ship as ID 1, and a half-duplex bus cannot address
duplicate IDs. Connect ONE servo to the bus, give it a unique ID, then repeat for
each servo. Once every servo on a bus has a distinct ID, the commissioning GUI's
wiggle-to-identify / assign flow can map them to joints.

    dogv3-set-id --port COM5 --to 7

Safety: this scans first and refuses to write if more than one servo answers, so
you cannot accidentally set every servo on the bus to the same ID.
"""
from __future__ import annotations

import argparse
import sys

from ..driver.feetech import FeetechDriver, make_transport
from ..driver.mux import Bus


def set_servo_id(
    driver: FeetechDriver,
    *,
    new_id: int,
    bus: str | None = None,
    expected_from: int | None = None,
    force: bool = False,
) -> tuple[Bus, int, bool]:
    """Set the (single) connected servo's ID to *new_id*. Returns
    ``(bus, source_id, verified_ok)``. Raises RuntimeError if it is not safe."""
    if not (0 <= new_id <= 253):
        raise ValueError("new_id must be 0..253")
    buses = [Bus.from_name(bus)] if bus else [Bus.A, Bus.B]
    found = driver.scan(buses)
    present = [(b, sid) for b, ids in found.items() for sid in ids]
    if not present:
        raise RuntimeError("no servo responded — connect exactly one servo and check 12 V / USB")
    if len(present) > 1 and not force:
        ids = [sid for _, sid in present]
        raise RuntimeError(
            f"found {len(present)} servos {ids}; connect ONE servo at a time to set IDs safely "
            "(or pass --force if every listed servo should take the new ID)"
        )
    target_bus, src = present[0]
    if expected_from is not None and src != expected_from:
        raise RuntimeError(f"found servo ID {src}, not --from {expected_from}")
    if src == new_id:
        return target_bus, src, True
    return target_bus, src, driver.write_id(target_bus, src, new_id)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Give a single servo a unique ID (connect ONE servo at a time)"
    )
    ap.add_argument("--port", "--port-serial", dest="port", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--tcp", default=None, help="ESP32 WiFi bridge HOST[:PORT]")
    ap.add_argument("--to", type=int, required=True, help="new servo ID (0..253; must be unique per bus)")
    ap.add_argument("--bus", default=None, choices=["A", "B"], help="restrict to one bus (default: scan both)")
    ap.add_argument("--from", dest="expected_from", type=int, default=None, help="expected current ID (safety check)")
    ap.add_argument("--force", action="store_true", help="write even if multiple servos respond (DANGEROUS)")
    args = ap.parse_args(argv)
    if not (0 <= args.to <= 253):
        ap.error("--to must be 0..253")

    transport = make_transport(args.port, args.tcp)
    if transport is None:
        print("No link: pass --port COMx or --tcp HOST[:PORT]")
        return 1
    driver = FeetechDriver(transport)
    try:
        bus, src, ok = set_servo_id(
            driver, new_id=args.to, bus=args.bus, expected_from=args.expected_from, force=args.force
        )
    except (RuntimeError, ValueError) as e:
        print(f"dogv3-set-id: {e}", file=sys.stderr)
        return 2
    finally:
        close = getattr(transport, "close", None)
        if close:
            close()

    if src == args.to:
        print(f"servo already has ID {args.to} on bus {bus.name_letter}; nothing to do")
        return 0
    if ok:
        print(f"OK: servo {src} -> {args.to} on bus {bus.name_letter} (verified on the new ID)")
        return 0
    print(
        f"FAILED: wrote {src} -> {args.to} on bus {bus.name_letter} but it did not answer on the new ID",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
