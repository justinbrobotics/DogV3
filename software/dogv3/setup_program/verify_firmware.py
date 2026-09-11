"""`verify_firmware` host CLI (Stage 1 of commissioning).

Sends the ASCII ``VERSION`` command and asserts the reply equals the expected
firmware string, runs ``PING``->``PONG`` for liveness, and optionally ``SCAN`` to
list responding IDs per bus. Prints a clear PASS/FAIL.

Works over either link:
  - USB serial   :  dogv3-verify-firmware COM5 --scan
  - WiFi SoftAP  :  dogv3-verify-firmware --tcp 192.168.4.1:3333 --scan

Recommended workflow: flash + verify over USB while the cable is attached, then
deploy on battery/WiFi. The ``--tcp`` path lets you re-verify the deployed robot
over WiFi.
"""
from __future__ import annotations

import argparse
import sys
import time

DEFAULT_EXPECTED = "DOGV3-MUX v1.0"
DEFAULT_BAUD = 1_000_000


def _collect_text(transport, duration: float) -> list[str]:
    """Read text lines arriving within *duration* seconds from any transport."""
    deadline = time.monotonic() + duration
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = transport.read(256)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.01)
    text = buf.decode("ascii", errors="replace")
    return [ln.strip() for ln in text.replace("\r", "\n").split("\n") if ln.strip()]


def verify_transport(transport, expected: str = DEFAULT_EXPECTED, do_scan: bool = False) -> bool:
    """Run the verify sequence against an already-open transport (serial or TCP)."""
    ok = True
    transport.reset_input()

    transport.write(b"VERSION\n")
    lines = _collect_text(transport, 0.8)
    version = next((ln for ln in lines if "DOGV3-MUX" in ln), None)
    if version == expected:
        print(f"[ OK ] VERSION = {version!r}")
    else:
        print(f"[FAIL] VERSION = {version!r}, expected {expected!r}")
        ok = False

    transport.reset_input()
    transport.write(b"PING\n")
    lines = _collect_text(transport, 0.5)
    if any(ln.upper() == "PONG" for ln in lines):
        print("[ OK ] PING -> PONG (firmware alive)")
    else:
        print(f"[FAIL] PING got no PONG (lines={lines})")
        ok = False

    if do_scan:
        transport.reset_input()
        transport.write(b"SCAN\n")
        lines = _collect_text(transport, 4.0)
        scan_lines = [ln for ln in lines if ln.startswith("SCAN")]
        for ln in scan_lines:
            print(f"       {ln}")
        if not any("DONE" in ln for ln in scan_lines):
            print("[WARN] SCAN did not complete in time")

    print(f"\n==== verify_firmware: {'PASS' if ok else 'FAIL'} ====")
    return ok


def verify(port: str, expected: str = DEFAULT_EXPECTED, baud: int = DEFAULT_BAUD,
           do_scan: bool = False) -> bool:
    """Verify over a serial port (opens + closes it)."""
    from ..driver.feetech import SerialTransport

    t = SerialTransport(port, baudrate=baud)
    time.sleep(0.3)
    try:
        return verify_transport(t, expected, do_scan)
    finally:
        t.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Verify DogV3 ESP32 mux firmware")
    ap.add_argument("port", nargs="?", help="ESP32 serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--tcp", default=None, help="verify over WiFi instead, HOST[:PORT] (e.g. 192.168.4.1:3333)")
    ap.add_argument("--expected", default=DEFAULT_EXPECTED, help="expected FIRMWARE_VERSION string")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--scan", action="store_true", help="also run SCAN and list responding IDs")
    args = ap.parse_args(argv)

    if not args.port and not args.tcp:
        ap.error("give a serial port or --tcp HOST[:PORT]")

    try:
        if args.tcp:
            from ..driver.feetech import make_transport

            t = make_transport(tcp=args.tcp)
            try:
                ok = verify_transport(t, args.expected, args.scan)
            finally:
                t.close()
        else:
            ok = verify(args.port, args.expected, args.baud, args.scan)
    except Exception as e:
        print(f"[FAIL] could not verify: {e}")
        return 2
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
