"""Pi-side BNO085 attitude reader — IMU mounted at the robot's body center.

A library (consumed in-process by the operate server for attitude telemetry)
plus a bench CLI. No FastAPI app here.

    dogv3-pi-imu --config robot_config.json    # one attitude line per second
    dogv3-pi-imu --dry --json                  # single snapshot, no hardware

Design:
  - :class:`ImuReader` owns a daemon thread that polls the BNO08x rotation
    vector at ``rate_hz`` and converts the quaternion to aerospace ZYX euler
    angles (yaw about Z, then pitch about Y, then roll about X) with local
    math — no scipy/numpy.
  - Hardware imports (board/busio + adafruit_bno08x) are lazy so this module
    imports on any machine; ``--dry`` needs zero hardware and synthesizes a
    gentle sway so GUIs render live-looking data.
  - I2C errors never kill the thread: the snapshot flips to ok=False carrying
    the error string (last attitude retained), and init retries every 2 s —
    a rebooted/unplugged sensor recovers without restarting the service.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from ..config.loader import ConfigError, load_config

INIT_RETRY_S = 2.0        # re-attempt sensor bring-up after an I2C failure
_NO_SAMPLE = "no sample yet"
# Warm-up is a normal transient, not a fault; _wait_first_sample keeps
# waiting while the reader reports it.
_WARMING_UP = "warming up"

# --dry synthetic attitude: gentle sway so dashboards look alive, small enough
# that nothing downstream mistakes it for a fall.
DRY_SWAY_HZ = 0.2
DRY_ROLL_AMP_DEG = 3.0
DRY_PITCH_AMP_DEG = 2.0
DRY_YAW_DRIFT_DPS = 1.0


def quat_to_euler_zyx(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Quaternion (w, x, y, z) -> (roll, pitch, yaw) in degrees, aerospace ZYX.

    Pure and module-level so it is unit-testable. Input need not be normalized;
    a zero-norm quaternion raises ValueError.
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clamp guards the asin domain when |sin(pitch)| rounds past 1 at gimbal lock.
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def euler_zyx_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    """Inverse of :func:`quat_to_euler_zyx` — used by the dry path so the
    published quat and euler fields always agree."""
    hr = math.radians(roll_deg) / 2.0
    hp = math.radians(pitch_deg) / 2.0
    hy = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _dry_attitude(t: float) -> tuple[float, float, float]:
    """Synthetic (roll, pitch, yaw) degrees at elapsed time *t* seconds."""
    w = 2.0 * math.pi * DRY_SWAY_HZ
    roll = DRY_ROLL_AMP_DEG * math.sin(w * t)
    pitch = DRY_PITCH_AMP_DEG * math.sin(w * t + 1.3)  # phase offset: not lockstep
    yaw = ((DRY_YAW_DRIFT_DPS * t + 180.0) % 360.0) - 180.0
    return roll, pitch, yaw


class ImuReader:
    """Daemon-thread attitude poller. start()/stop() lifecycle; stop() joins.
    ``snapshot()`` is safe from any thread."""

    def __init__(self, *, dry: bool = False, i2c_bus: int = 1,
                 i2c_address: int = 0x4A, rate_hz: float = 60.0):
        self.dry = dry
        self.i2c_bus = int(i2c_bus)
        self.i2c_address = int(i2c_address)
        self.rate_hz = max(1.0, float(rate_hz))
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._sensor = None
        self._next_init = 0.0  # monotonic deadline for the next init attempt
        # Sample timestamps for the measured-rate estimate (~2 s window).
        self._stamps: deque = deque(maxlen=max(8, int(self.rate_hz * 2)))
        self._state: dict = {
            "ok": False, "roll_deg": None, "pitch_deg": None, "yaw_deg": None,
            "quat": None, "t": None, "error": _NO_SAMPLE,
        }

    # -- readout (any thread) ---------------------------------------------
    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            st = dict(self._state)
            stamps = list(self._stamps)
        rate = 0.0
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        return {
            "ok": st["ok"],
            "roll_deg": st["roll_deg"],
            "pitch_deg": st["pitch_deg"],
            "yaw_deg": st["yaw_deg"],
            "quat": st["quat"],
            "age_s": None if st["t"] is None else max(0.0, now - st["t"]),
            "rate_hz_measured": round(rate, 2),
            "error": st["error"],
        }

    # -- publishing (thread only) -------------------------------------------
    def _publish(self, w: float, x: float, y: float, z: float) -> None:
        roll, pitch, yaw = quat_to_euler_zyx(w, x, y, z)
        now = time.monotonic()
        with self._lock:
            self._stamps.append(now)
            self._state = {
                "ok": True, "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
                "quat": [w, x, y, z], "t": now, "error": None,
            }

    def _fail(self, msg: str) -> None:
        # Keep the last attitude (stale-but-known beats None) — age_s and
        # ok=False tell the consumer not to trust it.
        with self._lock:
            self._state = {**self._state, "ok": False, "error": msg}

    # -- hardware (lazy, Pi only) -----------------------------------------
    def _init_sensor(self):
        """Bring up the BNO08x. Raises with an actionable message if the
        CircuitPython stack is missing (e.g. running off-Pi without --dry)."""
        try:
            from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
            from adafruit_bno08x.i2c import BNO08X_I2C
        except ImportError as e:
            raise RuntimeError(
                "BNO08x driver missing — on the Pi run: "
                "pip install adafruit-circuitpython-bno08x adafruit-blinka "
                "(or use --dry off-robot)"
            ) from e
        if self.i2c_bus == 1:
            try:
                import board
                import busio
            except (ImportError, NotImplementedError) as e:
                raise RuntimeError(
                    "Blinka board support unavailable (not a Pi?) — "
                    "pip install adafruit-blinka, or use --dry"
                ) from e
            i2c = busio.I2C(board.SCL, board.SDA)
        else:
            try:
                from adafruit_extended_bus import ExtendedI2C
            except ImportError as e:
                raise RuntimeError(
                    f"I2C bus {self.i2c_bus} needs: pip install adafruit-extended-bus"
                ) from e
            i2c = ExtendedI2C(self.i2c_bus)
        sensor = BNO08X_I2C(i2c, address=self.i2c_address)
        sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        return sensor

    def _step_real(self) -> None:
        now = time.monotonic()
        if self._sensor is None:
            if now < self._next_init:
                return
            self._next_init = now + INIT_RETRY_S
            try:
                self._sensor = self._init_sensor()
            except Exception as e:
                self._fail(str(e))
                return
        try:
            x, y, z, w = self._sensor.quaternion  # adafruit order: (i, j, k, real)
        except Exception as e:
            # Bus glitch / unplug: drop the handle, surface the error, re-init.
            self._sensor = None
            self._next_init = time.monotonic() + INIT_RETRY_S
            self._fail(str(e))
            return
        try:
            self._publish(w, x, y, z)
        except ValueError as e:
            # The BNO085 answers with an all-zero rotation vector for the first
            # few seconds after the report is enabled. That is WARM-UP, not a bus
            # fault, and it must not drop the handle: re-initialising restarts the
            # warm-up, so the reader would never converge. Keep the sensor and
            # keep polling — the quaternion becomes valid on its own.
            self._fail(f"{_WARMING_UP} ({e})")

    # -- thread lifecycle ---------------------------------------------------
    def _run(self) -> None:
        period = 1.0 / self.rate_hz
        t0 = time.monotonic()
        while not self._stop_event.is_set():
            if self.dry:
                roll, pitch, yaw = _dry_attitude(time.monotonic() - t0)
                self._publish(*euler_zyx_to_quat(roll, pitch, yaw))
            else:
                self._step_real()
            # Event wait, not sleep: stop() unblocks immediately at any rate.
            self._stop_event.wait(period)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="dogv3-imu", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def _is_transient(error: object) -> bool:
    """True while the reader is still coming up rather than actually broken.

    The BNO085 reports an all-zero rotation vector for several seconds after the
    report is enabled, so a bench read that gave up at the first non-``ok``
    snapshot would report a hard failure on a perfectly healthy sensor."""
    if error is None or error == _NO_SAMPLE:
        return True
    return str(error).startswith(_WARMING_UP)


def _wait_first_sample(reader: ImuReader, timeout: float = 12.0) -> dict:
    """Block until the thread produced a sample or hit a real (non-warm-up) error."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snap = reader.snapshot()
        if snap["ok"] or not _is_transient(snap["error"]):
            return snap
        time.sleep(0.02)
    return reader.snapshot()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="dogv3-pi-imu",
        description="BNO085 attitude bench reader (library: dogv3.pi.imu.ImuReader)",
    )
    ap.add_argument("--config", type=Path, default=None,
                    help="robot_config.json (peripherals.imu seeds bus/address/rate)")
    ap.add_argument("--dry", action="store_true", help="synthetic attitude, no hardware")
    ap.add_argument("--rate", type=float, default=None, help="poll rate in Hz (overrides config)")
    ap.add_argument("--bus", type=int, default=None, help="I2C bus number (overrides config)")
    ap.add_argument("--address", type=lambda s: int(s, 0), default=None,
                    help="I2C address, e.g. 0x4A (overrides config)")
    ap.add_argument("--json", action="store_true", help="print one snapshot as JSON and exit")
    args = ap.parse_args(argv)

    bus, address, rate = 1, 0x4A, 60.0
    if args.config is not None:
        try:
            spec = load_config(args.config, require_commissioned=False).peripherals.imu
        except ConfigError as e:
            print(f"config error: {e}")
            return 2
        bus, address, rate = spec.i2c_bus, spec.i2c_address, float(spec.rate_hz)
    if args.bus is not None:
        bus = args.bus
    if args.address is not None:
        address = args.address
    if args.rate is not None:
        rate = args.rate

    reader = ImuReader(dry=args.dry, i2c_bus=bus, i2c_address=address, rate_hz=rate)
    reader.start()
    try:
        if args.json:
            snap = _wait_first_sample(reader)
            print(json.dumps(snap))
            return 0 if snap["ok"] else 1
        source = "dry (synthetic)" if args.dry else f"I2C bus {bus} addr 0x{address:02X}"
        print(f"IMU bench: {source} at {rate:g} Hz — Ctrl-C to stop")
        while True:
            time.sleep(1.0)
            snap = reader.snapshot()
            if snap["ok"]:
                print(f"roll {snap['roll_deg']:+7.2f}  pitch {snap['pitch_deg']:+7.2f}  "
                      f"yaw {snap['yaw_deg']:+8.2f}  ({snap['rate_hz_measured']:.1f} Hz)")
            else:
                print(f"IMU not ok: {snap['error']}")
    except KeyboardInterrupt:
        return 0
    finally:
        reader.stop()


if __name__ == "__main__":
    raise SystemExit(main())
