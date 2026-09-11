"""Onboard camera service — MJPEG over HTTP from the Pi.

Serves the CSI camera (OV5647) as a browser-consumable MJPEG stream on the
robot LAN, plus a minimal status page:

    dogv3-pi-camera --config robot_config.json     # picamera2 hardware encode
    dogv3-pi-camera --dry                          # synthetic frames, no hardware

Endpoints:
  GET /             status page embedding the live stream
  GET /stream.mjpg  multipart/x-mixed-replace MJPEG (boundary "frame")
  GET /api/status   {mode, width, height, fps, frames_served}

Design:
  - One latest-frame mailbox (:class:`FrameHolder`, ``threading.Condition``)
    sits between the frame source and every HTTP client — the pattern of
    picamera2 examples/mjpeg_server_2.py. Each client always takes the newest
    frame, so slow or stalled viewers skip frames instead of buffering; memory
    is bounded at one frame regardless of client count or speed.
  - Real mode: picamera2 + MJPEGEncoder (hardware JPEG on Pi 0-4) writing into
    the holder via FileOutput. Dry mode: a publisher thread alternates two
    embedded solid-color JPEGs at the configured fps through the same holder,
    so the streaming path is identical in both modes and testable with zero
    hardware.
  - picamera2 is apt-only (python3-picamera2), not pip-installable, so the
    import is lazy and the venv on the Pi must be created with
    --system-site-packages.
"""
from __future__ import annotations

import argparse
import base64
import io
import threading

from ..config.loader import ConfigError, load_config
from ..config.schema import CameraSpec

try:
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn
except Exception:  # pragma: no cover - FastAPI optional at import time
    FastAPI = None  # type: ignore


STREAM_BOUNDARY = "frame"
# Generator wake-up period while no new frame arrives: keeps the stream
# responsive to shutdown/disconnect even if the source stops producing.
FRAME_WAIT_S = 1.0

# Two tiny valid JPEGs (solid-color 32x32, quality 70) for --dry, alternated
# so the stream visibly changes. They enter through the same FrameHolder the
# hardware encoder writes to — the streaming code has no dry/real branch.
_FRAME_A = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAoHBwgHBgoICAgLCgoLDhgQDg0NDh0VFhEYIx8l"
    "JCIfIiEmKzcvJik0KSEiMEExNDk7Pj4+JS5ESUM8SDc9Pjv/2wBDAQoLCw4NDhwQEBw7KCIo"
    "Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozv/wAAR"
    "CAAgACADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA"
    "AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkK"
    "FhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWG"
    "h4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl"
    "5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA"
    "AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYk"
    "NOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOE"
    "hYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk"
    "5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDlaKKK+mPnQooooAKKKKACiiigD//Z"
)
_FRAME_B = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAoHBwgHBgoICAgLCgoLDhgQDg0NDh0VFhEYIx8l"
    "JCIfIiEmKzcvJik0KSEiMEExNDk7Pj4+JS5ESUM8SDc9Pjv/2wBDAQoLCw4NDhwQEBw7KCIo"
    "Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozv/wAAR"
    "CAAgACADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA"
    "AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkK"
    "FhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWG"
    "h4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl"
    "5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA"
    "AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYk"
    "NOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOE"
    "hYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk"
    "5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDJooor5k+iCiiigAooooAKKKKAP//Z"
)


class CameraUnavailable(Exception):
    """Raised when the real camera stack cannot start (with the fix)."""


class FrameHolder(io.BufferedIOBase):
    """Latest-frame mailbox between the source and all stream clients.

    ``io.BufferedIOBase`` so picamera2's FileOutput can write straight into it.
    ``seq`` increments per published frame; readers wait for a seq change, so
    they naturally drop frames they were too slow to fetch."""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.frame: bytes | None = None
        self.seq = 0

    def write(self, buf) -> int:  # called by FileOutput / DryFrameSource
        with self.condition:
            self.frame = bytes(buf)
            self.seq += 1
            self.condition.notify_all()
        return len(buf)

    def next_frame(self, seen: int, timeout: float = FRAME_WAIT_S) -> tuple[bytes | None, int]:
        """Block until a frame newer than *seen* exists (or timeout); return
        (frame, seq). On timeout the caller sees seq == seen and retries."""
        with self.condition:
            self.condition.wait_for(lambda: self.seq != seen, timeout=timeout)
            return self.frame, self.seq


class DryFrameSource:
    """--dry publisher: alternates the two embedded JPEGs at *fps*."""

    def __init__(self, holder: FrameHolder, *, fps: int) -> None:
        self.holder = holder
        self.fps = max(1, int(fps))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="dogv3-cam-dry", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        period = 1.0 / self.fps
        i = 0
        while True:
            self.holder.write(_FRAME_A if i % 2 == 0 else _FRAME_B)
            i += 1
            if self._stop.wait(period):
                return

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


class PicameraSource:
    """Real camera: picamera2 + MJPEGEncoder (hardware JPEG) into the holder.

    All picamera2 imports live inside start() — the package is apt-only and
    absent on the Home PC / CI."""

    def __init__(self, holder: FrameHolder, *, width: int, height: int,
                 fps: int, quality: int) -> None:
        self.holder = holder
        self.width = width
        self.height = height
        self.fps = max(1, int(fps))
        self.quality = quality
        self._picam = None

    def start(self) -> None:
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import MJPEGEncoder, Quality
            from picamera2.outputs import FileOutput
        except ImportError as e:
            raise CameraUnavailable(
                "picamera2 is not importable. On the Pi: sudo apt install "
                "python3-picamera2, and create the venv with "
                "--system-site-packages (picamera2 is apt-only, not on PyPI). "
                "For hardware-free operation run with --dry."
            ) from e
        # MJPEGEncoder's rate control takes a Quality bucket, not 0-100; map
        # the CameraSpec hint onto the nearest bucket.
        q = self.quality
        quality_enum = (Quality.LOW if q <= 40 else Quality.MEDIUM if q <= 60
                        else Quality.HIGH if q <= 85 else Quality.VERY_HIGH)
        frame_us = int(1_000_000 / self.fps)
        self._picam = Picamera2()
        self._picam.configure(self._picam.create_video_configuration(
            main={"size": (self.width, self.height)},
            controls={"FrameDurationLimits": (frame_us, frame_us)},
        ))
        self._picam.start_recording(MJPEGEncoder(), FileOutput(self.holder),
                                    quality=quality_enum)

    def stop(self) -> None:
        if self._picam is not None:
            try:
                self._picam.stop_recording()
            finally:
                self._picam.close()
                self._picam = None


def build_app(holder: FrameHolder, *, mode: str, width: int, height: int, fps: int):
    app = FastAPI(title="DogV3 Pi Camera")
    served = {"frames": 0}
    served_lock = threading.Lock()

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return INDEX_HTML

    @app.get("/api/status")
    def status() -> dict:
        return {"mode": mode, "width": width, "height": height, "fps": fps,
                "frames_served": served["frames"]}

    @app.get("/stream.mjpg")
    def stream(frames: int | None = None) -> StreamingResponse:
        # ``frames=N`` ends the response after N parts — a bounded grab for
        # tests and debugging (TestClient cannot consume an endless stream).
        # Browsers omit it and get the endless live stream.
        def gen():
            seen = 0
            sent = 0
            idle = 0
            while frames is None or sent < frames:
                frame, latest = holder.next_frame(seen)
                if frame is None or latest == seen:
                    # Source not producing. Endless streams keep waiting for
                    # it to (re)start; bounded grabs give up so they finish.
                    idle += 1
                    if frames is not None and idle >= 3:
                        return
                    continue
                seen = latest
                idle = 0
                sent += 1
                with served_lock:
                    served["frames"] += 1
                # One self-contained part per yield: bounded memory, and a
                # slow client blocking in send simply skips to the newest
                # frame on the next next_frame() call.
                yield (
                    b"--" + STREAM_BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    + frame + b"\r\n"
                )

        return StreamingResponse(
            gen(),
            media_type=f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY}",
        )

    return app


INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8>
<title>DogV3 Camera</title>
<style>
 :root{color-scheme:dark}
 body{font-family:system-ui,sans-serif;margin:0;background:#0e1116;color:#e6e6e6;
  display:flex;flex-direction:column;align-items:center;gap:10px;padding:16px}
 h1{font-size:18px;margin:0}
 img{max-width:100%;border:1px solid #2a2f37;border-radius:6px;background:#000}
 #meta{font-size:13px;color:#9aa4b2}
</style></head><body>
<h1>DogV3 &middot; Pi Camera</h1>
<img src="/stream.mjpg" alt="live camera stream">
<div id=meta>loading&hellip;</div>
<script>
fetch('/api/status').then(r=>r.json()).then(s=>{
 document.getElementById('meta').textContent=s.mode+' \\u00b7 '+s.width+'x'+s.height+' @ '+s.fps+' fps';
});
</script></body></html>"""


def run(config_path: str | None, *, host: str, port: int | None,
        width: int | None, height: int | None, fps: int | None,
        quality: int | None, dry: bool) -> int:
    if FastAPI is None:
        print("FastAPI/uvicorn not installed; cannot run the camera service.")
        return 1

    # Config (optional) supplies defaults; CLI flags override field-by-field.
    spec = CameraSpec()
    cfg_port = 8080
    if config_path is not None:
        try:
            cfg = load_config(config_path, require_commissioned=False)
        except ConfigError as e:
            print(f"Cannot start camera service: {e}")
            return 1
        spec = cfg.peripherals.camera
        cfg_port = cfg.network.camera_port
    width = spec.width if width is None else width
    height = spec.height if height is None else height
    fps = spec.fps if fps is None else fps
    quality = spec.quality if quality is None else quality
    port = cfg_port if port is None else port

    holder = FrameHolder()
    source: DryFrameSource | PicameraSource
    if dry:
        mode = "dry"
        source = DryFrameSource(holder, fps=fps)
    else:
        mode = "picamera2"
        source = PicameraSource(holder, width=width, height=height,
                                fps=fps, quality=quality)
    try:
        source.start()
    except CameraUnavailable as e:
        print(f"Cannot start camera: {e}")
        return 1

    app = build_app(holder, mode=mode, width=width, height=height, fps=fps)
    view_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    url = f"http://{view_host}:{port}"
    print(f"\n  DogV3 Pi Camera\n  Open {url}  (stream: {url}/stream.mjpg)"
          f"\n  mode={mode} {width}x{height} @ {fps} fps\n")
    try:
        uvicorn.run(app, host=host, port=port)
    finally:
        source.stop()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="DogV3 onboard camera: hardware-encoded MJPEG stream over HTTP."
    )
    ap.add_argument("--config", default=None,
                    help="robot_config.json (reads peripherals.camera + network.camera_port)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=None,
                    help="HTTP port (default: config camera_port, else 8080)")
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument("--quality", type=int, default=None, help="JPEG quality hint 0-100")
    ap.add_argument("--dry", action="store_true",
                    help="synthetic frames, no camera hardware (runs anywhere)")
    args = ap.parse_args(argv)
    return run(
        args.config,
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
        dry=args.dry,
    )


if __name__ == "__main__":
    raise SystemExit(main())
