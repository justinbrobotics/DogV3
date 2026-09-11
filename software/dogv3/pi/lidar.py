"""D4 milestone 1 — RPLIDAR C1 point-cloud GUI, served from the Pi.

A browser app: the Pi spins the lidar and pushes one snapshot per revolution
over a WebSocket; the Home-PC browser renders a live 2D polar view (robot at
center, 0 deg = forward/up, clockwise) with range rings, zoom, and a
persistence slider for a map-sketch effect.

    dogv3-pi-lidar --config robot_config.json     # on the Pi
    dogv3-pi-lidar --dry                          # dev box: synthetic room

Design:
  - One :class:`ScanReader` thread owns the serial port (pyserial is not
    re-entrant) and publishes a lock-guarded latest-snapshot; the WebSocket
    loop polls that snapshot and only ever sends the newest one, so a slow
    client drops revolutions instead of queueing them.
  - The C1 speaks the classic A-series standard protocol at 460800 baud over
    its CP2102 adapter; motor start/stop are harmless no-ops on C1. The
    ``rplidar`` import is lazy so this module loads on any machine.
  - On serial errors the reader closes the port, surfaces the error string in
    /api/status, and retries every 2 s — the lidar can be unplugged/replugged
    without restarting the service.
  - Read-only service: no mutating endpoints, so no token gate is needed.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import threading
import time
from collections import deque

from ..config.loader import ConfigError, load_config

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    import uvicorn
except Exception:  # pragma: no cover - FastAPI optional at import time
    FastAPI = None  # type: ignore

DEFAULT_DEVICE = "/dev/rplidar"
DEFAULT_BAUD = 460800
DEFAULT_PORT = 8090
WS_POLL_S = 0.05          # snapshot poll cadence for the push loop
RETRY_S = 2.0             # serial reopen cadence after an error
HZ_WINDOW_REVS = 10       # scan-rate measurement window

# Dry synthesis: a plausible room so the viewer is exercised end-to-end with
# zero hardware (10 Hz is the C1's nominal scan rate).
DRY_REV_HZ = 10.0
DRY_PTS_PER_REV = 500


class ScanReader:
    """Background reader: real lidar or dry synthesis -> latest snapshot.

    Snapshot shape: ``{seq, t_mono, points: [[angle_deg, dist_mm], ...],
    hz, pps}`` with quality<1 and zero-distance returns already dropped."""

    def __init__(self, *, dry: bool, device: str = DEFAULT_DEVICE,
                 baudrate: int = DEFAULT_BAUD):
        self.dry = dry
        self.device = device
        self.baudrate = baudrate
        self._lock = threading.Lock()
        self._snapshot: dict = {}
        self._seq = 0
        self._error: str | None = None
        self._rev_times: deque[float] = deque(maxlen=HZ_WINDOW_REVS)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._rng = random.Random(0)  # deterministic dry room

    # -- published state (any thread) -------------------------------------
    def snapshot(self) -> dict:
        # Shallow copy is safe: _publish replaces the points list wholesale,
        # never mutates it in place.
        with self._lock:
            return dict(self._snapshot)

    def status(self) -> dict:
        with self._lock:
            snap = self._snapshot
            err = self._error
        return {
            "mode": "dry" if self.dry else "real",
            "device": "synthetic" if self.dry else self.device,
            "hz": snap.get("hz", 0.0),
            "points_per_scan": len(snap["points"]) if snap else 0,
            "last_scan_age_s": (round(time.monotonic() - snap["t_mono"], 3)
                                if snap else None),
            "error": err,
        }

    def _publish(self, points: list[list[float]]) -> None:
        now = time.monotonic()
        self._rev_times.append(now)
        hz = 0.0
        if len(self._rev_times) >= 2:
            span = self._rev_times[-1] - self._rev_times[0]
            if span > 0:
                hz = (len(self._rev_times) - 1) / span
        with self._lock:
            self._seq += 1
            self._error = None  # a fresh revolution clears any stale link error
            self._snapshot = {
                "seq": self._seq,
                "t_mono": now,
                "points": points,
                "hz": round(hz, 2),
                "pps": int(round(hz * len(points))),
            }

    # -- thread lifecycle --------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_dry if self.dry else self._run_real,
            name="dogv3-lidar-scan", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # Real mode blocks at most the serial timeout before noticing.
            self._thread.join(timeout=5.0)
            self._thread = None

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -- real hardware -----------------------------------------------------
    def _run_real(self) -> None:
        try:
            from rplidar import RPLidar  # lazy: hardware dep, Pi-only
        except ImportError:
            with self._lock:
                self._error = ("python package 'rplidar' missing — run "
                               "'pip install rplidar-roboticia' on the Pi")
            return
        while not self._stop.is_set():
            lidar = None
            try:
                lidar = RPLidar(self.device, baudrate=self.baudrate, timeout=3)
                lidar.start_motor()  # no-op on C1 (motor is self-managed)
                # iter_scans yields one revolution of (quality, angle, dist).
                for scan in lidar.iter_scans():
                    if self._stop.is_set():
                        break
                    self._publish([[a % 360.0, float(d)]
                                   for q, a, d in scan if q >= 1 and d > 0])
            except Exception as e:
                with self._lock:
                    self._error = str(e) or type(e).__name__
            finally:
                if lidar is not None:
                    for op in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                        try:
                            op()
                        except Exception:
                            pass
            self._stop.wait(RETRY_S)

    # -- dry synthesis -------------------------------------------------------
    def _dry_scan(self, t: float) -> list[list[float]]:
        """One synthetic revolution: rectangular room ~5 x 7 m with wall
        noise, plus a 200 mm obstacle blob orbiting the robot at 1.5 m."""
        hx, hy = 2500.0, 3500.0  # half extents, mm (walls 2.5 / 3.5 m away)
        oa = (t * 0.5) % (2.0 * math.pi)
        ox, oy = 1500.0 * math.sin(oa), 1500.0 * math.cos(oa)
        r = 200.0
        pts: list[list[float]] = []
        step = 360.0 / DRY_PTS_PER_REV
        for i in range(DRY_PTS_PER_REV):
            if self._rng.random() < 0.01:
                continue  # specular dropout
            a = (i + self._rng.random() * 0.4) * step
            rad = math.radians(a)
            dx, dy = math.sin(rad), math.cos(rad)  # 0 deg = +forward, clockwise
            d = min(hx / abs(dx) if abs(dx) > 1e-9 else 1e12,
                    hy / abs(dy) if abs(dy) > 1e-9 else 1e12)
            # Ray-circle intersection for the moving blob (nearer hit wins).
            b = ox * dx + oy * dy
            disc = b * b - (ox * ox + oy * oy - r * r)
            if disc > 0 and b > 0:
                th = b - math.sqrt(disc)
                if 0 < th < d:
                    d = th
            d += self._rng.gauss(0.0, 12.0)
            if d <= 0:
                continue
            pts.append([round(a % 360.0, 2), round(d, 1)])
        return pts

    def _run_dry(self) -> None:
        period = 1.0 / DRY_REV_HZ
        t0 = time.monotonic()
        while not self._stop.is_set():
            start = time.monotonic()
            self._publish(self._dry_scan(start - t0))
            rem = period - (time.monotonic() - start)
            if rem > 0:
                self._stop.wait(rem)


def build_app(reader: ScanReader):
    app = FastAPI(title="DogV3 Lidar (D4)")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return INDEX_HTML

    @app.get("/api/status")
    def status() -> dict:
        return reader.status()

    @app.websocket("/ws/scan")
    async def ws_scan(ws: WebSocket) -> None:
        """Push each NEW snapshot as it lands. Polling the latest snapshot
        (rather than queueing every revolution) makes slow clients drop
        frames instead of falling behind; the seq gate stops duplicates."""
        await ws.accept()
        last_seq = -1
        try:
            while True:
                snap = reader.snapshot()
                if snap and snap["seq"] != last_seq:
                    last_seq = snap["seq"]
                    await ws.send_text(json.dumps(snap))
                await asyncio.sleep(WS_POLL_S)
        except (WebSocketDisconnect, RuntimeError):
            return  # client closed (or send on a closing socket): session over

    return app


INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8>
<title>DogV3 Lidar</title>
<style>
 :root{color-scheme:dark}
 html,body{margin:0;height:100%;background:#0e1116;color:#e6e6e6;font-family:system-ui,sans-serif;overflow:hidden}
 #cv{position:absolute;inset:0;display:block}
 #hud{position:absolute;top:10px;left:12px;pointer-events:none}
 #hud b{font-size:15px}
 #stats{color:#9aa4b2;margin-top:2px;font-size:13px;font-variant-numeric:tabular-nums}
 #ctl{position:absolute;bottom:12px;left:12px;display:flex;gap:8px;align-items:center;
      background:rgba(17,21,27,.9);border:1px solid #2a2f37;border-radius:8px;padding:8px 10px;font-size:12px}
 button{border:0;border-radius:6px;padding:6px 12px;color:#fff;font-weight:600;cursor:pointer;font-size:13px;background:#374151}
 #pause.on{background:#b45309}
 input[type=range]{width:120px}
 .lab{color:#9aa4b2}
</style></head><body>
<canvas id=cv></canvas>
<div id=hud><b>DogV3 · Lidar (D4)</b><div id=stats>connecting…</div></div>
<div id=ctl>
 <button id=zin>+</button><button id=zout>−</button><span class=lab id=scale></span>
 <span class=lab>persist</span><input id=pslider type=range min=0 max=5 step=0.25 value=0><span class=lab id=pv>0 s</span>
 <button id=pause>Pause</button>
</div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let W=0,H=0;
function resize(){const dpr=window.devicePixelRatio||1;W=innerWidth;H=innerHeight;
 cv.width=W*dpr;cv.height=H*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);}
addEventListener('resize',resize);resize();

let mmPerPx=12, persistS=0, paused=false, scans=[], lastMsg=0, hz=0, nPts=0, wsOpen=false;

// ---- auto-reconnecting WebSocket; paused = drop incoming (never queue) ----
function connect(){
 const ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws/scan');
 ws.onopen=()=>{wsOpen=true;};
 ws.onmessage=e=>{lastMsg=performance.now(); if(paused)return;
  const s=JSON.parse(e.data); hz=s.hz; nPts=s.points.length;
  scans.push({t:performance.now(),pts:s.points});};
 ws.onclose=()=>{wsOpen=false;setTimeout(connect,1000);};
}
connect();

// ---- controls ----
function zoom(f){mmPerPx=Math.min(200,Math.max(1,mmPerPx*f));}
zin.onclick=()=>zoom(1/1.25); zout.onclick=()=>zoom(1.25);
cv.addEventListener('wheel',e=>{e.preventDefault();zoom(e.deltaY>0?1.15:1/1.15);},{passive:false});
pslider.oninput=()=>{persistS=parseFloat(pslider.value);pv.textContent=persistS+' s';};
pause.onclick=()=>{paused=!paused;pause.textContent=paused?'Resume':'Pause';pause.classList.toggle('on',paused);};

// ---- render: polar -> cartesian, 0 deg = up/forward, clockwise ----
function draw(){
 requestAnimationFrame(draw);
 ctx.clearRect(0,0,W,H);
 const cx=W/2, cy=H/2, now=performance.now();
 // range rings
 ctx.strokeStyle='#232a34'; ctx.fillStyle='#5b6672'; ctx.font='11px system-ui'; ctx.lineWidth=1;
 for(const m of [1000,2000,5000,10000]){
  const r=m/mmPerPx; if(r<14||r>Math.hypot(W,H)) continue;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2); ctx.stroke();
  ctx.fillText((m/1000)+' m', cx+4, cy-r+12);
 }
 ctx.strokeStyle='#1a212b';
 ctx.beginPath(); ctx.moveTo(0,cy);ctx.lineTo(W,cy);ctx.moveTo(cx,0);ctx.lineTo(cx,H); ctx.stroke();
 // scans: prune to the persistence window (always keep the newest), draw
 // oldest first so the live revolution lands on top.
 const keepMs=persistS*1000;
 scans=scans.filter((s,i)=>i===scans.length-1||now-s.t<=keepMs);
 for(let i=0;i<scans.length;i++){
  const s=scans[i], live=i===scans.length-1, age=now-s.t;
  const a=live?1:Math.max(0,1-age/Math.max(keepMs,1));
  if(a<=0) continue;
  ctx.fillStyle=live?'#3fd07f':'rgba(63,170,120,'+(a*0.5).toFixed(3)+')';
  for(const p of s.pts){
   const rad=p[0]*Math.PI/180, d=p[1]/mmPerPx;
   ctx.fillRect(cx+d*Math.sin(rad)-1, cy-d*Math.cos(rad)-1, 2, 2);
  }
 }
 // robot marker + heading arrow (forward = up)
 ctx.fillStyle='#3b82f6'; ctx.strokeStyle='#3b82f6'; ctx.lineWidth=2;
 ctx.beginPath(); ctx.moveTo(cx,cy-9); ctx.lineTo(cx-6,cy+7); ctx.lineTo(cx+6,cy+7); ctx.closePath(); ctx.fill();
 ctx.beginPath(); ctx.moveTo(cx,cy-9); ctx.lineTo(cx,cy-22); ctx.stroke();
 ctx.beginPath(); ctx.moveTo(cx,cy-26); ctx.lineTo(cx-4,cy-18); ctx.lineTo(cx+4,cy-18); ctx.closePath(); ctx.fill();
 // stats
 scale.textContent=mmPerPx.toFixed(1)+' mm/px';
 const stale=lastMsg?Math.round(now-lastMsg):null;
 stats.textContent=(wsOpen?'':'reconnecting… ')+(paused?'PAUSED · ':'')
  +hz.toFixed(1)+' Hz · '+nPts+' pts/scan · ws '+(stale===null?'—':stale+' ms');
}
requestAnimationFrame(draw);
</script></body></html>"""


def run(*, device: str, baudrate: int, host: str, port: int, dry: bool) -> int:
    if FastAPI is None:
        print("FastAPI/uvicorn not installed; cannot run the lidar GUI.")
        return 1
    reader = ScanReader(dry=dry, device=device, baudrate=baudrate)
    app = build_app(reader)
    reader.start()
    view_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    print(f"\n  DogV3 Lidar (D4)\n  Open http://{view_host}:{port}\n"
          f"  source={'DRY (synthetic room)' if dry else device}\n")
    try:
        uvicorn.run(app, host=host, port=port)
    finally:
        reader.stop()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="DogV3 D4 lidar GUI: live RPLIDAR C1 point cloud in the browser."
    )
    ap.add_argument("--config", default=None,
                    help="robot_config.json (optional; supplies device/baud/port)")
    ap.add_argument("--device", default=None,
                    help=f"lidar serial device (default from config, else {DEFAULT_DEVICE})")
    ap.add_argument("--baud", type=int, default=None,
                    help=f"serial baudrate (default from config, else {DEFAULT_BAUD})")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=None,
                    help=f"HTTP port (default from config, else {DEFAULT_PORT})")
    ap.add_argument("--dry", action="store_true",
                    help="synthesize scans; no lidar or serial needed")
    args = ap.parse_args(argv)

    cfg = None
    if args.config:
        try:
            cfg = load_config(args.config, require_commissioned=False)
        except ConfigError as e:
            print(f"Cannot load config: {e}")
            return 1
    spec = cfg.peripherals.lidar if cfg else None
    if spec is not None and not spec.enabled and not args.dry:
        print("Lidar disabled in config (peripherals.lidar.enabled=false); "
              "enable it or run with --dry.")
        return 1
    device = args.device or (spec.device if spec else DEFAULT_DEVICE)
    baudrate = args.baud or (spec.baudrate if spec else DEFAULT_BAUD)
    port = args.port or (cfg.network.lidar_port if cfg else DEFAULT_PORT)
    return run(device=device, baudrate=baudrate, host=args.host, port=port, dry=args.dry)


if __name__ == "__main__":
    raise SystemExit(main())
