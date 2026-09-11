"""Pi audio service — synthesized sound effects + offline TTS (port 8091).

Runs on the Pi against the Wonrabai USB sound card (driver-free UAC device,
plain ALSA). Clips are synthesized from math at startup into a temp cache —
no binary assets in the repo — and played with ``aplay``; speech goes through
``espeak-ng`` (or optionally ``piper``, see below).

    dogv3-pi-audio --config robot_config.json
    dogv3-pi-audio --alsa-device plughw:1,0 --volume 60
    dogv3-pi-audio --dry                       # API/GUI only, no subprocesses

Design:
  - One playback at a time: a non-blocking lock serializes aplay/TTS
    subprocesses; a busy service answers ``{ok: false, busy: true}`` instead
    of queueing (a barking robot must not build a backlog of barks).
  - Say text is untrusted operator input: length-capped and passed as an argv
    list — never shell=True.
  - TTS engine "piper" is optional. It needs the piper binary AND a voice
    model configured system-wide; if selected but not installed the service
    falls back to espeak-ng and says so in the response.
  - Dry mode (--dry, or auto on Windows without aplay) synthesizes the clips
    but skips every subprocess, so this box can exercise the whole API.
"""
from __future__ import annotations

import argparse
import math
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import wave
from pathlib import Path

from ..config.loader import ConfigError, load_config
from ..config.schema import AudioSpec

try:
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import HTMLResponse
    from pydantic import BaseModel
    import uvicorn
except Exception:  # pragma: no cover - FastAPI optional at import time
    FastAPI = None  # type: ignore
    BaseModel = object  # type: ignore


# A deliberately short set of voices that is present in the Pi's eSpeak NG
# 1.51 installation.  Keep the API constrained to known voices: the browser
# gets friendly labels while the subprocess still receives one validated argv
# element (never shell text).
TTS_VOICES = (
    {"id": "en-us", "label": "American"},
    {"id": "en-us+f3", "label": "American (female)"},
    {"id": "en-gb", "label": "British"},
    {"id": "en-gb-scotland", "label": "Scottish"},
    {"id": "en-us-nyc", "label": "New York"},
    {"id": "en-us+robosoft3", "label": "Robot"},
)
TTS_VOICE_IDS = frozenset(v["id"] for v in TTS_VOICES)
DEFAULT_TTS_VOICE = "en-us"
DEFAULT_TTS_RATE_WPM = 90
MIN_TTS_RATE_WPM = 80
MAX_TTS_RATE_WPM = 220


# Request bodies must live at module scope: with ``from __future__ import
# annotations`` FastAPI resolves these annotations against module globals, so a
# class nested inside build_app would be mistaken for a query param.
class PlayBody(BaseModel):
    name: str


class SayBody(BaseModel):
    text: str
    voice: str = DEFAULT_TTS_VOICE
    volume: int | None = None
    rate_wpm: int = DEFAULT_TTS_RATE_WPM


SAMPLE_RATE = 22050
SAY_MAX_CHARS = 200
# Regenerable cache — clips are pure functions of the code, so /tmp is their
# home; a wiped tmpfs just costs one re-synthesis at startup.
CLIP_CACHE = Path(tempfile.gettempdir()) / "dogv3_clips"


# ---------------------------------------------------------------- clips ----
def _sweep(dur_s: float, f0: float, f1: float, *, timbre: str = "sine",
           amp: float = 0.85) -> list[float]:
    """Phase-continuous exponential frequency sweep (f0 == f1 = plain tone)."""
    n = int(dur_s * SAMPLE_RATE)
    out = []
    phase = 0.0
    for i in range(n):
        f = f0 * (f1 / f0) ** (i / n)
        phase += 2.0 * math.pi * f / SAMPLE_RATE
        s = math.sin(phase)
        if timbre == "growl":     # soft-clipped sine: square-ish, animal-like
            s = math.tanh(2.5 * s)
        elif timbre == "square":  # harsh alarm timbre
            s = 1.0 if s >= 0 else -1.0
        out.append(amp * s)
    return out


def _silence(dur_s: float) -> list[float]:
    return [0.0] * int(dur_s * SAMPLE_RATE)


def _fade(samples: list[float], dur_s: float = 0.008) -> list[float]:
    """Linear fade in/out so segment edges never click."""
    n = min(int(dur_s * SAMPLE_RATE), len(samples) // 2)
    for i in range(n):
        g = i / n
        samples[i] *= g
        samples[-1 - i] *= g
    return samples


def _clip_bark() -> list[float]:
    # Two short falling chirps with a growly (soft-clipped) timbre.
    chirp = _fade(_sweep(0.14, 400, 150, timbre="growl"))
    return chirp + _silence(0.07) + chirp


def _clip_chime() -> list[float]:
    return _fade(_sweep(0.18, 660, 660)) + _fade(_sweep(0.22, 880, 880))


def _clip_arm() -> list[float]:
    return _fade(_sweep(0.45, 220, 880))


def _clip_disarm() -> list[float]:
    return _fade(_sweep(0.45, 880, 220))


def _clip_estop() -> list[float]:
    # Harsh alternating two-tone buzz, unmistakable over gait noise.
    out: list[float] = []
    for i in range(10):
        f = 440 if i % 2 == 0 else 310
        out += _fade(_sweep(0.08, f, f, timbre="square"), 0.002)
    return out


def _clip_boot() -> list[float]:
    out: list[float] = []
    for f in (523.25, 659.25, 783.99):  # C5 E5 G5
        out += _fade(_sweep(0.11, f, f)) + _silence(0.02)
    return out


def _clip_beep() -> list[float]:
    return _fade(_sweep(0.12, 880, 880))


# Public clip set (GET /api/clips). "beep" is synthesized too but served only
# through POST /api/beep — it is a utility blip, not part of the vocabulary.
CLIP_NAMES = ("bark", "chime", "arm", "disarm", "estop", "boot")
_CLIP_BUILDERS = {
    "bark": _clip_bark,
    "chime": _clip_chime,
    "arm": _clip_arm,
    "disarm": _clip_disarm,
    "estop": _clip_estop,
    "boot": _clip_boot,
    "beep": _clip_beep,
}


def _write_wav(path: Path, samples: list[float]) -> None:
    frames = bytearray()
    for s in samples:
        frames += struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32767))
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)  # 16-bit
        w.setframerate(SAMPLE_RATE)
        w.writeframes(bytes(frames))


def ensure_clips(cache_dir: Path | None = None) -> dict[str, Path]:
    """Synthesize any missing clip WAV into the cache; return name -> path."""
    cache = Path(cache_dir) if cache_dir is not None else CLIP_CACHE
    cache.mkdir(parents=True, exist_ok=True)
    out: dict[str, Path] = {}
    for name, build in _CLIP_BUILDERS.items():
        p = cache / f"{name}.wav"
        if not p.exists():
            _write_wav(p, _fade(build()))
        out[name] = p
    return out


# --------------------------------------------------------------- player ----
class AudioPlayer:
    """Serialized playback owner: at most one aplay/TTS subprocess at a time.

    The lock is taken non-blocking on every request so the API always answers
    immediately; the owning background thread releases it when its subprocess
    chain finishes. Dry mode records intent and releases straight away."""

    def __init__(self, *, alsa_device: str, dry: bool) -> None:
        self.alsa_device = alsa_device
        self.dry = dry
        self.last_played: str | None = None
        self._busy = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._closed = False

    def play_wav(self, path: Path, label: str) -> dict:
        if not self._busy.acquire(blocking=False):
            return {"ok": False, "busy": True}
        self.last_played = label
        if self.dry:
            self._busy.release()
            return {"ok": True, "dry": True}
        steps = [(["aplay", "-q", "-D", self.alsa_device, str(path)], None)]
        self._start(steps)
        return {"ok": True}

    def say(self, text: str, *, engine: str, volume: int, work_dir: Path,
            voice: str = DEFAULT_TTS_VOICE,
            rate_wpm: int = DEFAULT_TTS_RATE_WPM) -> dict:
        note = None
        if engine == "piper" and shutil.which("piper") is None:
            note = "piper not installed; fell back to espeak-ng"
            engine = "espeak-ng"
        if not self._busy.acquire(blocking=False):
            return {"ok": False, "busy": True}
        self.last_played = f"say:{text[:40]}"
        if self.dry:
            self._busy.release()
            return {"ok": True, "dry": True, **({"note": note} if note else {})}
        if engine == "espeak-ng":
            if shutil.which("espeak-ng") is None:
                self._busy.release()
                return {"ok": False,
                        "error": "espeak-ng not found — on the Pi: sudo apt install espeak-ng"}
            # espeak-ng -a is amplitude 0..200; AudioSpec.volume is 0..100.
            amp = str(max(0, min(200, volume * 2)))
            # Render to a WAV and play it with aplay rather than letting
            # espeak-ng open the sound card itself. espeak-ng has no device
            # flag: left to itself it always plays to the ALSA DEFAULT, so on
            # any robot whose alsa_device is not "default" the chimes come out
            # of the speaker and the speech silently goes somewhere else. Going
            # through aplay -D is the only way speech and clips share a device,
            # and it matches what the piper branch below already does.
            out_wav = work_dir / "say_espeak.wav"
            steps = [(["espeak-ng", "-v", voice, "-a", amp, "-s", str(rate_wpm),
                       "-w", str(out_wav), "--", text], None),
                     (["aplay", "-q", "-D", self.alsa_device, str(out_wav)], None)]
        else:
            # piper reads text on stdin and writes a WAV; it relies on a voice
            # model configured system-wide. A runtime piper failure just stops
            # the chain (the not-installed case already fell back above).
            out_wav = work_dir / "say_piper.wav"
            steps = [(["piper", "--output_file", str(out_wav)], text),
                     (["aplay", "-q", "-D", self.alsa_device, str(out_wav)], None)]
        self._start(steps)
        return {"ok": True, **({"note": note} if note else {})}

    def _start(self, steps: list[tuple[list[str], str | None]]) -> None:
        threading.Thread(target=self._run_steps, args=(steps,),
                         name="dogv3-audio", daemon=True).start()

    def _run_steps(self, steps: list[tuple[list[str], str | None]]) -> None:
        try:
            for argv, stdin_text in steps:
                if self._closed:
                    break
                # argv list only — say text is untrusted; never shell=True.
                proc = subprocess.Popen(
                    argv,
                    stdin=subprocess.PIPE if stdin_text is not None else subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self._proc = proc
                try:
                    proc.communicate(stdin_text.encode() if stdin_text else None)
                finally:
                    self._proc = None
                if proc.returncode != 0:
                    break
        except OSError:
            pass  # binary vanished mid-call; lock still released below
        finally:
            self._busy.release()

    def stop(self) -> None:
        """Shutdown: kill any in-flight playback (a long TTS must not outlive
        the service)."""
        self._closed = True
        proc = self._proc
        if proc is not None:
            try:
                proc.terminate()
            except OSError:
                pass


# ----------------------------------------------------------------- app -----
def build_app(spec: AudioSpec, *, dry: bool, cache_dir: Path | None = None):
    app = FastAPI(title="DogV3 Pi Audio")
    clips = ensure_clips(cache_dir)
    player = AudioPlayer(alsa_device=spec.alsa_device, dry=dry)
    app.state.player = player  # main() stops it on shutdown

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return INDEX_HTML

    @app.get("/api/clips")
    def clips_list() -> dict:
        return {"clips": list(CLIP_NAMES)}

    @app.get("/api/voices")
    def voices_list() -> dict:
        return {
            "voices": list(TTS_VOICES),
            "default": DEFAULT_TTS_VOICE,
            "default_volume": spec.volume,
            "default_rate_wpm": DEFAULT_TTS_RATE_WPM,
            "min_rate_wpm": MIN_TTS_RATE_WPM,
            "max_rate_wpm": MAX_TTS_RATE_WPM,
        }

    @app.get("/api/status")
    def status() -> dict:
        return {
            "mode": "dry" if dry else "live",
            "alsa_device": spec.alsa_device,
            "tts_engine": spec.tts_engine,
            "volume": spec.volume,
            "last_played": player.last_played,
        }

    @app.post("/api/play")
    def play(body: PlayBody) -> dict:
        path = clips.get(body.name)
        if path is None:
            raise HTTPException(404, f"unknown clip {body.name!r}")
        return player.play_wav(path, body.name)

    @app.post("/api/beep")
    def beep() -> dict:
        return player.play_wav(clips["beep"], "beep")

    @app.post("/api/say")
    def say(body: SayBody) -> dict:
        text = body.text.strip()[:SAY_MAX_CHARS]
        if not text:
            raise HTTPException(400, "text required")
        if body.voice not in TTS_VOICE_IDS:
            raise HTTPException(400, f"unknown voice {body.voice!r}")
        volume = spec.volume if body.volume is None else body.volume
        if not 0 <= volume <= 100:
            raise HTTPException(400, "volume must be between 0 and 100")
        if not MIN_TTS_RATE_WPM <= body.rate_wpm <= MAX_TTS_RATE_WPM:
            raise HTTPException(
                400, f"rate_wpm must be between {MIN_TTS_RATE_WPM} and {MAX_TTS_RATE_WPM}")
        result = player.say(text, engine=spec.tts_engine, volume=volume,
                            work_dir=clips["beep"].parent, voice=body.voice,
                            rate_wpm=body.rate_wpm)
        if result.get("error"):
            raise HTTPException(503, result["error"])
        return result

    return app


INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8>
<title>DogV3 Audio</title>
<style>
 :root{color-scheme:dark}
 body{font-family:system-ui,sans-serif;background:#0e1116;color:#e6e6e6;margin:0;padding:24px;max-width:560px}
 h1{font-size:18px;margin:0 0 4px}
 .hint{font-size:12px;color:#7e8794;margin:2px 0 14px}
 #clips{display:flex;flex-wrap:wrap;gap:6px;margin:10px 0}
 button{border:0;border-radius:6px;padding:10px 14px;color:#fff;font-weight:600;cursor:pointer;font-size:14px;background:#374151}
 #sayb{background:#2563eb}
 input,select{background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:8px}
 input[type=text]{flex:1}
 .row{display:flex;gap:6px;margin:10px 0}
 .control{display:grid;grid-template-columns:110px 1fr 58px;align-items:center;gap:8px;font-size:12px;color:#9aa4b2}
 #st{font-size:12px;color:#9aa4b2;font-family:ui-monospace,monospace;word-break:break-all}
</style></head><body>
<h1>DogV3 · Audio</h1>
<p class=hint id=mode></p>
<div id=clips></div>
<div class=row><select id=sayvoice aria-label="Voice"></select><input id=saytext type=text maxlength=200 placeholder="text to speak"><button id=sayb onclick=say()>Say</button></div>
<label class=control>Voice volume<input id=sayvolume type=range min=0 max=100 step=1 value=80><output id=sayvolout>80%</output></label>
<label class=control>Speech speed<input id=sayrate type=range min=80 max=220 step=5 value=90><output id=sayrateout>90 WPM</output></label>
<p id=st></p>
<script>
const J=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:b?JSON.stringify(b):undefined})
 .then(r=>r.json()).then(r=>{document.getElementById('st').textContent=JSON.stringify(r);});
async function init(){
 const cl=await fetch('/api/clips').then(r=>r.json());
 const c=document.getElementById('clips');
 for(const n of cl.clips){const b=document.createElement('button');b.textContent=n;b.onclick=()=>J('/api/play',{name:n});c.appendChild(b);}
 const bp=document.createElement('button');bp.textContent='beep';bp.onclick=()=>J('/api/beep');c.appendChild(bp);
 const vl=await fetch('/api/voices').then(r=>r.json());
 const vs=document.getElementById('sayvoice');
 for(const v of vl.voices){const o=document.createElement('option');o.value=v.id;o.textContent=v.label;o.selected=v.id===vl.default;vs.appendChild(o);}
 const vol=document.getElementById('sayvolume'),rate=document.getElementById('sayrate');
 vol.value=vl.default_volume??80; rate.min=vl.min_rate_wpm??80; rate.max=vl.max_rate_wpm??220; rate.value=vl.default_rate_wpm??90;
 vol.oninput=()=>document.getElementById('sayvolout').textContent=vol.value+'%';
 rate.oninput=()=>document.getElementById('sayrateout').textContent=rate.value+' WPM'; vol.oninput(); rate.oninput();
 const s=await fetch('/api/status').then(r=>r.json());
 document.getElementById('mode').textContent='mode='+s.mode+' · device='+s.alsa_device+' · tts='+s.tts_engine+' · vol='+s.volume;
}
function say(){const t=document.getElementById('saytext').value.trim(),voice=document.getElementById('sayvoice').value,volume=+document.getElementById('sayvolume').value,rate_wpm=+document.getElementById('sayrate').value;if(t)J('/api/say',{text:t,voice,volume,rate_wpm});}
init();
</script></body></html>"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="DogV3 Pi audio service: synthesized sound effects + offline TTS."
    )
    ap.add_argument("--config", default=None,
                    help="robot_config.json (peripherals.audio + network.audio_port defaults)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=None, help="default: network.audio_port (8091)")
    ap.add_argument("--alsa-device", default=None, help="override peripherals.audio.alsa_device")
    ap.add_argument("--tts-engine", default=None, choices=["espeak-ng", "piper"])
    ap.add_argument("--volume", type=int, default=None, help="0..100, override peripherals.audio.volume")
    ap.add_argument("--dry", action="store_true",
                    help="synthesize clips but skip aplay/TTS subprocesses")
    args = ap.parse_args(argv)

    if FastAPI is None:
        print("FastAPI/uvicorn not installed; cannot run the audio service.")
        return 1

    spec = AudioSpec()
    port = 8091
    if args.config:
        try:
            cfg = load_config(args.config, require_commissioned=False)
        except ConfigError as e:
            print(f"Refusing to start: {e}")
            return 1
        spec = cfg.peripherals.audio
        port = cfg.network.audio_port
        if not spec.enabled:
            print("note: peripherals.audio.enabled is false in config; running anyway (explicit launch)")
    if args.alsa_device is not None:
        spec.alsa_device = args.alsa_device
    if args.tts_engine is not None:
        spec.tts_engine = args.tts_engine
    if args.volume is not None:
        spec.volume = max(0, min(100, args.volume))
    if args.port is not None:
        port = args.port

    # AUTO-dry: the Windows dev box has no ALSA — never attempt subprocesses.
    dry = args.dry or (sys.platform == "win32" and shutil.which("aplay") is None)

    app = build_app(spec, dry=dry)
    view_host = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
    print(f"\n  DogV3 Pi Audio\n  Open http://{view_host}:{port}\n"
          f"  mode={'DRY (no playback)' if dry else 'live'}"
          f" device={spec.alsa_device} tts={spec.tts_engine} vol={spec.volume}\n")
    try:
        uvicorn.run(app, host=args.host, port=port)
    finally:
        app.state.player.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
