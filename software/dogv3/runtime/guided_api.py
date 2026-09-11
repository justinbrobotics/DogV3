"""HTTP and static assets for the guided tuner, using the operator's auth gate."""
from __future__ import annotations

from pathlib import Path
import math
from typing import Literal

from fastapi import HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field, model_validator

from ..config.loader import save_config
from .guided_tuner import GuidedTuner, TrialMonitor, context_key, screen_trajectory
from .gait_library import REFERENCE_NAME, catalog


class BeginBody(BaseModel):
    surface: str = Field(min_length=3, max_length=160)
    profile: str | None = None


class SessionBody(BaseModel):
    session_id: str


class NextBody(SessionBody):
    skip: bool = False


class CandidateBody(SessionBody):
    candidate_id: str


class ResultBody(CandidateBody):
    recording_id: str
    outcome: Literal["better", "same", "worse", "unclear"]
    issues: list[Literal["dragging", "slipping", "rocking", "fell", "hot_or_power_issue"]] = Field(default_factory=list, max_length=5)
    note: str = Field(default="", max_length=1000)
    distance_m: float | None = Field(default=None, gt=0, le=20, allow_inf_nan=False)
    travel_time_s: float | None = Field(default=None, gt=0, le=600, allow_inf_nan=False)

    @model_validator(mode="after")
    def paired_measurement(self):
        if (self.distance_m is None) != (self.travel_time_s is None):
            raise ValueError("Enter both measured distance and travel time, or leave both empty.")
        return self


class ExportBody(SessionBody):
    name: str = Field(min_length=1, max_length=80)
    gait_type: Literal["trot", "crawl"] = "trot"


class PackBody(BaseModel):
    surface: str = Field(default="Same surface throughout this session; unspecified", min_length=3, max_length=160)


def install_guided_routes(app, loop, config_path, *, dry, live_seed, state, file_changed):
    path = Path(config_path)
    mode = "simulation" if loop.twin is not None else (
        "dry" if dry or loop.controller.driver is None else "physical")
    tuner = GuidedTuner(path.with_name(path.stem + ".tuning") / (mode + ".json"), mode)
    monitor = TrialMonitor()
    loop.guided_monitor = monitor
    # Exposed for integration tests and explicit host inspection, not hardware access.
    app.state.guided_tuner, app.state.guided_monitor = tuner, monitor

    def ensure_session(body):
        if tuner.session.id != body.session_id:
            raise ValueError("This session changed in another tab. Refresh before continuing.")
        if file_changed():
            raise ValueError("Robot config changed on disk. Reload the operator before continuing.")
        tuner.check_context(loop.controller.config, loop.hz)

    def payload():
        with tuner.lock:
            session = tuner.journal.sessions[-1] if tuner.journal.sessions else None
            mismatch = bool(session and session.context != context_key(loop.controller.config, loop.hz))
            return {"mode": mode, "session": session.model_dump() if session else None,
                "session_count": len(tuner.journal.sessions), "error": tuner.error,
                "context_changed": mismatch or file_changed(), "recording": monitor.snapshot(),
                "load": loop.snapshot().get("guided_load", {"status": "idle"}),
                "catalog": catalog(), "reference_name": REFERENCE_NAME,
                "trajectory_scope": "Straight motion only; use full forward input for the test pack, with zero height trim.",
                "feedback": "Records operator observations, drive intent and available IMU attitude. No extra servo polling.",
                "round_limit": 12}

    def call(action, *, write=True, after_commit=None, on_error=None):
        # Keep the journal commit and monitor cleanup indivisible across tabs.
        with tuner.lock:
            try:
                try:
                    with tuner.transaction() if write else tuner.lock:
                        action()
                except Exception:
                    if on_error:
                        on_error()
                    raise
                if after_commit:
                    after_commit()
                return payload()
            except ValueError as exc:
                raise HTTPException(409, str(exc)) from exc
            except OSError as exc:
                raise HTTPException(503, "Could not save tuning history; previous saved data was preserved.") from exc

    @app.get("/static/guided-tuner.js")
    def guided_js():
        return FileResponse(Path(__file__).parent / "static" / "guided-tuner.js",
                            media_type="application/javascript", headers={"Cache-Control": "no-cache"})

    @app.get("/static/guided-tuner.css")
    def guided_css():
        return FileResponse(Path(__file__).parent / "static" / "guided-tuner.css",
                            media_type="text/css", headers={"Cache-Control": "no-cache"})

    @app.get("/api/tuner")
    def guided_status():
        return payload()

    @app.get("/api/tuner/history")
    def guided_history():
        with tuner.lock:
            return tuner.journal.model_dump()

    @app.post("/api/tuner/begin")
    def guided_begin(body: BeginBody):
        def action():
            if monitor.snapshot() or (tuner.journal.sessions and tuner.session.pending_recording):
                raise ValueError("Save or discard the recording before starting a new session.")
            if loop.controller.state.armed or loop._btn["arm"]:
                raise ValueError("Capture the reference while disarmed.")
            if loop.auto_mode or loop._pending_gait or loop._pending_guided:
                raise ValueError("Turn AUTO off and wait for pending gait edits before capturing a reference.")
            if file_changed():
                raise ValueError("Config changed on disk. Reload the operator first.")
            if not body.surface.strip():
                raise ValueError("Describe the test surface and support setup.")
            if body.profile:
                seed = loop.controller.config.gait_profiles.get(body.profile)
                if seed is None:
                    raise ValueError("That saved profile no longer exists.")
                name = body.profile
            else:
                seed = live_seed()
                name = state["active_profile"] or "Captured robot gait"
            with loop.guided_check():
                tuner.begin(loop.controller.config, seed, loop.hz, body.surface.strip(), name)
        return call(action)

    @app.post("/api/tuner/begin-pack")
    def guided_begin_pack(body: PackBody):
        def action():
            if not body.surface.strip():
                raise ValueError("Describe the test surface and support setup.")
            if monitor.snapshot() or (tuner.journal.sessions and tuner.session.pending_recording):
                raise ValueError("Finish or discard the current recording first.")
            if file_changed():
                raise ValueError("Config changed on disk. Reload the operator first.")
            if loop.auto_mode or loop._pending_gait or loop._pending_guided:
                raise ValueError("Turn AUTO off and wait for pending edits.")
            cfg = loop.controller.config
            seed = cfg.gait_profiles.get(REFERENCE_NAME)
            if seed is None:
                raise ValueError(f"The test set needs your saved {REFERENCE_NAME} reference.")
            with loop.guided_check():
                tuner.begin_pack(cfg, seed, loop.hz, body.surface.strip(), REFERENCE_NAME)
        return call(action)

    @app.post("/api/tuner/next")
    def guided_next(body: NextBody):
        def action():
            ensure_session(body)
            if monitor.snapshot() or tuner.session.pending_recording:
                raise ValueError("Save or discard the recording before requesting another experiment.")
            if body.skip:
                for c in tuner.session.candidates:
                    if c.status == "proposed":
                        c.status = "skipped"
            with loop.guided_check():
                tuner.propose(loop.controller.config, loop.hz)
        return call(action)

    @app.post("/api/tuner/prepare")
    def guided_prepare(body: CandidateBody):
        def action():
            ensure_session(body)
            if monitor.snapshot() or tuner.session.pending_recording:
                raise ValueError("Save or discard this recording before changing gait.")
            c = tuner.session.candidate(body.candidate_id)
            if any(isinstance(v, (int, float)) and not math.isfinite(v) for v in c.seed.model_dump().values()):
                raise ValueError("Non-finite settings cannot be loaded.")
            with loop.guided_check():
                if c.id != "reference":
                    screen = screen_trajectory(loop.controller.config, c.seed, loop.hz,
                        sample_hz=600 if tuner.session.method == "test_pack" else 0)
                    if not screen["passed"]:
                        raise ValueError("Experiment blocked: " + " ".join(screen["blockers"]))
                # Restoring the exact original reference is allowed even when
                # it exceeds the model. Its concerns remain visible, never a pass.
                loop.stage_guided_gait(body.session_id + "/" + c.id, c.seed)
            state["active_profile"] = None
            state["gait_dirty"] = True
        return call(action, write=False)

    @app.post("/api/tuner/record")
    def guided_record(body: CandidateBody):
        started = False
        def action():
            nonlocal started
            ensure_session(body)
            if tuner.session.pending_recording:
                raise ValueError("Save or discard the earlier recording first.")
            c = tuner.session.candidate(body.candidate_id)
            snap = loop.snapshot()
            load = snap.get("guided_load", {})
            if (load.get("status") != "loaded" or load.get("candidate_id") != body.session_id + "/" + c.id
                    or snap.get("gait_seed") != c.seed.model_dump()):
                raise ValueError("Load this exact gait and wait for it to be applied before recording.")
            if loop.auto_mode or loop._pending_gait or loop._pending_guided:
                raise ValueError("Turn AUTO off and finish pending edits before recording.")
            monitor.start(c.id, c.seed, loop.tick_stats(),
                          forward_intent=1. if tuner.session.method == "test_pack" else .5)
            started = True
            tuner.session.pending_recording = monitor.snapshot()
        return call(action, on_error=lambda: monitor.clear() if started else None)

    @app.post("/api/tuner/result")
    def guided_result(body: ResultBody):
        def action():
            ensure_session(body)
            recording = monitor.snapshot()
            if not recording or recording["id"] != body.recording_id or recording["candidate_id"] != body.candidate_id:
                raise ValueError("This recording changed or has already been saved.")
            observed = monitor.finish(link_lost=loop.link_lost, loop_stats=loop.tick_stats())
            tuner.record(body.candidate_id, body.outcome, body.issues, body.note, observed,
                         body.distance_m, body.travel_time_s)
            tuner.session.pending_recording = None
        return call(action, after_commit=monitor.clear)

    @app.post("/api/tuner/discard")
    def guided_discard(body: SessionBody):
        def action():
            # A config change must not trap an unsaved recording indefinitely.
            if tuner.session.id != body.session_id:
                raise ValueError("The session changed. Refresh first.")
            tuner.session.pending_recording = None
        return call(action, after_commit=monitor.clear)

    @app.post("/api/tuner/export")
    def guided_export(body: ExportBody):
        def action():
            ensure_session(body)
            name = body.name.strip()
            cfg = loop.controller.config
            if not name or name in cfg.gait_profiles:
                raise ValueError("Choose a new profile name; existing profiles are preserved.")
            best_id = tuner.session.best_by_gait.get(body.gait_type, tuner.session.best_id)
            best = tuner.session.candidate(best_id)
            if best.status != "best":
                raise ValueError("No experiment has two clean better results yet.")
            cfg = cfg.model_copy(deep=True)
            cfg.gait_profiles[name] = best.seed.model_copy(deep=True)
            save_config(cfg, path)
            loop.controller.config.gait_profiles[name] = best.seed.model_copy(deep=True)
            state["loaded_mtime"] = path.stat().st_mtime
        return call(action, write=False)
