"""Setup Program HTTP/WebSocket server (Host UI for commissioning).

Wraps the :class:`Commissioner` state machine behind a small REST API plus a
1 Hz telemetry WebSocket that feeds the live joint-angle readout. Run on the
bench Host PC with the ESP32 on USB.

    dogv3-setup --port-serial COM5 --config robot_config.json

If ``--port-serial`` is omitted the server runs in *dry* mode (no hardware): the
state machine still works for config bookkeeping and UI development.
"""
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

from ..config.loader import default_config, load_config, save_config, ConfigError
from ..driver.feetech import FeetechDriver, make_transport
from .set_servo_id import set_servo_id
from .state_machine import Commissioner, Stage, DIRECTION_QUESTION
from .verify_firmware import verify_transport
from ..validate import validate_config, reconstruct_pose, pose_symmetry

try:
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    from pydantic import BaseModel
    import uvicorn
except Exception:  # pragma: no cover - FastAPI optional at import time
    FastAPI = None  # type: ignore
    BaseModel = object  # type: ignore


# Request bodies must be module-level: with ``from __future__ import annotations``
# FastAPI resolves these annotations against module globals, so a class nested
# inside build_app would be mistaken for a query param and the JSON body dropped.
class AssignBody(BaseModel):
    servo_id: int
    slot: str


class DirectionBody(BaseModel):
    servo_id: int
    moved_correctly: bool


class RangeBody(BaseModel):
    servo_id: int
    min_raw: int
    home_raw: int
    max_raw: int
    write_eeprom: bool = False


class GaitBody(BaseModel):
    levers: dict


class TorqueBody(BaseModel):
    enable: bool


class SetIdBody(BaseModel):
    new_id: int
    bus: str | None = None
    force: bool = False


class TorqueAllBody(BaseModel):
    enable: bool = False


class MirrorBody(BaseModel):
    source_leg: str


class FollowStartBody(BaseModel):
    master_leg: str = "BL"
    free_joints: list[str] = []


class FollowLimitBody(BaseModel):
    joint: str
    which: str


def build_app(commissioner: Commissioner, transport, link: str | None, config_path: Path):
    app = FastAPI(title="DogV3 Setup")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return INDEX_HTML

    @app.get("/api/status")
    def status():
        ok, warnings = commissioner.ready_to_emit()
        return {
            "stages": [s.value for s in Stage],
            "slots": commissioner.status(),
            "slot_meta": _slot_meta(commissioner),
            "buses": _bus_meta(commissioner),
            "discovered": commissioner.discovered.__dict__,
            "ready_to_emit": ok,
            "warnings": warnings,
            "dry": transport is None,
            "link": link,
            "firmware_expected": commissioner.config.firmware_expected,
            "frame": commissioner.config.frame.model_dump(),
            "joint_order": commissioner.config.joint_order,
            "direction_questions": DIRECTION_QUESTION,
        }

    @app.post("/api/verify_firmware")
    def api_verify():
        if transport is None:
            raise HTTPException(400, "no link (dry mode)")
        ok = verify_transport(transport, commissioner.config.firmware_expected, do_scan=True)
        return {"pass": ok}

    @app.post("/api/set_id")
    def api_set_id(body: SetIdBody):
        if transport is None or commissioner.driver is None:
            raise HTTPException(400, "no link (dry mode); connect the ESP32 to set IDs")
        try:
            bus, src, ok = set_servo_id(
                commissioner.driver, new_id=body.new_id, bus=body.bus, force=body.force
            )
        except (RuntimeError, ValueError) as e:
            raise HTTPException(400, str(e))
        if src == body.new_id:
            msg = f"servo already has ID {body.new_id} on bus {bus.name_letter}"
        elif ok:
            msg = f"servo {src} -> {body.new_id} on bus {bus.name_letter} (verified)"
        else:
            raise HTTPException(400, f"wrote {src} -> {body.new_id} but it did not answer on the new ID")
        return {"ok": ok, "message": msg, "bus": bus.name_letter, "source_id": src, "new_id": body.new_id}

    @app.post("/api/discover")
    def api_discover():
        try:
            res = commissioner.discover()
        except RuntimeError as e:
            raise HTTPException(400, str(e))
        return res.__dict__

    @app.post("/api/wiggle/{servo_id}")
    def api_wiggle(servo_id: int):
        try:
            return {"moved": commissioner.wiggle(servo_id)}
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.get("/api/candidates/{servo_id}")
    def api_candidates(servo_id: int):
        return {"slots": commissioner.candidate_slots(servo_id)}

    @app.post("/api/assign")
    def api_assign(body: AssignBody):
        try:
            spec = commissioner.assign(body.servo_id, body.slot)
        except ValueError as e:
            raise HTTPException(400, str(e))
        return spec.model_dump()

    @app.post("/api/torque/{servo_id}")
    def api_torque(servo_id: int, body: TorqueBody):
        try:
            return {"ok": commissioner.set_torque(servo_id, body.enable)}
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/torque_all")
    def api_torque_all(body: TorqueAllBody):
        try:
            return {"results": commissioner.set_all_torque(body.enable)}
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/mirror_ranges")
    def api_mirror(body: MirrorBody):
        try:
            return commissioner.mirror_ranges(body.source_leg)
        except (ValueError, KeyError) as e:
            raise HTTPException(400, str(e))

    @app.post("/api/home_all")
    def api_home_all():
        if transport is None or commissioner.driver is None:
            raise HTTPException(400, "no link (dry mode)")
        try:
            commissioner.home_all()
            return {"ok": True}
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/follow/start")
    def api_follow_start(body: FollowStartBody):
        try:
            return commissioner.follow_start(body.master_leg, body.free_joints)
        except (RuntimeError, ValueError) as e:
            raise HTTPException(400, str(e))

    @app.post("/api/follow/tick")
    def api_follow_tick():
        try:
            return commissioner.follow_tick()
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/follow/check")
    def api_follow_check():
        try:
            return commissioner.follow_check()
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/follow/set_limit")
    def api_follow_set_limit(body: FollowLimitBody):
        try:
            return commissioner.follow_set_limit(body.joint, body.which)
        except (RuntimeError, ValueError) as e:
            raise HTTPException(400, str(e))

    @app.post("/api/follow/stop")
    def api_follow_stop():
        try:
            return commissioner.follow_stop()
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.get("/api/raw/{servo_id}")
    def api_raw(servo_id: int):
        try:
            return {"raw": commissioner.read_raw(servo_id)}
        except RuntimeError as e:
            raise HTTPException(400, str(e))

    @app.post("/api/center/{servo_id}")
    def api_center(servo_id: int):
        try:
            return {"centered": commissioner.center(servo_id)}
        except (RuntimeError, ValueError) as e:
            raise HTTPException(400, str(e))

    @app.post("/api/direction/probe/{servo_id}")
    def api_dir_probe(servo_id: int):
        try:
            return {"probed": commissioner.probe_direction(servo_id)}
        except (RuntimeError, ValueError) as e:
            raise HTTPException(400, str(e))

    @app.post("/api/direction/set")
    def api_dir_set(body: DirectionBody):
        spec = commissioner.set_direction(body.servo_id, body.moved_correctly)
        return spec.model_dump()

    @app.post("/api/range")
    def api_range(body: RangeBody):
        try:
            spec = commissioner.set_range(
                body.servo_id, body.min_raw, body.home_raw, body.max_raw, write_eeprom=body.write_eeprom
            )
        except (KeyError, ValueError) as e:
            raise HTTPException(400, str(e))
        return spec.model_dump()

    @app.post("/api/gait")
    def api_gait(body: GaitBody):
        commissioner.update_gait_seed(**body.levers)
        return commissioner.config.gait_seed.model_dump()

    @app.post("/api/emit")
    def api_emit():
        ok, warnings = commissioner.ready_to_emit()
        save_config(commissioner.config, config_path)  # save even with warnings (warned)
        return {"saved": str(config_path), "ready": ok, "warnings": warnings}

    @app.post("/api/validate")
    def api_validate():
        report = validate_config(commissioner.config)
        pose = symmetry = None
        if commissioner.driver is not None:
            raws: dict[int, int] = {}
            for leg in ("FL", "FR", "BL", "BR"):
                for joint in ("hip", "femur", "tibia"):
                    sid = getattr(commissioner.config.legs[leg].ids, joint)
                    if sid is None:
                        continue
                    r = commissioner.driver.read_present_position_raw(commissioner._bus_for(sid), sid)
                    if r is not None:
                        raws[sid] = r
            pose = reconstruct_pose(commissioner.config, raws)
            symmetry = pose_symmetry(commissioner.config, raws)
        return {"config": report, "pose": pose, "symmetry": symmetry}

    @app.websocket("/ws/pose")
    async def ws_pose(ws: WebSocket):
        """1 Hz visualizer feed: live per-joint angle for the selected servo."""
        await ws.accept()
        try:
            while True:
                pose = commissioner.poll_pose()
                await ws.send_text(json.dumps({"pose": pose, "slots": commissioner.status()}))
                await asyncio.sleep(1.0)
        except WebSocketDisconnect:
            return

    return app


def _slot_meta(commissioner: Commissioner) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        leg_spec = commissioner.config.legs[leg]
        bus_spec = commissioner.config.buses[leg_spec.bus]
        for joint in commissioner.config.joint_order:
            slot = f"{leg}_{joint}"
            out[slot] = {
                "leg": leg,
                "joint": joint,
                "label": f"{leg} {joint.title()}",
                "bus": leg_spec.bus,
                "bus_role": bus_spec.role,
                "side": leg_spec.side,
            }
    return out


def _bus_meta(commissioner: Commissioner) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for bus, spec in commissioner.config.buses.items():
        out[bus] = {
            "role": spec.role,
            "uart": spec.esp32_uart,
            "rx": spec.rx,
            "tx": spec.tx,
            "legs": [leg for leg, leg_spec in commissioner.config.legs.items() if leg_spec.bus == bus],
        }
    return out


# Revised focused D1 setup UI. One purpose-built screen for bring-up: scan the
# two buses, map each physical servo ID onto its named slot (FL_tibia, ...) fast
# — with the bus=side constraint enforced so a rear ID can only land on a rear
# slot — verify center/direction/range, and emit. The "forward" axis (frame
# forward_sign) is shown explicitly and drives the per-joint direction check.
# Only D1 bring-up lives here; gait tuning belongs to D2.
INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8>
<title>DogV3 D1</title>
<style>
 :root{color-scheme:light;--ink:#16232b;--muted:#5d6f7a;--line:#d7e0e6;--bg:#eef2f5;--panel:#fff;--slot:#f6f9fb;
  --blue:#1f6fb2;--green:#2e7d4f;--amber:#b56a17;--red:#c23b3b;--busA:#7a4fb0;--busB:#12839c}
 *{box-sizing:border-box}
 body{font-family:Segoe UI,system-ui,sans-serif;margin:0;background:var(--bg);color:var(--ink);height:100vh;overflow:hidden}
 button{border:1px solid #b7c4cc;background:#fff;color:var(--ink);border-radius:7px;padding:8px 11px;cursor:pointer;font-weight:600;font-size:13px}
 button:hover{border-color:#8fa3ae}
 button.primary{background:var(--blue);border-color:var(--blue);color:#fff}
 button.good{background:var(--green);border-color:var(--green);color:#fff}
 button.warn{background:var(--amber);border-color:var(--amber);color:#fff}
 button:disabled{opacity:.4;cursor:not-allowed}
 input{border:1px solid #b7c4cc;border-radius:6px;padding:7px 8px;font:inherit;width:84px}
 select{border:1px solid #b7c4cc;border-radius:6px;padding:7px 6px;font:inherit}
 h1{font-size:19px;margin:0}
 h2{font-size:13px;text-transform:uppercase;letter-spacing:.04em;color:var(--muted);margin:0 0 9px}
 small{color:var(--muted)}
 .app{display:grid;grid-template-rows:auto 1fr;height:100vh}
 .topbar{display:flex;align-items:center;gap:9px;padding:11px 14px;background:#fff;border-bottom:1px solid var(--line);flex-wrap:wrap}
 .topbar .spacer{flex:1}
 .pill{display:inline-flex;align-items:center;gap:5px;border:1px solid var(--line);border-radius:999px;padding:5px 10px;font-size:12px;color:var(--muted);background:#fff}
 .pill.ok{border-color:#a6d7ba;color:var(--green);background:#eef8f1}
 .pill.bad{border-color:#eab3b3;color:var(--red);background:#fdf1f1}
 .pill.hot{border-color:#b6d2e8;color:var(--blue);background:#eef6fd}
 #toast{font-size:13px;color:var(--muted);min-width:150px;text-align:right}
 .layout{display:grid;grid-template-columns:300px minmax(470px,1fr) 350px;gap:12px;padding:12px;min-height:0}
 .panel{background:var(--panel);border:1px solid var(--line);border-radius:9px;padding:13px;min-height:0;overflow:auto}
 .dup{border:1px solid #eab3b3;background:#fdf1f1;color:var(--red);border-radius:7px;padding:7px 9px;font-size:12px;font-weight:600;margin-bottom:9px}
 .busgrp{border:1px solid var(--line);border-radius:8px;margin-bottom:11px;overflow:hidden}
 .busgrp .head{padding:8px 10px;color:#fff;display:flex;justify-content:space-between;align-items:center}
 .busgrp.A .head{background:var(--busA)}
 .busgrp.B .head{background:var(--busB)}
 .busgrp .head b{font-size:13px}
 .busgrp .head small{color:rgba(255,255,255,.85)}
 .chips{display:flex;flex-wrap:wrap;gap:7px;padding:9px}
 .chip{min-width:52px;border:1px solid #b7c4cc;border-radius:7px;background:#fff;padding:7px 8px;cursor:pointer;text-align:center;font-weight:700}
 .chip small{display:block;font-weight:600;color:var(--muted);margin-top:2px}
 .chip.unmapped{border-style:dashed;border-color:#c39b45;background:#fdf7e9}
 .chip.mapped{border-color:#9cccab;background:#eef8f2}
 .chip.mapped small{color:var(--green)}
 .chip.sel{outline:3px solid #9cc4e6;border-color:var(--blue)}
 .chip.dup{border-color:var(--red);background:#fdeeee}
 .idtool{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-top:8px}
 details{margin-top:10px}
 summary{cursor:pointer;font-size:12px;color:var(--muted)}
 .fwd{display:flex;align-items:center;gap:10px;justify-content:center;font-weight:800;color:var(--blue);margin-bottom:8px;letter-spacing:.03em}
 .fwd .arrow{font-size:22px}
 .maprow{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px}
 .leg{border:2px solid var(--line);border-radius:9px;padding:9px;background:#fff;display:grid;gap:7px}
 .leg.A{border-color:#d9c9ec}
 .leg.B{border-color:#bfe0e8}
 .leg.dim{opacity:.4}
 .legtitle{display:flex;justify-content:space-between;align-items:center;font-weight:800}
 .legtitle .tag{font-size:11px;font-weight:700;color:#fff;border-radius:5px;padding:2px 6px}
 .leg.A .tag{background:var(--busA)}
 .leg.B .tag{background:var(--busB)}
 .slot{border:1px solid #cbd6dd;border-radius:7px;background:var(--slot);padding:7px 8px;cursor:pointer;text-align:left;display:grid;grid-template-columns:1fr auto;gap:3px 6px;align-items:center}
 .slot:disabled{cursor:not-allowed;opacity:.5}
 .slot.drop{border-color:var(--blue);background:#eef6fd;box-shadow:0 0 0 2px #cfe3f4 inset}
 .slot.filled{border-color:#9cccab;background:#eef8f2}
 .slot.sel{outline:3px solid #9cc4e6}
 .slot .j{font-weight:800;text-transform:capitalize}
 .slot .id{font-weight:800;color:var(--blue);text-align:right}
 .slot .live{font-size:11px;color:var(--muted)}
 .dots{grid-column:1/3;display:flex;gap:4px;margin-top:2px}
 .dot{width:20px;height:16px;border-radius:4px;border:1px solid #cdd6dc;text-align:center;font-size:10px;line-height:15px;color:#8598a2}
 .dot.on{background:var(--green);border-color:var(--green);color:#fff}
 .legend{font-size:12px;color:var(--muted);border-top:1px dashed var(--line);padding-top:8px;line-height:1.5}
 .selhead{border:1px solid var(--line);border-radius:8px;background:#fbfcfd;padding:9px;margin-bottom:10px}
 .kv{display:grid;grid-template-columns:64px 1fr;gap:3px 8px;font-size:13px}
 .kv b{color:var(--muted);font-weight:600}
 .step{border:1px solid var(--line);border-radius:8px;padding:9px;margin-bottom:9px}
 .step.done{border-color:#9cccab;background:#f4fbf6}
 .step .st{display:flex;justify-content:space-between;align-items:center;font-weight:700;margin-bottom:7px}
 .badge{font-size:11px;font-weight:700;border-radius:999px;padding:2px 8px;background:#eef2f5;color:var(--muted)}
 .badge.ok{background:#e4f4ea;color:var(--green)}
 .row{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
 .q{border-left:4px solid var(--blue);background:#eef6fd;padding:7px 9px;border-radius:0 6px 6px 0;font-weight:600;margin-bottom:7px}
 .rng{display:grid;grid-template-columns:52px 1fr auto;gap:6px;align-items:center;margin-bottom:6px}
 .rng label{font-size:12px;color:var(--muted)}
 .emit{font-size:13px;line-height:1.5}
 .hint{font-size:12px;color:var(--muted);margin-top:6px}
 @media(max-width:1080px){body{overflow:auto}.app{height:auto}.layout{grid-template-columns:1fr}}
</style></head><body>
<div class=app>
 <div class=topbar>
  <h1>DogV3 D1</h1>
  <span id=mode class=pill>...</span>
  <span id=link class=pill>link</span>
  <span id=ready class=pill>config</span>
  <span id=prog class=pill>0/12</span>
  <div class=spacer></div>
  <button class=warn onclick=torqueAllOff() title="Release torque on all 12 servos at once">&#9209; Torque OFF all</button>
  <button onclick=verify()>Verify firmware</button>
  <button class=primary onclick=scan()>Scan buses</button>
  <button class=good onclick=emit()>Save config</button>
  <div id=toast></div>
 </div>
 <div class=layout>
  <section class=panel>
   <h2>Detected IDs</h2>
   <div id=ids></div>
   <details>
    <summary>Set a servo ID (connect one servo only)</summary>
    <div class=idtool>New ID <input id=newid type=number value=1 style=width:60px>
     bus <select id=idbus><option value="">auto</option><option>A</option><option>B</option></select>
     <button onclick=setId()>Write ID</button></div>
    <div id=idout class=hint></div>
   </details>
  </section>
  <section class=panel>
   <div class=fwd><span class=arrow>&#9650;</span><span id=fwdlabel>FORWARD</span></div>
   <div id=map></div>
   <div class=legend id=legend></div>
  </section>
  <section class=panel>
   <h2>Selected servo</h2>
   <div id=work></div>
   <h2 style=margin-top:14px>Emit gate</h2>
   <div id=emit class=emit></div>
   <h2 style=margin-top:14px>Validate</h2>
   <div class=step>
    <div class=row><button class=primary onclick=validateConfig()>Validate config &amp; pose</button></div>
    <div id=validateout class=hint style="margin-top:6px"></div>
    <div class=hint>Checks the config is commissioned, symmetric, knees-back, and every stand angle within range; with hardware it reads all 12 servos and shows the live pose's left/right symmetry. Pose the robot symmetrically first for the symmetry numbers to mean something.</div>
   </div>
   <h2 style=margin-top:14px>Mirror-follow range teach</h2>
   <div class=step>
    <div class=row>master
     <select id=flmaster><option>BL</option><option>BR</option><option>FL</option><option>FR</option></select>
     <label><input type=checkbox id=flj_hip checked> hip</label>
     <label><input type=checkbox id=flj_femur checked> femur</label>
     <label><input type=checkbox id=flj_tibia checked> tibia</label>
    </div>
    <div class=row style="margin-top:6px">
     <button onclick=homeAll()>Home all &#8594; 2048</button>
     <button class=primary onclick=followStart()>Start follow</button>
     <button onclick=followCheck()>Check agreement</button>
     <button class=warn onclick=followStop()>Finish</button>
    </div>
    <div id=followlive class=hint style="margin-top:6px"></div>
    <div class=hint><b>Home</b> centers all servos. <b>Start</b> releases the master leg's checked joints — move them by hand; the other three legs mirror the same canonical motion. For each joint: move it to one extreme and click <b>Set min</b>, the other extreme and <b>Set max</b> — the limit is captured at that pose and applied to all four legs at once. Following keeps running the whole time. A joint shows &#10003; once both ends are set. Watch every leg move the SAME way; a leg moving opposite (or flagged by <b>Check agreement</b>) means its Direction/invert is wrong. Keep <b>Torque OFF all</b> handy; <b>Finish</b> releases torque.</div>
   </div>
  </section>
 </div>
</div>
<script>
const JOINTS=["hip","femur","tibia"];
const state={status:null,pose:{},sel:null,lastRaw:null,rng:{min:200,home:2048,max:3800},emit:null};

async function J(u,m,b){
 const r=await fetch(u,{method:m||"GET",headers:{"Content-Type":"application/json"},body:b?JSON.stringify(b):undefined});
 const j=await r.json().catch(()=>({}));
 if(!r.ok) throw new Error(j.detail||j.message||r.statusText);
 return j;
}
function toast(msg,kind){const el=document.getElementById("toast");el.textContent=msg||"";el.style.color=kind==="bad"?"var(--red)":kind==="ok"?"var(--green)":"var(--muted)";}
function pill(id,txt,kind){const el=document.getElementById(id);el.textContent=txt;el.className="pill "+(kind||"");}

function perBus(){const s=state.status;return (s&&s.discovered&&s.discovered.per_bus)||{};}
function busOfId(id){const pb=perBus();for(const b in pb){if((pb[b]||[]).includes(id))return b;}return null;}
function slotOfId(id){const s=state.status;if(!s)return null;for(const k in s.slots){if(s.slots[k].id===id)return k;}return null;}
function idsByBus(b){return ((perBus()[b])||[]).slice().sort((x,y)=>x-y);}
function dupSet(){const s=state.status;return new Set((s&&s.discovered&&s.discovered.duplicates)||[]);}
function partLabel(slot){return slot?slot.replace("_"," "):"";}
function dotsHtml(v){const keys=["assigned","center","direction","range"];const L=["A","C","D","R"];
 return '<div class=dots>'+L.map((x,i)=>'<span class="dot '+(v&&v[keys[i]]?"on":"")+'">'+x+'</span>').join("")+'</div>';}
function badge(on){return '<span class="badge '+(on?"ok":"")+'">'+(on?"done":"pending")+'</span>';}

async function refresh(){
 try{state.status=await J("/api/status");}catch(e){toast(e.message,"bad");return;}
 const id=state.sel;
 if(id!=null && busOfId(id)==null && slotOfId(id)==null){state.sel=null;}
 renderAll();
}
function renderAll(){renderTop();renderIds();renderMap();renderWork();renderEmit();}

function renderTop(){
 const s=state.status;
 pill("mode",s.dry?"DRY (no hardware)":"HARDWARE",s.dry?"bad":"ok");
 pill("link",s.link||"no link",s.link?"hot":"bad");
 pill("ready",s.ready_to_emit?"READY TO SAVE":"INCOMPLETE",s.ready_to_emit?"ok":"bad");
 const rows=Object.values(s.slots);
 const mapped=rows.filter(v=>v.id!=null).length;
 const verified=rows.filter(v=>v.assigned&&v.center&&v.direction&&v.range).length;
 pill("prog","mapped "+mapped+"/12 - verified "+verified+"/12","");
 document.getElementById("fwdlabel").textContent="FORWARD  -  "+(s.frame.forward_sign||"+Y")+" (toward the head)";
}

function renderIds(){
 const s=state.status;const dups=dupSet();let html="";
 const total=(s.discovered&&s.discovered.total)||0;
 if(!total){html+='<small>No servos detected yet. Click <b>Scan buses</b> with the ESP32 powered and linked.</small>';}
 if(dups.size){html+='<div class=dup>Duplicate IDs on a bus: '+[...dups].join(", ")+'. A half-duplex bus cannot address duplicates - re-ID one of each with the tool below.</div>';}
 for(const bus of Object.keys(s.buses).sort()){
  const m=s.buses[bus];const ids=idsByBus(bus);
  html+='<div class="busgrp '+bus+'"><div class=head><div><b>Bus '+bus+' - '+m.role+'</b><br><small>drives '+m.legs.join(", ")+'</small></div><span class=pill style="background:#fff">'+ids.length+'/6</span></div><div class=chips>';
  if(ids.length){
   html+=ids.map(id=>{
    const slot=slotOfId(id);
    const cls=["chip",id===state.sel?"sel":"",slot?"mapped":"unmapped",dups.has(id)?"dup":""].join(" ");
    return '<button class="'+cls+'" onclick="selectId('+id+')">'+id+(slot?'<small>'+partLabel(slot)+'</small>':'<small>unmapped</small>')+'</button>';
   }).join("");
  }else{html+='<small style="padding:4px">none on this bus</small>';}
  html+='</div></div>';
 }
 document.getElementById("ids").innerHTML=html;
}

function renderMap(){
 const s=state.status;
 const selBus=state.sel!=null?busOfId(state.sel):null;
 const selSlot=state.sel!=null?slotOfId(state.sel):null;
 const legCard=(name)=>{
  const bus=s.slot_meta[name+"_hip"].bus;
  const dim=selBus&&selBus!==bus;
  let h='<div class="leg '+bus+(dim?" dim":"")+'"><div class=legtitle><span>'+name+'</span><span class=tag>Bus '+bus+'</span></div>';
  for(const j of JOINTS){
   const slot=name+"_"+j;const v=s.slots[slot];const filled=v.id!=null;
   const drop=selBus&&selBus===bus&&!filled;
   const isSel=selSlot===slot;
   const live=state.pose[slot];const liveTxt=live?(Math.round(live.deg)+"deg"):"";
   const dis=(selBus&&selBus!==bus)?"disabled":"";
   h+='<button class="slot '+(filled?"filled":"")+' '+(drop?"drop":"")+' '+(isSel?"sel":"")+'" data-slot="'+slot+'" '+dis+'>'
    +'<span class=j>'+j+'</span><span class=id>'+(filled?("ID "+v.id):"-")+'</span>'
    +'<span class=live id="live-'+slot+'">'+liveTxt+'</span><span></span>'
    +dotsHtml(v)+'</button>';
  }
  return h+'</div>';
 };
 let html='<div class="maprow front">'+legCard("FL")+legCard("FR")+'</div>';
 html+='<div class="maprow rear">'+legCard("BL")+legCard("BR")+'</div>';
 document.getElementById("map").innerHTML=html;
 document.getElementById("legend").innerHTML='Direction check - <b>hip</b>: foot swings OUTWARD from centerline. <b>femur</b>: leg rotates FORWARD (toward the arrow). <b>tibia</b>: knee FLEXES (foot drawn under). Dots A/C/D/R = assigned, center, direction, range.';
}

function selectId(id){state.sel=id;state.lastRaw=null;renderAll();toast("ID "+id+" selected - Wiggle to see which joint moves");}

async function assignSel(slot){
 if(state.sel==null){toast("Select a detected ID first","bad");return;}
 const bus=busOfId(state.sel);
 try{
  await J("/api/assign","POST",{servo_id:state.sel,slot});
  toast("ID "+state.sel+" mapped to "+partLabel(slot),"ok");
  await refresh();
  const next=nextUnmapped(bus);
  if(next!=null){state.sel=next;renderAll();}
 }catch(e){toast(e.message,"bad");}
}
function nextUnmapped(preferBus){
 const s=state.status;const all=[];
 for(const b of Object.keys(s.buses).sort()){for(const id of idsByBus(b)){all.push(id);}}
 const same=all.filter(id=>busOfId(id)===preferBus&&slotOfId(id)==null);
 if(same.length)return same[0];
 const any=all.filter(id=>slotOfId(id)==null);
 return any.length?any[0]:null;
}

function renderWork(){
 const s=state.status;const id=state.sel;const el=document.getElementById("work");
 if(id==null){el.innerHTML='<small>Select a detected ID from Bus A or Bus B to begin. Then <b>Wiggle</b> it, watch which joint twitches, and click that joint on the body map.</small>';return;}
 const bus=busOfId(id);const slot=slotOfId(id);
 const role=bus?s.buses[bus].role:"?";
 const meta=slot?s.slot_meta[slot]:null;const v=slot?s.slots[slot]:null;
 let h='<div class=selhead><div class=kv><b>ID</b><span>'+id+'</span><b>Bus</b><span>'+(bus||"?")+' ('+role+')</span><b>Part</b><span>'+(slot?partLabel(slot):"not mapped yet")+'</span></div></div>';
 h+='<div class=step><div class=st><span>1 - Identify</span></div><div class=row><button class=primary onclick=wiggle()>Wiggle</button><small>Watch which joint twitches.</small></div>';
 if(!slot){h+='<div class=hint>Then click the matching joint on the body map. Only <b>Bus '+bus+' ('+role+')</b> slots are selectable, so a '+role+' servo cannot be mapped to the wrong end.</div>';}
 h+='</div>';
 if(slot){
  h+='<div class="step '+(v.center?"done":"")+'"><div class=st><span>2 - Center</span>'+badge(v.center)+'</div>'
   +'<div class=row><button onclick=torque(false)>Torque off</button><button onclick=torque(true)>Hold</button><button class=good onclick=center()>Set center</button></div>'
   +'<div class=hint>Torque off, move the joint by hand to true neutral (leg straight down, hips square), then Set center.</div></div>';
  const q=s.direction_questions[meta.joint];
  h+='<div class="step '+(v.direction?"done":"")+'"><div class=st><span>3 - Direction</span>'+badge(v.direction)+'</div>'
   +'<div class=q>'+q+'</div>'
   +'<div class=hint>FORWARD is toward the head (the arrow, '+(s.frame.forward_sign||"+Y")+'). Probe sends a small canonical-positive move; answer Correct or Invert. Invert only flips this servo encoder - it does not touch side or knee branch.</div>'
   +'<div class=row><button class=primary onclick=dirProbe()>Probe</button><button class=good onclick=dirSetOk()>Correct</button><button class=warn onclick=dirSetNo()>Invert</button></div></div>';
  h+='<div class="step '+(v.range?"done":"")+'"><div class=st><span>4 - Range</span>'+badge(v.range)+'</div>'
   +'<div class=rng><label>min</label><input id=mn type=number value="'+state.rng.min+'" oninput=capState()><button onclick=capMn()>= now</button></div>'
   +'<div class=rng><label>home</label><input id=hm type=number value="'+state.rng.home+'" oninput=capState()><button onclick=capHm()>= now</button></div>'
   +'<div class=rng><label>max</label><input id=mx type=number value="'+state.rng.max+'" oninput=capState()><button onclick=capMx()>= now</button></div>'
   +'<div class=row><button class=good onclick=setSoftRange()>Set soft range</button><button onclick=setRangeEeprom()>Set + EEPROM</button></div>'
   +'<div class=hint>Torque off; move to each safe extreme and click "= now". Home = 2048.</div>'
   +'<div class=row style="margin-top:8px;border-top:1px dashed var(--line);padding-top:8px"><button onclick=mirrorLeg() title="Copy this leg\\'s three trained ranges onto the other three legs">Mirror '+meta.leg+' &#8594; other 3 legs</button></div>'
   +'<div class=hint>Train all three joints of <b>'+meta.leg+'</b> first (all show <b>R</b>), then this copies its ranges to the matching joints on the other legs, auto-inverting per servo. Soft limits only; verify each leg once.</div></div>';
  const d=state.pose[slot];
  h+='<div class=hint id=liveread>live: raw '+(d?d.raw:"-")+' - '+(d?d.deg.toFixed(1)+"deg":"-")+' - last read '+(state.lastRaw==null?"-":state.lastRaw)+'</div>';
 }
 el.innerHTML=h;
}

function renderEmit(){
 const s=state.status;const rows=Object.values(s.slots);const c=k=>rows.filter(v=>v[k]).length;
 let h='<div>assigned '+c("assigned")+'/12 - center '+c("center")+'/12 - direction '+c("direction")+'/12 - range '+c("range")+'/12</div>';
 h+='<div class=hint>'+(s.ready_to_emit?'<b style="color:var(--green)">All 12 servos verified - Save writes a commissioned robot_config.json.</b>':'Finish every A/C/D/R, then Save.')+'</div>';
 if(state.emit){
  h+='<div class=hint>'+(state.emit.ready?'<b style="color:var(--green)">ready</b>':'<b style="color:var(--red)">incomplete</b>')+' - saved '+state.emit.saved+'</div>';
  const w=state.emit.warnings||[];
  if(w.length){h+='<div class=hint style="color:var(--red)">'+w.slice(0,6).join("<br>")+(w.length>6?"<br>...":"")+'</div>';}
 }
 document.getElementById("emit").innerHTML=h;
}

function paintLive(){
 for(const slot in state.pose){const e=document.getElementById("live-"+slot);if(e){const d=state.pose[slot];e.textContent=d?Math.round(d.deg)+"deg":"";}}
 const sel=state.sel!=null?slotOfId(state.sel):null;const lr=document.getElementById("liveread");
 if(lr&&sel){const d=state.pose[sel];lr.textContent='live: raw '+(d?d.raw:"-")+' - '+(d?d.deg.toFixed(1)+"deg":"-")+' - last read '+(state.lastRaw==null?"-":state.lastRaw);}
}

function capState(){const g=id=>{const e=document.getElementById(id);return e?+e.value:null;};
 const mn=g("mn"),hm=g("hm"),mx=g("mx");
 if(mn!=null)state.rng.min=mn;if(hm!=null)state.rng.home=hm;if(mx!=null)state.rng.max=mx;}
async function readRaw(){const r=await J("/api/raw/"+state.sel);state.lastRaw=r.raw;paintLive();return r.raw;}
async function cap(field){try{const raw=await readRaw();const e=document.getElementById(field);if(e)e.value=raw;capState();}catch(e){toast(e.message,"bad");}}
function capMn(){cap("mn");}
function capHm(){cap("hm");}
function capMx(){cap("mx");}

async function verify(){try{const r=await J("/api/verify_firmware","POST");toast(r.pass?"firmware PASS":"firmware FAIL",r.pass?"ok":"bad");}catch(e){toast(e.message,"bad");}}
async function scan(){try{
 const r=await J("/api/discover","POST");
 const nd=(r.duplicates||[]).length;
 toast("scan found "+r.total+" IDs"+(nd?" ("+nd+" duplicate)":""),(r.total===12&&!nd)?"ok":undefined);
 await refresh();
 if(state.sel==null){const n=nextUnmapped(null);if(n!=null){state.sel=n;renderAll();}}
}catch(e){toast(e.message,"bad");}}
async function wiggle(){if(state.sel==null)return toast("Select an ID first","bad");
 try{const r=await J("/api/wiggle/"+state.sel,"POST");toast(r.moved?"wiggle sent - which joint moved?":"wiggle failed",r.moved?"ok":"bad");}catch(e){toast(e.message,"bad");}}
async function torque(on){try{const r=await J("/api/torque/"+state.sel,"POST",{enable:on});toast(r.ok?(on?"holding":"torque off"):"torque failed",r.ok?"ok":"bad");}catch(e){toast(e.message,"bad");}}
async function torqueAllOff(){
 try{
  const r=await J("/api/torque_all","POST",{enable:false});
  clearFollowUi();
  const res=r.results||[];const bad=res.filter(x=>!x.ok);
  if(bad.length){toast("released "+(res.length-bad.length)+"/"+res.length+" - NO RELEASE: "+bad.map(x=>x.slot+" (#"+x.id+")").join(", "),"bad");}
  else{toast("all "+res.length+" servos released","ok");}
 }catch(e){toast(e.message,"bad");}
}
async function mirrorLeg(){
 const slot=state.sel!=null?slotOfId(state.sel):null;
 if(!slot){toast("select a mapped servo on the leg you trained first","bad");return;}
 const leg=slot.split("_")[0];
 try{
  const r=await J("/api/mirror_ranges","POST",{source_leg:leg});
  toast("mirrored "+leg+" -> "+r.count+" servos on the other legs","ok");
  await refresh();
 }catch(e){toast(e.message,"bad");}
}
let followTimer=null;
function clearFollowUi(){
 if(followTimer){clearInterval(followTimer);followTimer=null;}
 const el=document.getElementById("followlive");if(el)el.innerHTML="";
}
function followCfg(){
 const leg=document.getElementById("flmaster").value;
 const free=["hip","femur","tibia"].filter(j=>document.getElementById("flj_"+j).checked);
 return {master_leg:leg,free_joints:free};
}
async function homeAll(){try{await J("/api/home_all","POST");clearFollowUi();toast("homing all servos to center (2048)","ok");}catch(e){toast(e.message,"bad");}}
async function followStart(){
 const cfg=followCfg();
 if(!cfg.free_joints.length){toast("check at least one joint to free","bad");return;}
 try{
  await J("/api/follow/start","POST",cfg);
  toast("centered; "+cfg.master_leg+" "+cfg.free_joints.join("/")+" released — move it by hand","ok");
  buildFollowSkeleton(cfg.master_leg,cfg.free_joints);
  if(followTimer)clearInterval(followTimer);
  followTimer=setInterval(followTick,120);
 }catch(e){toast(e.message,"bad");}
}
async function followTick(){
 try{const r=await J("/api/follow/tick","POST");updateFollow(r);}catch(e){}
}
function buildFollowSkeleton(master,free){
 const el=document.getElementById("followlive");if(!el)return;
 let h='<b>Master '+master+'</b> — move a joint to an extreme, then Set min / Set max:<table style="width:100%;font-size:12px;margin-top:3px">';
 for(const j of free){
  h+='<tr><td><b>'+j+'</b> <span id="fa_'+j+'">-</span></td>'
   +'<td><button onclick="followSetLimit(&quot;'+j+'&quot;,&quot;min&quot;)">Set min</button> '
   +'<button onclick="followSetLimit(&quot;'+j+'&quot;,&quot;max&quot;)">Set max</button></td>'
   +'<td><span id="fr_'+j+'">[-, -]</span></td></tr>';
 }
 h+='</table><div style="margin-top:4px"><button onclick="followSetLimit(&quot;all&quot;,&quot;min&quot;)">All min</button> <button onclick="followSetLimit(&quot;all&quot;,&quot;max&quot;)">All max</button></div>';
 el.innerHTML=h;
}
function updateFollow(r){
 if(!r||!r.joints)return;
 for(const j in r.joints){const d=r.joints[j];
  const a=document.getElementById("fa_"+j);if(a)a.textContent=d.angle_deg+" deg";
  const rr=document.getElementById("fr_"+j);if(rr)rr.textContent="["+(d.min_deg==null?"-":d.min_deg)+", "+(d.max_deg==null?"-":d.max_deg)+"]"+(d.verified?" OK":"");
 }
}
function updateFollowCapture(set){
 for(const j in set){const d=set[j];
  const rr=document.getElementById("fr_"+j);if(rr)rr.textContent="["+(d.min_deg==null?"-":d.min_deg)+", "+(d.max_deg==null?"-":d.max_deg)+"]"+(d.applied?" OK":"");
 }
}
async function followSetLimit(joint,which){
 try{const r=await J("/api/follow/set_limit","POST",{joint:joint,which:which});
  updateFollowCapture(r.set||{});
  const vals=Object.values(r.set||{});
  toast((joint==='all'?'all':joint)+" "+which+" captured"+(vals.some(x=>x.applied)?" - range applied to all legs":" - now set the other end"),"ok");
 }catch(e){toast(e.message,"bad");}
}
async function followCheck(){
 try{
  const r=await J("/api/follow/check","POST");
  const bad=[];
  for(const j in r.joints){const dis=(r.joints[j].followers||[]).filter(x=>!x.agrees).map(x=>x.leg);
   if(dis.length)bad.push(j+": "+dis.join(","));}
  toast(bad.length?("DISAGREE (check invert/visual) -> "+bad.join(" | ")):("all legs agree within "+r.tolerance_deg+"deg"),bad.length?"bad":"ok");
 }catch(e){toast(e.message,"bad");}
}
async function followStop(){
 try{
  const r=await J("/api/follow/stop","POST");
  clearFollowUi();
  const pend=(r.pending&&r.pending.length)?" - still pending: "+r.pending.join("/"):"";
  toast("finished; ranged "+((r.verified||[]).join("/")||"none")+pend,pend?undefined:"ok");
  await refresh();
 }catch(e){clearFollowUi();toast(e.message,"bad");}
}
async function center(){try{const r=await J("/api/center/"+state.sel,"POST");toast(r.centered?"center set":"center failed - is the joint at neutral?",r.centered?"ok":"bad");await refresh();}catch(e){toast(e.message,"bad");}}
async function dirProbe(){try{const r=await J("/api/direction/probe/"+state.sel,"POST");toast(r.probed?"probe sent - watch the joint":"probe failed",r.probed?"ok":"bad");}catch(e){toast(e.message,"bad");}}
async function dirSet(ok){try{await J("/api/direction/set","POST",{servo_id:state.sel,moved_correctly:ok});toast(ok?"direction confirmed":"invert set","ok");await refresh();}catch(e){toast(e.message,"bad");}}
function dirSetOk(){dirSet(true);}
function dirSetNo(){dirSet(false);}
async function setRange(eeprom){capState();
 try{await J("/api/range","POST",{servo_id:state.sel,min_raw:state.rng.min,home_raw:state.rng.home,max_raw:state.rng.max,write_eeprom:!!eeprom});
  toast(eeprom?"range + EEPROM set":"soft range set","ok");await refresh();}catch(e){toast(e.message,"bad");}}
function setSoftRange(){setRange(false);}
function setRangeEeprom(){setRange(true);}
async function emit(){try{const r=await J("/api/emit","POST");state.emit=r;toast("config saved",r.ready?"ok":undefined);await refresh();}catch(e){toast(e.message,"bad");}}
async function validateConfig(){
 try{
  const r=await J("/api/validate","POST");
  const c=r.config||{};const iss=c.issues||[];
  let h='<b>Config: '+(c.ok?(c.symmetric?'<span style="color:var(--green)">VALID &amp; SYMMETRIC</span>':'<span style="color:var(--amber)">valid (warnings)</span>'):'<span style="color:var(--red)">NOT VALID</span>')+'</b>';
  h+='<ul style="margin:4px 0 0 16px;padding:0">';
  for(const i of iss){const col=i.level==='error'?'var(--red)':(i.level==='warn'?'var(--amber)':'var(--green)');h+='<li style="color:'+col+'">'+i.message+'</li>';}
  h+='</ul>';
  if(r.symmetry){h+='<div style="margin-top:6px"><b>Live pose symmetry (deg, ~0 = symmetric):</b><table style="width:100%;font-size:12px"><tr><td></td><td>front L/R</td><td>rear L/R</td><td>L f/r</td></tr>';
   for(const j in r.symmetry){const d=r.symmetry[j];h+='<tr><td><b>'+j+'</b></td><td>'+d.front_left_right_deg+'</td><td>'+d.rear_left_right_deg+'</td><td>'+d.left_front_rear_deg+'</td></tr>';}
   h+='</table></div>';}
  document.getElementById('validateout').innerHTML=h;
  toast(c.ok?(c.symmetric?'valid & symmetric':'valid, warnings'):'not valid - see report',c.ok?(c.symmetric?'ok':'bad'):'bad');
 }catch(e){toast(e.message,'bad');}
}
async function setId(){
 const bus=document.getElementById("idbus").value||null;
 const nid=+document.getElementById("newid").value;
 try{const r=await J("/api/set_id","POST",{new_id:nid,bus});
  document.getElementById("idout").innerHTML='<span style="color:var(--green)">'+r.message+'</span>';
  document.getElementById("newid").value=nid+1;await refresh();
 }catch(e){document.getElementById("idout").innerHTML='<span style="color:var(--red)">'+e.message+'</span>';}
}

document.getElementById("map").addEventListener("click",ev=>{
 const b=ev.target.closest("button.slot");
 if(!b||b.disabled)return;
 assignSel(b.dataset.slot);
});
const ws=new WebSocket((location.protocol==="https:"?"wss":"ws")+"://"+location.host+"/ws/pose");
ws.onmessage=e=>{try{const d=JSON.parse(e.data);state.pose=d.pose||{};paintLive();}catch(_){}};
refresh();setInterval(refresh,4000);
</script></body></html>"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="DogV3 Setup Program server (D1 commissioning GUI)")
    ap.add_argument("--config", default="robot_config.json")
    ap.add_argument("--port-serial", default=None, help="ESP32 serial port (omit for dry mode)")
    ap.add_argument("--tcp", default=None, help="ESP32 WiFi bridge HOST[:PORT] (e.g. 192.168.4.1:3333)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true", help="do not auto-open the browser")
    args = ap.parse_args(argv)

    cfg_path = Path(args.config)
    try:
        config = load_config(cfg_path)
    except ConfigError:
        config = default_config()  # start fresh from the SAMPLE config

    transport = make_transport(args.port_serial, args.tcp)
    driver = FeetechDriver(transport) if transport is not None else None
    commissioner = Commissioner(config, driver)
    link = args.tcp or args.port_serial

    if FastAPI is None:
        print("FastAPI/uvicorn not installed; cannot run server.")
        return 1
    app = build_app(commissioner, transport, link, cfg_path)
    view_host = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
    url = f"http://{view_host}:{args.port}"
    print(f"\n  DogV3 Setup (D1)\n  Open {url}\n  link={'DRY (no hardware)' if transport is None else link}\n")
    if not args.no_browser:
        import threading
        import webbrowser

        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
