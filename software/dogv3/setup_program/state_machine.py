"""Resumable commissioning state machine (Program 1 core).

Drives the operator through: VERIFY -> DISCOVER -> ASSIGN -> CENTER ->
DIRECTION -> RANGE -> VERIFY(visualizer) -> TUNE -> EMIT. Each servo's
``verified`` bits track progress so the process can be paused, resumed, and
re-run per joint.

Hardware actions go through a :class:`FeetechDriver`; the bookkeeping that
mutates the config is pure and unit-testable without hardware. Every jog is a
small bounded relative move at low speed, one servo at a time (guardrail).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from ..config.schema import RobotConfig, ServoSpec, VerifiedBits
from ..driver.feetech import FeetechDriver
from ..driver.mux import Bus
from ..kinematics.conventions import physical_sign
from ..kinematics.leg import COUNTS_PER_RAD, NEUTRAL_COUNT, angle_to_count, count_to_angle

JOINTS = ("hip", "femur", "tibia")
LEGS = ("FL", "FR", "BL", "BR")

# Commissioning jog parameters (guardrails).
JOG_DEG = 8.0
JOG_SPEED = 200      # counts/s-ish; low speed
JOG_ACC = 20
JOG_COUNTS = round(math.radians(JOG_DEG) * COUNTS_PER_RAD)

# Live mirror-follow range teach (safe hand-guided ranging).
HOME_SPEED = 300         # gentle speed for centering all servos to 2048
FOLLOW_SPEED = 500       # follower tracking speed; slow enough to react to a wrong invert
FOLLOW_ACC = 20
AGREE_TOL_DEG = 4.0      # canonical-angle agreement tolerance for the live invert check
MIN_SWEEP_DEG = 5.0      # a follow session needs at least this much master travel to set a range


class Stage(str, Enum):
    VERIFY_FIRMWARE = "verify_firmware"
    DISCOVER = "discover"
    ASSIGN = "assign"
    CENTER = "center"
    DIRECTION = "direction"
    RANGE = "range"
    VISUALIZE = "visualize"
    TUNE = "tune"
    EMIT = "emit"


# Physical-outcome question per joint (DIRECTION stage). Never CW/CCW or counts.
DIRECTION_QUESTION = {
    "hip": "Did the foot swing OUTWARD from the centerline?",
    "femur": "Did the leg rotate FORWARD (toward the head)?",
    "tibia": "Did the knee FLEX (foot drawn under)?",
}


@dataclass
class DiscoverResult:
    per_bus: dict[str, list[int]] = field(default_factory=dict)
    duplicates: list[int] = field(default_factory=list)
    total: int = 0

    @property
    def all_ids(self) -> list[int]:
        ids: list[int] = []
        for v in self.per_bus.values():
            ids.extend(v)
        return ids


def _bus_enum(name: str) -> Bus:
    return Bus.from_name(name)


class Commissioner:
    """Holds the working config + (optional) driver and applies each stage."""

    def __init__(self, config: RobotConfig, driver: Optional[FeetechDriver] = None):
        self.config = config
        self.driver = driver
        self.discovered: DiscoverResult = DiscoverResult()
        # Map discovered id -> bus letter, learned during discover.
        self._id_bus: dict[int, str] = {}
        # Active live mirror-follow session, or None. Holds master leg, the free
        # joint set, and explicit per-joint limit captures for this session.
        self._follow: Optional[dict] = None

    # ---- Stage 2: DISCOVER --------------------------------------------
    def discover(self) -> DiscoverResult:
        if self.driver is None:
            raise RuntimeError("discover requires a connected driver")
        found = self.driver.scan()
        per_bus: dict[str, list[int]] = {}
        seen: dict[int, int] = {}
        self._id_bus.clear()
        for bus, ids in found.items():
            letter = bus.name_letter
            per_bus[letter] = sorted(ids)
            for i in ids:
                seen[i] = seen.get(i, 0) + 1
                self._id_bus[i] = letter
        duplicates = sorted([i for i, c in seen.items() if c > 1])
        self.discovered = DiscoverResult(
            per_bus=per_bus, duplicates=duplicates, total=len(seen)
        )
        return self.discovered

    def candidate_slots(self, servo_id: int) -> list[str]:
        """Slots a discovered ID may occupy, restricted to its bus (front=B,
        rear=A) to halve choices and catch miswires."""
        bus_letter = self._id_bus.get(servo_id)
        if bus_letter is None:
            # Unknown bus -> allow all unfilled slots.
            return [s for s in self._all_slots() if self.config.servo_by_slot(s) is None]
        out = []
        for leg in LEGS:
            if self.config.legs[leg].bus != bus_letter:
                continue
            for joint in JOINTS:
                slot = f"{leg}_{joint}"
                if self.config.servo_by_slot(slot) is None:
                    out.append(slot)
        return out

    @staticmethod
    def _all_slots() -> list[str]:
        return [f"{leg}_{joint}" for leg in LEGS for joint in JOINTS]

    # ---- Stage 3: ASSIGN (wiggle-to-identify) -------------------------
    def wiggle(self, servo_id: int) -> bool:
        """Torque on only this servo, jog a small bounded relative delta and
        return. The operator watches which physical joint moves."""
        if self.driver is None:
            raise RuntimeError("wiggle requires a connected driver")
        bus = _bus_enum(self._id_bus.get(servo_id, "A"))
        if not self.driver.enable_torque(bus, servo_id, True):
            return False
        present = self.driver.read_present_position_raw(bus, servo_id)
        if present is None:
            return False
        target = max(0, min(4095, present + JOG_COUNTS))
        self.driver.write_pos_ex(bus, servo_id, target, JOG_SPEED, JOG_ACC)
        time.sleep(0.5)
        self.driver.write_pos_ex(bus, servo_id, present, JOG_SPEED, JOG_ACC)
        time.sleep(0.5)
        return True

    def assign(self, servo_id: int, slot: str) -> ServoSpec:
        """Record id<->slot. Restricted-to-bus check happens in the UI via
        :meth:`candidate_slots`; here we also enforce it."""
        leg, joint = slot.split("_")
        if joint not in JOINTS or leg not in LEGS:
            raise ValueError(f"bad slot {slot}")
        # Remove any prior assignment of this id or slot (allow re-assign).
        self._clear_id(servo_id)
        self._clear_slot(slot)
        setattr(self.config.legs[leg].ids, joint, servo_id)
        spec = ServoSpec(slot=slot)
        spec.verified.assigned = True
        self.config.servos[str(servo_id)] = spec
        return spec

    def _clear_id(self, servo_id: int) -> None:
        self.config.servos.pop(str(servo_id), None)
        for leg in LEGS:
            for joint in JOINTS:
                if getattr(self.config.legs[leg].ids, joint) == servo_id:
                    setattr(self.config.legs[leg].ids, joint, None)

    def _clear_slot(self, slot: str) -> None:
        leg, joint = slot.split("_")
        prev_id = getattr(self.config.legs[leg].ids, joint)
        if prev_id is not None:
            self.config.servos.pop(str(prev_id), None)
            setattr(self.config.legs[leg].ids, joint, None)

    # ---- Manual helpers (needed to hand-position during CENTER/RANGE) --
    def _bus_for(self, servo_id: int) -> Bus:
        """Resolve a servo's bus from its assigned slot, else from discovery."""
        spec = self.config.servos.get(str(servo_id))
        if spec is not None:
            return _bus_enum(self._slot_bus(spec.slot))
        return _bus_enum(self._id_bus.get(servo_id, "A"))

    def set_torque(self, servo_id: int, enable: bool) -> bool:
        """Release (False) or hold (True) a single servo so the operator can
        move the joint by hand for centering and range-finding."""
        if self.driver is None:
            raise RuntimeError("set_torque requires a connected driver")
        return self.driver.enable_torque(self._bus_for(servo_id), servo_id, enable)

    def read_raw(self, servo_id: int) -> int | None:
        """Read a servo's present raw position (for capturing limits by hand)."""
        if self.driver is None:
            raise RuntimeError("read_raw requires a connected driver")
        return self.driver.read_present_position_raw(self._bus_for(servo_id), servo_id)

    # ---- Stage 4: CENTER (CalibrationOfs) -----------------------------
    def center(self, servo_id: int) -> bool:
        """Operator has the joint at neutral; store current pos as middle so
        present position reads ~2048. Re-read to confirm."""
        if self.driver is None:
            raise RuntimeError("center requires a connected driver")
        spec = self.config.servos.get(str(servo_id))
        if spec is None:
            raise ValueError(f"servo {servo_id} not assigned")
        bus = _bus_enum(self._slot_bus(spec.slot))
        if not self.driver.calibration_ofs(bus, servo_id):
            return False
        time.sleep(0.2)
        pos = self.driver.read_present_position_raw(bus, servo_id)
        if pos is None or abs(pos - NEUTRAL_COUNT) > 50:
            return False
        spec.ofs_calibrated = True
        spec.home_raw = NEUTRAL_COUNT
        spec.verified.center = True
        return True

    # ---- Stage 5: DIRECTION -------------------------------------------
    def probe_direction(self, servo_id: int) -> bool:
        """Command a small canonical-positive move so the operator can answer
        the physical-outcome question."""
        if self.driver is None:
            raise RuntimeError("direction probe requires a connected driver")
        spec = self.config.servos[str(servo_id)]
        bus = _bus_enum(self._slot_bus(spec.slot))
        # Canonical-positive in count space depends on current invert.
        direction = -1 if spec.invert else 1
        target = max(0, min(4095, NEUTRAL_COUNT + direction * JOG_COUNTS))
        self.driver.enable_torque(bus, servo_id, True)
        self.driver.write_pos_ex(bus, servo_id, target, JOG_SPEED, JOG_ACC)
        time.sleep(0.5)
        self.driver.write_pos_ex(bus, servo_id, NEUTRAL_COUNT, JOG_SPEED, JOG_ACC)
        time.sleep(0.4)
        return True

    def set_direction(self, servo_id: int, moved_correctly: bool) -> ServoSpec:
        """Keep the probed direction when correct; flip it when incorrect.

        :meth:`probe_direction` already applies the servo's current ``invert``
        while generating its canonical-positive move.  Therefore a correct
        observation confirms the existing value (including ``True`` on a
        previously commissioned servo), while an incorrect observation means
        that value must be toggled. ``side`` remains independent.
        """
        spec = self.config.servos[str(servo_id)]
        if not moved_correctly:
            spec.invert = not spec.invert
        spec.verified.direction = True
        return spec

    # ---- Stage 6: RANGE -----------------------------------------------
    def set_range(
        self,
        servo_id: int,
        min_raw: int,
        home_raw: int,
        max_raw: int,
        margin_deg: float = 5.0,
        write_eeprom: bool = False,
    ) -> ServoSpec:
        spec = self.config.servos[str(servo_id)]
        # An inverted servo makes the operator capture the two extremes with
        # min_raw > max_raw. Keep the raw bounds ordered so soft limits, the
        # ServoSpec validator, and — critically — the EEPROM hard-limit write
        # never get an inverted window. An inverted MIN/MAX burned into a servo
        # breaks its position clamp and it then drives continuously in one
        # direction (a "runaway" servo).
        if min_raw > max_raw:
            min_raw, max_raw = max_raw, min_raw
        spec.min_raw = min_raw
        spec.home_raw = home_raw
        spec.max_raw = max_raw
        margin = margin_deg
        # Soft limits a margin inside the mechanical stops (in joint degrees).
        hard_min_deg, hard_max_deg = sorted((
            self._count_to_deg(min_raw, spec),
            self._count_to_deg(max_raw, spec),
        ))
        inset = min(margin, max(0.0, (hard_max_deg - hard_min_deg) / 2.0))
        spec.soft_min_deg = hard_min_deg + inset
        spec.soft_max_deg = hard_max_deg - inset
        if write_eeprom and self.driver is not None:
            bus = _bus_enum(self._slot_bus(spec.slot))
            if self.driver.write_angle_limits(bus, servo_id, min_raw, max_raw):
                spec.eeprom_min = min_raw
                spec.eeprom_max = max_raw
        spec.verified.range = True
        return spec

    @staticmethod
    def _count_to_deg(count: int, spec: ServoSpec) -> float:
        direction = -1 if spec.invert else 1
        return math.degrees(direction * (count - NEUTRAL_COUNT) / COUNTS_PER_RAD)

    # ---- Mass torque control (safety) ---------------------------------
    def set_all_torque(self, enable: bool) -> list[dict]:
        """Torque on/off *every* assigned servo in one call — the mass safety
        control. Returns a per-servo ``{id, slot, bus, ok}`` list so the operator
        can see which servos actually acknowledged (a servo that won't release
        shows ``ok=False``). Torque-off is retried once per servo since a dropped
        reply on the half-duplex bus is the common failure."""
        if self.driver is None:
            raise RuntimeError("set_all_torque requires a connected driver")
        if not enable:
            self._follow = None
        results: list[dict] = []
        for leg in LEGS:
            for joint in JOINTS:
                sid = getattr(self.config.legs[leg].ids, joint)
                if sid is None:
                    continue
                bus = self._bus_for(sid)
                ok = False
                try:
                    ok = self.driver.enable_torque(bus, sid, enable)
                    if not ok:
                        ok = self.driver.enable_torque(bus, sid, enable)  # one retry
                except Exception:
                    ok = False
                results.append({"id": sid, "slot": f"{leg}_{joint}", "bus": bus.name_letter, "ok": bool(ok)})
        return results

    # ---- Mirror ranges from one trained leg ---------------------------
    def mirror_ranges(self, source_leg: str) -> dict:
        """Copy *source_leg*'s three trained joint ranges onto the matching joints
        of the other three legs.

        The transform goes through the canonical joint-angle domain so it is
        correct regardless of the SolidWorks double-mirror (left/right + front/
        rear) and the inner-facing wire inversions: each source limit is turned
        into a canonical angle using the *source* servo's ``invert``, then back
        into raw counts using each *target* servo's own ``invert``. Because every
        ``invert`` was already fixed in the DIRECTION step, a target whose invert
        differs comes out reflected about 2048; one that matches comes out
        identical. Writes soft limits only (no EEPROM), and marks range verified."""
        if source_leg not in LEGS:
            raise ValueError(f"unknown leg {source_leg!r}")
        # Source leg must be fully range-trained first.
        for joint in JOINTS:
            sid = getattr(self.config.legs[source_leg].ids, joint)
            spec = self.config.servos.get(str(sid)) if sid is not None else None
            if spec is None or not spec.verified.range:
                raise ValueError(f"train all three ranges on {source_leg} first ({joint} not done)")
        changed: list[dict] = []
        for joint in JOINTS:
            src = self.config.servos[str(getattr(self.config.legs[source_leg].ids, joint))]
            a = count_to_angle(src.min_raw, invert=src.invert)
            b = count_to_angle(src.max_raw, invert=src.invert)
            lo, hi = (a, b) if a <= b else (b, a)  # canonical-frame angle envelope (rad)
            for leg in LEGS:
                if leg == source_leg:
                    continue
                tid = getattr(self.config.legs[leg].ids, joint)
                tgt = self.config.servos.get(str(tid)) if tid is not None else None
                if tgt is None:
                    raise ValueError(f"{leg}_{joint} is not assigned yet; map all 12 before mirroring")
                if not tgt.verified.direction:
                    raise ValueError(f"{leg}_{joint} (ID {tid}) has no verified direction; do its Direction step before mirroring")
                ra = angle_to_count(lo, invert=tgt.invert)
                rb = angle_to_count(hi, invert=tgt.invert)
                mn, mx = (ra, rb) if ra <= rb else (rb, ra)
                self.set_range(tid, mn, NEUTRAL_COUNT, mx, write_eeprom=False)
                changed.append({"leg": leg, "joint": joint, "id": tid, "min_raw": mn, "max_raw": mx})
        return {"source": source_leg, "count": len(changed), "changed": changed}

    # ---- Live mirror-follow range teach -------------------------------
    def home_all(self, speed: int = HOME_SPEED) -> None:
        """Torque on every assigned servo and drive it to center (2048) at a
        gentle speed. One sync-write per bus; commands, does not block on arrival."""
        if self.driver is None:
            raise RuntimeError("home_all requires a connected driver")
        self._follow = None
        plan: dict[Bus, list[tuple[int, int, int, int]]] = {Bus.A: [], Bus.B: []}
        for leg in LEGS:
            for joint in JOINTS:
                sid = getattr(self.config.legs[leg].ids, joint)
                if sid is None:
                    continue
                bus = self._bus_for(sid)
                self.driver.enable_torque(bus, sid, True)
                plan[bus].append((sid, NEUTRAL_COUNT, speed, FOLLOW_ACC))
        self.driver.sync_write_positions(plan)

    def follow_start(self, master_leg: str, free_joints: list[str]) -> dict:
        """Home all servos to center, then release torque on the *master* leg's
        selected joints so the operator can move them by hand. The matching joint
        on the other three legs will mirror the master's canonical angle on each
        :meth:`follow_tick`. Requires a verified direction on every involved servo
        (invert must be known for the mirror to be meaningful)."""
        if self.driver is None:
            raise RuntimeError("follow requires a connected driver")
        if master_leg not in LEGS:
            raise ValueError(f"unknown leg {master_leg!r}")
        free = [j for j in JOINTS if j in (free_joints or list(JOINTS))]
        if not free:
            raise ValueError("select at least one joint to free")
        for j in free:
            for leg in LEGS:
                sid = getattr(self.config.legs[leg].ids, j)
                spec = self.config.servos.get(str(sid)) if sid is not None else None
                if spec is None:
                    raise ValueError(f"{leg}_{j} is not assigned yet")
                if not spec.verified.direction:
                    raise ValueError(f"{leg}_{j} (ID {sid}) has no verified direction; finish its Direction step first")
        self.home_all()
        time.sleep(1.5)  # let the servos reach center before releasing the master
        for j in free:
            sid = getattr(self.config.legs[master_leg].ids, j)
            self.driver.enable_torque(self._bus_for(sid), sid, False)
        # Min/max are captured explicitly per joint via follow_set_limit. Track
        # current-session application separately from any old saved range bits.
        self._follow = {"master": master_leg, "free": free, "captured": {j: {"applied": False} for j in free}}
        return {"master": master_leg, "free": free}

    def _phys_sign(self, leg: str, joint: str) -> int:
        """This leg's servo :func:`physical_sign` — +1 if raw increases for the
        physical-positive move (outward/forward/flex). See kinematics.conventions."""
        return physical_sign(
            self.config.legs[leg].side,
            self.config.servos[str(getattr(self.config.legs[leg].ids, joint))].invert,
            joint,
        )

    def _phys_deg(self, leg: str, joint: str, raw: int) -> float:
        return math.degrees(self._phys_sign(leg, joint) * (raw - NEUTRAL_COUNT) / COUNTS_PER_RAD)

    def follow_tick(self) -> dict:
        """One follow cycle. The master's free joints are read and the same joint
        on the other three legs is driven in the SAME PHYSICAL direction (all hips
        outward together, etc.) — NOT the same canonical IK angle, which for the hip
        would send opposite-side legs inward. Each follower's raw deviation from
        center = master's physical deviation, re-signed by that leg's axis. One
        sync-write per bus; ~8-10 Hz."""
        sess = self._follow
        if not sess:
            raise RuntimeError("no active follow session")
        if self.driver is None:
            raise RuntimeError("follow requires a connected driver")
        master, free, captured = sess["master"], sess["free"], sess["captured"]
        plan: dict[Bus, list[tuple[int, int, int, int]]] = {Bus.A: [], Bus.B: []}
        joints_out: dict[str, dict] = {}
        for j in free:
            m_id = getattr(self.config.legs[master].ids, j)
            raw = self.driver.read_present_position_raw(self._bus_for(m_id), m_id)
            if raw is None:
                continue  # dropped reply this tick; skip, try again next tick
            phys = self._phys_sign(master, j) * (raw - NEUTRAL_COUNT)  # physical deviation, counts
            for leg in LEGS:
                if leg == master:
                    continue
                t_id = getattr(self.config.legs[leg].ids, j)
                cmd = max(0, min(4095, NEUTRAL_COUNT + self._phys_sign(leg, j) * phys))
                plan[self._bus_for(t_id)].append((t_id, cmd, FOLLOW_SPEED, FOLLOW_ACC))
            cap = captured.get(j, {})
            joints_out[j] = {
                "master_id": m_id,
                "angle_deg": round(self._phys_deg(master, j, raw), 1),
                "min_deg": round(self._phys_deg(master, j, cap["min"]), 1) if "min" in cap else None,
                "max_deg": round(self._phys_deg(master, j, cap["max"]), 1) if "max" in cap else None,
                "verified": bool(cap.get("applied")),
            }
        self.driver.sync_write_positions(plan)
        return {"master": master, "free": free, "joints": joints_out}

    def follow_set_limit(self, joint: str, which: str) -> dict:
        """Capture the master's CURRENT position as the *min* or *max* end for
        *joint* (or ``"all"`` free joints) and apply the matching PHYSICAL extreme
        to all four legs — each leg gets the same physical travel, re-signed through
        its own axis, so "master at outward max" sets every leg's outward max.
        Following continues; a joint's range is finalized (marked verified) once both
        ends are set with at least ``MIN_SWEEP_DEG`` between them."""
        sess = self._follow
        if not sess:
            raise RuntimeError("no active follow session")
        if self.driver is None:
            raise RuntimeError("follow requires a connected driver")
        if which not in ("min", "max"):
            raise ValueError("which must be 'min' or 'max'")
        master, free, captured = sess["master"], sess["free"], sess["captured"]
        targets = list(free) if joint == "all" else [joint]
        readings: list[tuple[str, int]] = []
        for j in targets:
            if j not in free:
                raise ValueError(f"{j} is not one of the freed joints {free}")
            m_id = getattr(self.config.legs[master].ids, j)
            raw = self.driver.read_present_position_raw(self._bus_for(m_id), m_id)
            if raw is None:
                raise RuntimeError(f"could not read master {master}_{j}; hold still and retry")
            readings.append((j, raw))

        result: dict[str, dict] = {}
        for j, m_raw in readings:
            cap = captured.setdefault(j, {})
            cap[which] = m_raw  # store the master's raw at this end
            psm = self._phys_sign(master, j)
            applied = False
            sweep_deg = None
            if "min" in cap and "max" in cap:
                e0 = psm * (cap["min"] - NEUTRAL_COUNT)  # physical extents, counts (+ = out/fwd/flex)
                e1 = psm * (cap["max"] - NEUTRAL_COUNT)
                lo, hi = sorted((e0, e1))
                sweep_deg = math.degrees((hi - lo) / COUNTS_PER_RAD)
                if sweep_deg >= MIN_SWEEP_DEG:
                    for leg in LEGS:
                        sid = getattr(self.config.legs[leg].ids, j)
                        psf = self._phys_sign(leg, j)
                        r0 = NEUTRAL_COUNT + psf * e0
                        r1 = NEUTRAL_COUNT + psf * e1
                        mn, mx = sorted((int(round(r0)), int(round(r1))))
                        mn = max(0, min(NEUTRAL_COUNT, mn))  # keep home (2048) inside the range
                        mx = min(4095, max(NEUTRAL_COUNT, mx))
                        self.set_range(sid, mn, NEUTRAL_COUNT, mx, write_eeprom=False)
                    applied = True
            cap["applied"] = applied
            result[j] = {
                "which": which,
                "angle_deg": round(self._phys_deg(master, j, m_raw), 1),
                "min_deg": round(self._phys_deg(master, j, cap["min"]), 1) if "min" in cap else None,
                "max_deg": round(self._phys_deg(master, j, cap["max"]), 1) if "max" in cap else None,
                "sweep_deg": round(sweep_deg, 1) if sweep_deg is not None else None,
                "applied": applied,
            }
        return {"set": result}

    def follow_check(self) -> dict:
        """Read every leg's present position for each free joint and compare their
        PHYSICAL angle (outward/forward/flex convention) to the master's. All legs
        at the same physical angle => they moved together correctly. A leg that
        DISAGREES is stalled against a stop or mis-signed — flag it for a look."""
        sess = self._follow
        if not sess:
            raise RuntimeError("no active follow session")
        if self.driver is None:
            raise RuntimeError("follow requires a connected driver")
        master, free = sess["master"], sess["free"]
        out: dict[str, dict] = {}
        all_agree = True
        for j in free:
            m_id = getattr(self.config.legs[master].ids, j)
            m_raw = self.driver.read_present_position_raw(self._bus_for(m_id), m_id)
            m_ang = self._phys_deg(master, j, m_raw) if m_raw is not None else None
            rows = []
            for leg in LEGS:
                if leg == master:
                    continue
                sid = getattr(self.config.legs[leg].ids, j)
                raw = self.driver.read_present_position_raw(self._bus_for(sid), sid)
                ang = self._phys_deg(leg, j, raw) if raw is not None else None
                agrees = (m_ang is not None and ang is not None and abs(ang - m_ang) <= AGREE_TOL_DEG)
                all_agree = all_agree and agrees
                rows.append({"leg": leg, "id": sid, "angle_deg": round(ang, 1) if ang is not None else None, "agrees": agrees})
            out[j] = {"master_angle_deg": round(m_ang, 1) if m_ang is not None else None, "followers": rows}
        return {"joints": out, "tolerance_deg": AGREE_TOL_DEG, "all_agree": all_agree}

    def follow_stop(self) -> dict:
        """End the session and torque off everything (safe release on a stand).
        Ranges are applied live in :meth:`follow_set_limit`, so this just reports
        which free joints ended up fully captured (both ends -> range verified)."""
        sess = self._follow
        if not sess:
            raise RuntimeError("no active follow session")
        master, free, captured = sess["master"], sess["free"], sess["captured"]
        verified, pending = [], []
        for j in free:
            (verified if captured.get(j, {}).get("applied") else pending).append(j)
        if self.driver is not None:
            for leg in LEGS:
                for joint in JOINTS:
                    sid = getattr(self.config.legs[leg].ids, joint)
                    if sid is not None:
                        self.driver.enable_torque(self._bus_for(sid), sid, False)
        self._follow = None
        return {"verified": verified, "pending": pending}

    # ---- Stage 7: VISUALIZE (1 Hz poll) -------------------------------
    def poll_pose(self) -> dict[str, dict]:
        """Poll each assigned servo's present position for the 3D skeleton."""
        # During a live follow session the fast tick loop owns the serial bus;
        # skip the heavy 12-servo pose read so it doesn't stutter the following.
        if self._follow is not None:
            return {}
        out: dict[str, dict] = {}
        for leg in LEGS:
            for joint in JOINTS:
                sid = getattr(self.config.legs[leg].ids, joint)
                if sid is None:
                    continue
                spec = self.config.servos[str(sid)]
                raw = None
                if self.driver is not None:
                    bus = _bus_enum(self._slot_bus(spec.slot))
                    raw = self.driver.read_present_position_raw(bus, sid)
                if raw is None:
                    raw = spec.home_raw
                deg = self._count_to_deg(raw, spec)
                out[f"{leg}_{joint}"] = {
                    "id": sid,
                    "raw": raw,
                    "deg": round(deg, 2),
                    "frame_value": round(deg, 2),
                }
        return out

    # ---- Stage 8: TUNE ------------------------------------------------
    def update_gait_seed(self, **levers) -> None:
        data = self.config.gait_seed.model_dump()
        data.update({k: v for k, v in levers.items() if k in data})
        self.config.gait_seed = type(self.config.gait_seed).model_validate(data)

    # ---- helpers ------------------------------------------------------
    def _slot_bus(self, slot: str) -> str:
        leg = slot.split("_")[0]
        return self.config.legs[leg].bus

    def status(self) -> dict:
        """Per-slot verified-bit summary for the UI."""
        rows = {}
        for leg in LEGS:
            for joint in JOINTS:
                slot = f"{leg}_{joint}"
                sid = getattr(self.config.legs[leg].ids, joint)
                spec = self.config.servos.get(str(sid)) if sid is not None else None
                v: VerifiedBits = spec.verified if spec else VerifiedBits()
                rows[slot] = {
                    "id": sid,
                    "assigned": v.assigned,
                    "center": v.center,
                    "direction": v.direction,
                    "range": v.range,
                }
        return rows

    def ready_to_emit(self) -> tuple[bool, list[str]]:
        warnings: list[str] = []
        if not self.config.all_slots_assigned():
            warnings.append("not all 12 slots assigned")
        for slot, row in self.status().items():
            for bit in ("assigned", "center", "direction", "range"):
                if not row[bit]:
                    warnings.append(f"{slot}: {bit} not verified")
        return (len(warnings) == 0, warnings)
