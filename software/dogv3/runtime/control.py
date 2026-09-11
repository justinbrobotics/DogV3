"""Runtime control: gait -> IK -> servo counts -> SyncWritePosEx, with the
ARM gate, soft-start ramp and command-stream watchdog.

The hot path computes one :meth:`RobotController.compute_plan` per tick and
hands a per-bus ``{bus: [(id, count, speed, acc), ...]}`` plan to the driver's
``sync_write_positions`` — never per-servo blocking writes per cycle.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from ..config.schema import RobotConfig
from ..driver.feetech import FeetechDriver
from ..driver.mux import Bus
from ..gait.trot import (
    DEFAULT_MANUAL_PITCH_DEG,
    DEFAULT_MANUAL_SWAY_MM,
    GaitCommand,
    MANUAL_PITCH_MAX_DEG,
    MANUAL_SWAY_MAX_MM,
    TrotGait,
)
from ..kinematics.leg import LegGeometry, angle_to_count, inverse_kinematics, joint_positions

JOINT_FOR_ANGLE = ("hip", "femur", "tibia")  # t1, t2, t3 order

# Nominal hip locations on the body (X=lateral, Y=forward signs), used to place
# each leg's skeleton in the body frame for the live pose view.
LEG_HIP_SIGN = {"FL": (-1.0, 1.0), "FR": (1.0, 1.0), "BL": (-1.0, -1.0), "BR": (1.0, -1.0)}

INTENT_STALE_HOLD_TIMEOUT = 0.3   # seconds without a command -> hold stand, stay armed
INTENT_STALE_DISARM_TIMEOUT = 10.0  # long stale intent -> disarm and drop torque
WATCHDOG_TIMEOUT = INTENT_STALE_HOLD_TIMEOUT  # backward-compatible name for tests/tools
SOFT_START_TIME = 1.2    # ramp from stand to full gait amplitude
MANUAL_SWAY_SLEW_MM_S = 25.0  # fixed physical rate; 20 mm takes 0.8 s
MANUAL_PITCH_SLEW_DEG_S = 8.0  # 12-degree trial ceiling takes 1.5 s
# Servo goal speed/acc in the hot loop. 3400 counts/s = ~299 deg/s — just under
# the STS3215 no-load 375 deg/s, and above its ~250 deg/s loaded ceiling, so the
# servo's own physics (not this cap) limits the gait. The old value of 1500
# (131.8 deg/s) silently clipped every useful trot's swing: the conservative /
# balanced / aggressive sets need >=1727 / 2388 / 3276 counts/s respectively.
# Smoothness is owned by the software soft-start blend, not this profile; keep
# ACC high so each 60 Hz streamed target completes within its tick (a slow
# internal trapezoid that never finishes adds lag and heat).
DEFAULT_SPEED = 3400     # counts/s (~299 deg/s) -- already the STS3215's no-load
                         # ceiling (~50 RPM at 12V), so raising it is a no-op.
# ACC is a SINGLE byte on the wire (``acc & 0xFF`` in protocol._pos_payload), so
# 254 is the maximum the servo can be told. 150 gave 1318 deg/s^2, and the swing
# demands 167 deg/s at the live gait -- 127 ms to reach it, out of a 280 ms swing.
# That is 45% of every swing spent accelerating and as much again decelerating,
# so the foot never reaches the commanded trajectory and lands still catching up.
# 254 gives 2232 deg/s^2 and cuts that to 75 ms.
#
# This also brings the value in line with the intent stated above it: a target
# streamed at 60 Hz should complete inside its own 16.7 ms tick. 150 missed that
# by 7.6x. 254 still does not meet it, which is why per-tick matched speed --
# rather than a fixed profile -- is the real fix.
#
# COST: higher acceleration means higher current spikes, more heat and more
# battery sag. Watch servo temperature, and do not run this on a suspect pack.
# If achieved velocity does not rise on the leg bench when ACC rises, the servo
# is already saturating under load and this changes nothing but the current draw.
DEFAULT_ACC = 254        # x100 counts/s^2 (~2232 deg/s^2), the register maximum
CONTROL_HZ_DEFAULT = 60.0  # default control-loop rate (50-100 Hz range)
DEADZONE_DEFAULT = 0.08    # joystick stick deadzone
# How often to re-attempt a torque-on that did not fully land. A rail that
# comes up AFTER the arm edge would otherwise leave the robot armed and limp
# forever, because torque is edge-triggered and there is no second edge.
TORQUE_RETRY_S = 1.0


@dataclass
class ControllerState:
    armed: bool = False
    estop: bool = False
    soft_start: float = 0.0          # 0..1 amplitude ramp
    last_cmd_time: float = 0.0
    last_seq: int = -1
    watchdog_tripped: bool = False


class RobotController:
    def __init__(self, config: RobotConfig, driver: FeetechDriver | None, *, now=time.monotonic):
        self.config = config
        self.driver = driver
        self._now = now
        self.state = ControllerState(last_cmd_time=now())
        # Stale-intent thresholds; the module constants are the hardware
        # defaults. The physics-sim path relaxes them (no safety stakes, and
        # backgrounded browser tabs throttle the intent heartbeat to ~1 Hz).
        self.stale_hold_timeout = INTENT_STALE_HOLD_TIMEOUT
        self.stale_disarm_timeout = INTENT_STALE_DISARM_TIMEOUT
        self.cmd = GaitCommand()          # shaped command the gait actually sees
        self.cmd_target = GaitCommand()   # raw teleop intent (slew target)
        self.gait = TrotGait(config.gait_seed, config)
        self.last_targets = self.gait.stand_targets(self.cmd)
        # Held custom pose: {leg: (x, y, z)} foot targets, or None for gait.
        # Set via hold_pose(); any nonzero drive intent clears it back to gait.
        self.pose_hold: dict[str, tuple[float, float, float]] | None = None
        self.pose_name: str | None = None
        # Running trick (pose sequence) or None; see start_trick().
        self.trick: dict | None = None
        # Level-referenced (roll, pitch) in degrees, or None when no IMU is
        # attached. Set by whoever owns the sensor — the controller never
        # imports it, so the hot path stays hardware-free and testable.
        self.attitude: tuple[float, float] | None = None
        # Torque bookkeeping. A write that silently fails leaves the robot
        # ARMED and streaming gait into servos that were never told to hold.
        self.torque_wanted = False
        self.torque_acked = 0
        self.torque_total = 0
        self._torque_retry_t = 0.0
        # Raw servo positions captured immediately before torque-on.  The
        # unarmed loop deliberately keeps streaming the neutral stand goal,
        # so enabling torque without first replacing that stored goal can make
        # a collapsed robot demand tens of degrees from every loaded joint at
        # once.  Keep the measured counts and blend from them to the IK plan
        # during the existing soft-start window.
        self.link_resynced = False
        self.arm_capture_acked = 0
        self.arm_capture_total = 0
        self._arm_start_counts: dict[tuple[Bus, int], int] | None = None
        self._trim: dict[str, float] = {leg: 0.0 for leg in ("FL", "FR", "BL", "BR")}
        self._geom = {
            leg: LegGeometry(
                L1=config.links_mm.L1_hip,
                L2=config.links_mm.L2_femur,
                L3=config.links_mm.L3_tibia,
                side=config.legs[leg].side,
                knee_sign=config.legs[leg].knee_sign,
            )
            for leg in ("FL", "FR", "BL", "BR")
        }

    # -- command intake --------------------------------------------------
    def submit_command(self, cmd: dict) -> None:
        """Last-wins update from a teleop command dict. E-STOP always wins."""
        seq = cmd.get("seq", self.state.last_seq + 1)
        if seq < self.state.last_seq:
            return  # stale/out-of-order
        self.state.last_seq = seq
        self.state.last_cmd_time = self._now()
        self.state.watchdog_tripped = False
        if cmd.get("estop"):
            self.estop()
            return
        # E-STOP always wins: a latched E-STOP cannot be overridden by an arm
        # command. The operator must explicitly clear_estop() first.
        if self.state.estop:
            self.state.armed = False
        else:
            self.state.armed = bool(cmd.get("arm", self.state.armed))
        self.cmd_target = GaitCommand(
            vx=float(cmd.get("vx", 0.0)),
            vy=float(cmd.get("vy", 0.0)),
            wz=float(cmd.get("wz", 0.0)),
            body_z=float(cmd.get("body_z", 0.0)),
            sway=max(-1.0, min(1.0, float(cmd.get("sway", 0.0)))),
            sway_max_mm=max(0.0, min(
                MANUAL_SWAY_MAX_MM,
                float(cmd.get("sway_max_mm", DEFAULT_MANUAL_SWAY_MM)))),
            pitch=max(-1.0, min(1.0, float(cmd.get("pitch", 0.0)))),
            pitch_max_deg=max(0.0, min(
                MANUAL_PITCH_MAX_DEG,
                float(cmd.get("pitch_max_deg", DEFAULT_MANUAL_PITCH_DEG)))),
        )

    # -- custom poses ------------------------------------------------------
    def hold_pose(self, feet: dict, name: str | None = None) -> None:
        """Hold a named pose: {leg: [x, y, z]} foot targets. Blended in from the
        current soft-start state; cleared automatically by drive intent."""
        self.pose_hold = {leg: (float(p[0]), float(p[1]), float(p[2])) for leg, p in feet.items()}
        self.pose_name = name
        self.state.soft_start = 0.0  # re-ramp into the pose gently

    def clear_pose(self) -> None:
        if self.pose_hold is not None:
            self.pose_hold = None
            self.pose_name = None
            self.state.soft_start = 0.0  # re-ramp back into stand/gait

    def capture_pose(self) -> dict[str, list[float]]:
        """The current commanded foot targets as a pose dict (for saving)."""
        return {leg: [round(t.x, 1), round(t.y, 1), round(t.z, 1)] for leg, t in self.last_targets.items()}

    def _drive_active(self) -> bool:
        # Raw intent, not the shaped command: a pose/trick cancels the instant
        # the operator asks to drive, not after the slew ramps up.
        t = self.cmd_target
        return abs(t.vx) > 0.02 or abs(t.vy) > 0.02 or abs(t.wz) > 0.02

    def _shape_cmd(self, dt: float) -> None:
        """Slew drive and two-axis stand lean without letting them overlap.

        Any drive request first recentres a lean, and a bumper/trigger request
        waits until shaped drive has stopped. That makes both axes planted-stand
        demonstrations instead of unmodelled walking balance inputs.
        """
        raw_drive_active = self._drive_active()
        shaped_drive_active = any(
            abs(getattr(self.cmd, f)) > 0.02 for f in ("vx", "vy", "wz"))
        selected_max = max(0.0, min(MANUAL_SWAY_MAX_MM, self.cmd_target.sway_max_mm))
        selected_pitch = max(
            0.0, min(MANUAL_PITCH_MAX_DEG, self.cmd_target.pitch_max_deg))
        lean_blocked = raw_drive_active or shaped_drive_active
        sway_input = 0.0 if lean_blocked else self.cmd_target.sway
        pitch_input = 0.0 if lean_blocked else self.cmd_target.pitch
        # Full commands on both pairs are projected onto one unit circle, so a
        # corner lean cannot stack 100% lateral and 100% pitch simultaneously.
        combined = math.hypot(sway_input, pitch_input)
        if combined > 1.0:
            sway_input /= combined
            pitch_input /= combined
        sway_target_mm = sway_input * selected_max
        pitch_target_deg = pitch_input * selected_pitch
        current_sway_mm = float(self.cmd.sway_mm or 0.0)
        current_pitch_deg = float(self.cmd.pitch_deg or 0.0)
        sway_step_mm = MANUAL_SWAY_SLEW_MM_S * dt
        current_sway_mm += max(
            -sway_step_mm, min(sway_step_mm, sway_target_mm - current_sway_mm))
        pitch_step_deg = MANUAL_PITCH_SLEW_DEG_S * dt
        current_pitch_deg += max(
            -pitch_step_deg, min(pitch_step_deg, pitch_target_deg - current_pitch_deg))
        self.cmd.sway_mm = current_sway_mm
        self.cmd.sway_max_mm = selected_max
        self.cmd.sway = (max(-1.0, min(1.0, current_sway_mm / selected_max))
                         if selected_max > 0.0 else 0.0)
        self.cmd.pitch_deg = current_pitch_deg
        self.cmd.pitch_max_deg = selected_pitch
        self.cmd.pitch = (max(-1.0, min(1.0, current_pitch_deg / selected_pitch))
                          if selected_pitch > 0.0 else 0.0)

        # Hold locomotion at zero until an existing lean has returned to centre.
        drive_blocked = raw_drive_active and (
            abs(current_sway_mm) > 0.5 or abs(current_pitch_deg) > 0.2)
        lim = self.gait.seed.accel_limit * dt
        for f in ("vx", "vy", "wz"):
            cur = getattr(self.cmd, f)
            tgt = 0.0 if drive_blocked else getattr(self.cmd_target, f)
            setattr(self.cmd, f, cur + max(-lim, min(lim, tgt - cur)))
        self.cmd.body_z = self.cmd_target.body_z  # height trim stays immediate

    # -- tricks: pose sequences via the stand "home" hub -------------------
    def start_trick(self, name: str, steps: list[tuple[dict, float, float]]) -> None:
        """Run a pose sequence. ``steps`` = [(feet_dict, move_s, hold_s), ...].

        Safe entry rule: the executor always blends to STAND first, then
        through the steps, then back to stand — so a trick is valid from any
        starting posture without path planning. Drive intent or E-STOP cancels
        immediately (falls back to gait/stand with a fresh soft-start ramp)."""
        stand = {leg: (t.x, t.y, t.z) for leg, t in self.gait.stand_targets(self.cmd).items()}
        norm = [({lg: (float(p[0]), float(p[1]), float(p[2])) for lg, p in feet.items()}, m, h)
                for feet, m, h in steps]
        src = {leg: (t.x, t.y, t.z) for leg, t in self.last_targets.items()}
        self.trick = {
            "name": name,
            "wp": [(stand, 0.8, 0.2)] + norm + [(stand, 0.8, 0.0)],
            "i": 0, "t": 0.0, "src": src,
        }
        self.clear_pose()

    def stop_trick(self) -> None:
        if self.trick is not None:
            self.trick = None
            self.state.soft_start = 0.0  # re-ramp back into stand/gait

    def _trick_targets(self, dt: float):
        """Advance the trick timeline and return blended FootTargets. Uses a
        smoothstep ease between waypoints; finishes by clearing ``trick``."""
        from ..gait.trot import FootTarget

        tk = self.trick
        feet, move_s, hold_s = tk["wp"][tk["i"]]
        tk["t"] += dt
        if tk["t"] >= move_s + hold_s:
            # Waypoint done: it becomes the source for the next blend.
            tk["src"] = feet
            tk["t"] = 0.0
            tk["i"] += 1
            if tk["i"] >= len(tk["wp"]):
                self.trick = None
                self.state.soft_start = 0.0
                return self.gait.stand_targets(self.cmd)
            feet, move_s, hold_s = tk["wp"][tk["i"]]
        u = min(1.0, tk["t"] / move_s)
        u = u * u * (3.0 - 2.0 * u)  # smoothstep: zero-velocity endpoints
        src = tk["src"]
        return {
            leg: FootTarget(
                leg=leg,
                x=src[leg][0] + (feet[leg][0] - src[leg][0]) * u,
                y=src[leg][1] + (feet[leg][1] - src[leg][1]) * u,
                z=src[leg][2] + (feet[leg][2] - src[leg][2]) * u,
                phase=0.0, in_stance=True,
            )
            for leg in src
        }

    def trick_status(self) -> str | None:
        if self.trick is None:
            return None
        return f"{self.trick['name']} ({min(self.trick['i'] + 1, len(self.trick['wp']))}/{len(self.trick['wp'])})"

    def _pose_targets(self):
        from ..gait.trot import FootTarget

        return {
            leg: FootTarget(leg=leg, x=x, y=y, z=z, phase=0.0, in_stance=True)
            for leg, (x, y, z) in (self.pose_hold or {}).items()
        }

    def arm(self) -> None:
        if not self.state.estop:
            self.state.armed = True

    def disarm(self) -> None:
        self.state.armed = False
        self.state.soft_start = 0.0
        self._arm_start_counts = None

    def estop(self) -> None:
        self.state.estop = True
        self.trick = None  # never resume a trick across an E-STOP
        self.cmd = GaitCommand()         # E-STOP does not glide
        self.cmd_target = GaitCommand()
        self.disarm()

    def clear_estop(self) -> None:
        self.state.estop = False

    # -- watchdog --------------------------------------------------------
    def check_watchdog(self) -> bool:
        """Return True if intent is stale enough to hold stand."""
        if self._now() - self.state.last_cmd_time > self.stale_hold_timeout:
            if self.state.armed:
                self.state.watchdog_tripped = True
            return True
        return False

    def check_long_stale(self) -> bool:
        """Return True if intent has been stale long enough to disarm."""
        return self._now() - self.state.last_cmd_time > self.stale_disarm_timeout

    def _retry_torque_if_short(self) -> None:
        """Re-attempt a torque-on that did not fully land.

        Torque is edge-triggered on arm, so a write that went into an
        unpowered or sagging bus has no second edge to correct it: the robot
        stays armed, streams gait, and never holds. Retrying while armed lets
        a rail that comes good late self-heal instead of needing a manual
        disarm/re-arm."""
        if self.driver is None or not (self.state.armed and self.torque_wanted):
            return
        if self.torque_total and self.torque_acked >= self.torque_total:
            return
        if self._now() - self._torque_retry_t < TORQUE_RETRY_S:
            return
        self.apply_torque(True)

    # -- hot loop --------------------------------------------------------
    def tick(self, dt: float) -> dict[Bus, list[tuple[int, int, int, int]]]:
        """Advance one control step; returns the per-bus sync-write plan and (as
        a side effect) sends it if a driver is attached."""
        stale = self.check_watchdog()
        long_stale = self.check_long_stale()
        if stale:
            # A lost browser/gamepad link must return both lean axes to centre,
            # not leave the body indefinitely shifted or pitched.
            self.cmd_target.sway = 0.0
            self.cmd_target.pitch = 0.0
        self._shape_cmd(dt)
        self._retry_torque_if_short()

        if long_stale and self.state.armed:
            self.disarm()
            self.apply_torque(False)
            targets = self.gait.stand_targets(self.cmd)
        elif self.state.estop or not self.state.armed:
            # Hold a safe stand pose, torque follows arm state elsewhere.
            self.state.soft_start = 0.0
            targets = self.gait.stand_targets(self.cmd)
        elif stale:
            # Short stale intent holds a safe stand and stays armed.
            self.state.soft_start = 0.0
            targets = self.gait.stand_targets(self.cmd)
        elif self.trick is not None and not self._drive_active():
            # Trick executor owns the timeline; its blends are already smooth.
            targets = self._trick_targets(dt)
        elif self.pose_hold is not None and not self._drive_active():
            # Hold the custom pose, soft-blended in from stand.
            self.state.soft_start = min(1.0, self.state.soft_start + dt / SOFT_START_TIME)
            targets = self._blend(self._pose_targets(), dt)
        else:
            if self.trick is not None:
                self.stop_trick()  # drive intent cancels a running trick
            if self.pose_hold is not None:
                self.clear_pose()  # drive intent overrides a held pose
            self.state.soft_start = min(1.0, self.state.soft_start + dt / SOFT_START_TIME)
            targets = self._blend(self.gait.step(dt, self.cmd), dt)

        targets = self._apply_stabilizer(targets, dt)
        self.last_targets = targets
        plan = self.compute_plan(targets)
        plan = self._apply_arm_start_ramp(plan)
        if self.driver is not None:
            self.driver.sync_write_positions(plan)
        return plan

    def _apply_arm_start_ramp(
        self, plan: dict[Bus, list[tuple[int, int, int, int]]]
    ) -> dict[Bus, list[tuple[int, int, int, int]]]:
        """Blend raw goals from the measured pre-arm pose to the IK plan.

        A Cartesian blend cannot safely represent an arbitrary collapsed pose;
        its feet may not even lie on a shared valid body plane.  Raw-count
        interpolation starts every joint at exactly the position where torque
        was engaged and converges to the normal, limit-checked plan over
        ``SOFT_START_TIME``.
        """
        starts = self._arm_start_counts
        if not starts:
            return plan
        s = max(0.0, min(1.0, self.state.soft_start))
        if s >= 1.0:
            self._arm_start_counts = None
            return plan
        blended: dict[Bus, list[tuple[int, int, int, int]]] = {Bus.A: [], Bus.B: []}
        for bus, entries in plan.items():
            for sid, goal, speed, acc in entries:
                start = starts.get((bus, sid), goal)
                count = round(start + s * (goal - start))
                blended[bus].append((sid, count, speed, acc))
        return blended

    def _apply_stabilizer(self, targets, dt):
        """Trim STANCE leg length toward level using measured body attitude.

        A body roll of ``r`` lifts the hip at lateral arm ``a`` by ``a*sin(r)``;
        shortening that leg by the same amount puts the body back level. Same
        argument in pitch with the fore/aft arm. Only stance legs are trimmed —
        lengthening a swing leg does nothing useful and risks a foot strike —
        and every leg slews toward its target (0 for swing legs) so a trim
        decays smoothly instead of snapping off at liftoff.

        SIGN CHECK, on the stand, before enabling: tilt the body left by hand.
        The LEFT legs should try to shorten and the right to lengthen. If they
        do the opposite, set ``invert_roll``; the same test nose-down for
        ``invert_pitch``. Getting this wrong is positive feedback.
        """
        st = getattr(self.config, "stabilizer", None)
        if st is None or not st.enabled:
            return targets
        roll = pitch = 0.0
        if self.attitude is not None:
            r, p = self.attitude
            roll = 0.0 if abs(r) < st.deadband_deg else r
            pitch = 0.0 if abs(p) < st.deadband_deg else p
        rr = math.radians(roll) * (-1.0 if st.invert_roll else 1.0)
        pr = math.radians(pitch) * (-1.0 if st.invert_pitch else 1.0)
        half_w = self.config.body_mm.shoulder_lateral / 2.0
        half_l = self.config.body_mm.fore_aft / 2.0
        step = st.slew_mm_s * dt
        for leg, ft in targets.items():
            sx, sy = LEG_HIP_SIGN[leg]
            if ft.in_stance:
                want = -st.gain * (rr * sx * half_w + pr * sy * half_l)
                want = max(-st.max_trim_mm, min(st.max_trim_mm, want))
            else:
                want = 0.0          # let a swing leg's trim decay to nothing
            cur = self._trim.get(leg, 0.0)
            cur += max(-step, min(step, want - cur))
            self._trim[leg] = cur
            ft.z += cur
        return targets

    def _blend(self, targets, dt):
        """Apply the soft-start amplitude ramp by blending gait targets toward
        the neutral stand pose."""
        s = self.state.soft_start
        if s >= 1.0:
            return targets
        stand = self.gait.stand_targets(self.cmd)
        blended = {}
        for leg, t in targets.items():
            st = stand[leg]
            t.x = st.x + s * (t.x - st.x)
            t.y = st.y + s * (t.y - st.y)
            t.z = st.z + s * (t.z - st.z)
            blended[leg] = t
        return blended

    def compute_plan(self, targets) -> dict[Bus, list[tuple[int, int, int, int]]]:
        """Foot targets -> IK -> per-servo counts grouped by bus."""
        plan: dict[Bus, list[tuple[int, int, int, int]]] = {Bus.A: [], Bus.B: []}
        for leg, ft in targets.items():
            geom = self._geom[leg]
            angles = inverse_kinematics(geom, ft.x, ft.y, ft.z)
            bus = Bus.from_name(self.config.legs[leg].bus)
            for joint, theta in zip(JOINT_FOR_ANGLE, angles):
                sid = self.config.id_for(leg, joint)
                if sid is None:
                    continue
                spec = self.config.servos.get(str(sid))
                if spec is None:
                    continue
                count = angle_to_count(
                    theta,
                    invert=spec.invert,
                    offset=0.0,
                    min_raw=spec.min_raw,
                    max_raw=spec.max_raw,
                )
                plan[bus].append((sid, count, DEFAULT_SPEED, DEFAULT_ACC))
        return plan

    # -- torque management ----------------------------------------------
    def prepare_arm(self) -> tuple[int, int]:
        """Capture every present position and install it as the torque-on goal.

        Returns ``(captured, total)``.  Hardware callers must refuse ARM unless
        all configured servos were read.  Dry/sim controllers have no driver,
        so ``(0, 0)`` is their successful no-hardware result.
        """
        self.arm_capture_acked = self.arm_capture_total = 0
        self._arm_start_counts = None
        self.state.soft_start = 0.0
        if self.driver is None:
            return 0, 0

        # A failed earlier transaction can leave both kernel bytes and a
        # partial mux frame behind.  ARM is a safety boundary: begin its reads
        # from a known-clean receive state.
        # Host-side clearing is not enough on its own: the ESP32's own rxState
        # is a static global, and a truncated or mis-aligned write strands it
        # in ASCII_LINE, after which it answers every mux frame with
        # "ERR unknown cmd" and never recovers. resync() walks it back to IDLE
        # and confirms with VERSION. Observed on the robot as arm_capture 0/12
        # with driver_io mux_frames 0 and text_head b'ERR unknown cmd: ...'.
        self.driver.reset_input()
        self.link_resynced = self.driver.resync()

        starts: dict[tuple[Bus, int], int] = {}
        plan: dict[Bus, list[tuple[int, int, int, int]]] = {Bus.A: [], Bus.B: []}
        total = 0
        for leg in ("FL", "FR", "BL", "BR"):
            bus = Bus.from_name(self.config.legs[leg].bus)
            for joint in JOINT_FOR_ANGLE:
                sid = self.config.id_for(leg, joint)
                if sid is None:
                    continue
                total += 1
                position = self.driver.read_position(bus, sid)
                if position is None:
                    continue
                starts[(bus, sid)] = position
                plan[bus].append((sid, position, DEFAULT_SPEED, DEFAULT_ACC))

        captured = len(starts)
        self.arm_capture_acked, self.arm_capture_total = captured, total
        if total == 0 or captured != total:
            return captured, total

        # Replace the stand goals that the torque-off loop has been streaming.
        # This broadcast has no reply; the preceding reads prove each servo and
        # bus are alive, and the following torque writes remain acknowledged.
        self.driver.sync_write_positions(plan)
        self._arm_start_counts = starts
        return captured, total

    def apply_torque(self, enable: bool) -> tuple[int, int]:
        """Enable/disable torque on every assigned servo; returns (acked, total).

        The return is the point. ``enable_torque`` reports whether the servo
        acknowledged, and discarding it hides the worst failure this system
        has: writes into an unpowered or sagging bus leave the robot ARMED,
        streaming gait, and completely limp, with nothing in telemetry saying
        so. The count is published as ``telemetry()['torque']`` and short
        counts are retried by :meth:`tick` while armed.
        """
        self.torque_wanted = enable
        if self.driver is None:
            self.torque_acked = self.torque_total = 0
            return 0, 0
        self.driver.reset_input()
        acked = total = 0
        for leg in ("FL", "FR", "BL", "BR"):
            bus = Bus.from_name(self.config.legs[leg].bus)
            for joint in JOINT_FOR_ANGLE:
                sid = self.config.id_for(leg, joint)
                if sid is None:
                    continue
                total += 1
                if self.driver.enable_torque(bus, sid, enable):
                    acked += 1
        self.torque_acked, self.torque_total = acked, total
        self._torque_retry_t = self._now()
        return acked, total

    # -- telemetry -------------------------------------------------------
    def telemetry(self) -> dict:
        return {
            "armed": self.state.armed,
            "estop": self.state.estop,
            "soft_start": round(self.state.soft_start, 3),
            "watchdog_tripped": self.state.watchdog_tripped,
            "torque": {"acked": self.torque_acked, "total": self.torque_total,
                       "wanted": self.torque_wanted},
            "link_resynced": self.link_resynced,
            "arm_capture": {"acked": self.arm_capture_acked,
                            "total": self.arm_capture_total},
            "driver_io": (self.driver.diagnostics()
                          if self.driver is not None and hasattr(self.driver, "diagnostics")
                          else None),
            "cmd": {"vx": self.cmd.vx, "vy": self.cmd.vy, "wz": self.cmd.wz,
                    "body_z": self.cmd.body_z, "sway": self.cmd.sway,
                    "sway_mm": round(float(self.cmd.sway_mm or 0.0), 1),
                    "sway_max_mm": self.cmd.sway_max_mm,
                    "pitch": self.cmd.pitch,
                    "pitch_deg": round(float(self.cmd.pitch_deg or 0.0), 1),
                    "pitch_max_deg": self.cmd.pitch_max_deg},
            "phase": round(self.gait.clock.phase, 3),
            "pose": self.pose_name,
            "trick": self.trick_status(),
        }

    def pose_snapshot(self) -> dict:
        """Body-frame skeleton of the last computed pose, for the live viewer.

        For each leg: the four chain points (hip, femur joint, knee, foot) placed
        at the leg's hip location on the body, plus joint angles, target raw
        counts, and stance/swing state. Derived from ``last_targets`` so it always
        matches what the hot loop just commanded."""
        legs: dict[str, dict] = {}
        for leg in ("FL", "FR", "BL", "BR"):
            geom = self._geom[leg]
            ft = self.last_targets[leg]
            angles = inverse_kinematics(geom, ft.x, ft.y, ft.z)
            sx, sy = LEG_HIP_SIGN[leg]
            ox = sx * self.config.body_mm.shoulder_lateral / 2.0
            oy = sy * self.config.body_mm.fore_aft / 2.0
            points = [
                [round(ox + p[0], 2), round(oy + p[1], 2), round(p[2], 2)]
                for p in joint_positions(geom, *angles)
            ]
            counts: dict[str, int | None] = {}
            for joint, theta in zip(JOINT_FOR_ANGLE, angles):
                sid = self.config.id_for(leg, joint)
                spec = self.config.servos.get(str(sid)) if sid is not None else None
                counts[joint] = (
                    angle_to_count(theta, invert=spec.invert, min_raw=spec.min_raw, max_raw=spec.max_raw)
                    if spec is not None
                    else None
                )
            legs[leg] = {
                "in_stance": ft.in_stance,
                "phase": round(ft.phase, 3),
                "points": points,
                "angles_deg": {j: round(math.degrees(a), 1) for j, a in zip(JOINT_FOR_ANGLE, angles)},
                "counts": counts,
            }
        return {
            "legs": legs,
            "body": {
                "shoulder_lateral": self.config.body_mm.shoulder_lateral,
                "fore_aft": self.config.body_mm.fore_aft,
            },
        }
