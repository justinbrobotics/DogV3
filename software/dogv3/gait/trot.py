"""Trot gait — generates per-leg foot targets in each leg's local frame.

Stance: the planted foot is dragged opposite the commanded body velocity (so the
body advances). Swing: the foot lifts and returns to the leading position.
Steering (``wz``) adds a per-leg tangential component derived from where each hip
sits on the body. All distances are millimetres; angles radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..config.schema import GaitSeed, RobotConfig
from .phase import CRAWL_LIFT_ORDER, CRAWL_MIN_DUTY, PhaseClock, TROT_OFFSETS

# Crawl lateral-sway shaping. A raw sine is exactly zero at g=0 and g=0.5 — the
# instants BL and BR lift — so the body carries no lean at two of the four
# liftoffs, and only reaches full lean halfway through the other two swings.
# Squashing the sine toward a square wave holds the lean across each entire
# swing window. tanh(K*v)/tanh(K) keeps the peaks at exactly +/-1, so
# ``body_sway_amp`` still means "millimetres of lean at full sway". K=2.5 is the
# knee of the trade: flat enough to cover a swing, smooth enough not to jerk the
# body at the half-cycle reversal.
SWAY_SQUARE_K = 2.5
# Manual trigger body motion is an all-feet-planted demo, not a walking balance
# input. The wider limits are trial ceilings only; the GUI starts lower.
DEFAULT_MANUAL_SWAY_MM = 20.0
MANUAL_SWAY_MAX_MM = 70.0
DEFAULT_MANUAL_PITCH_DEG = 5.0
MANUAL_PITCH_MAX_DEG = 12.0


def _sway_shape(v: float) -> float:
    """Squash a [-1, 1] sinusoid toward a square wave, preserving its peaks."""
    return math.tanh(SWAY_SQUARE_K * v) / math.tanh(SWAY_SQUARE_K)


@dataclass
class GaitCommand:
    """Latest teleop intent: normalized drive/sway plus body-height trim (mm).

    ``vy`` positive = forward (+Y). ``sway`` positive shifts the body right
    (+X), and is only applied by the all-feet-planted stand target.
    """

    vx: float = 0.0   # lateral
    vy: float = 0.0   # forward
    wz: float = 0.0   # yaw rate
    body_z: float = 0.0  # body-height delta (mm), added to gait body_height
    sway: float = 0.0  # normalized manual stand sway, -1=left / +1=right
    sway_max_mm: float = DEFAULT_MANUAL_SWAY_MM  # operator-selected amplitude
    sway_mm: float | None = None  # shaped actual shift; None uses sway*sway_max_mm
    pitch: float = 0.0  # normalized manual stand pitch, -1=nose-down / +1=nose-up
    pitch_max_deg: float = DEFAULT_MANUAL_PITCH_DEG
    pitch_deg: float | None = None  # shaped actual pitch angle


@dataclass
class FootTarget:
    leg: str
    x: float
    y: float
    z: float
    phase: float
    in_stance: bool


# Nominal hip locations on the body (signs in the X=lateral / Y=forward frame).
_HIP_SIGN: dict[str, tuple[float, float]] = {
    "FL": (-1.0, +1.0),
    "FR": (+1.0, +1.0),
    "BL": (-1.0, -1.0),
    "BR": (+1.0, -1.0),
}


def _swing_height(w: float, shape: str, step_height: float) -> float:
    """Vertical lift profile over swing progress w in [0, 1]."""
    if shape == "sine":
        return step_height * math.sin(math.pi * w)
    if shape == "cycloid":
        return step_height * (1 - math.cos(2 * math.pi * w)) / 2
    if shape == "flick":
        # Asymmetric: snap the foot up early (peak ~w=0.37), reach down late —
        # reads as "picking the leg up" instead of hovering it.
        return step_height * math.sin(math.pi * w ** 0.7)
    # parabola (default): 0 at endpoints, peak at mid-swing.
    return step_height * (4 * w * (1 - w))


def _swing_frac(w: float, duty: float, retract: float) -> float:
    """Horizontal swing progress: -0.5 (trailing) -> +0.5 (leading).

    retract=0 is the legacy constant-velocity return. retract=1 is a cubic
    Hermite whose endpoint slopes equal the stance drag rate, so the foot
    leaves and lands with ~zero velocity relative to the GROUND — no liftoff
    scuff, no touchdown skid. It overshoots the landing point mid-swing and
    pulls back: the visible 'paw flick'."""
    lin = -0.5 + w
    if retract <= 0.0:
        return lin
    m = -(1.0 - duty) / max(duty, 1e-6)  # stance rate in frac units per swing-w
    w2, w3 = w * w, w * w * w
    herm = (-0.5) * (2 * w3 - 3 * w2 + 1) + m * (w3 - 2 * w2 + w) \
        + 0.5 * (-2 * w3 + 3 * w2) + m * (w3 - w2)
    return (1.0 - retract) * lin + retract * herm


class TrotGait:
    """Stateful trot generator. Hold one per robot; tick it with dt + command."""

    def __init__(self, seed: GaitSeed, config: RobotConfig | None = None,
                 scheduler=None):
        self.seed = seed
        self.config = config
        # Optional contact-triggered scheduler (dogv3.gait.contact). None, or
        # one that is disabled, leaves the gait purely time-driven -- bit for bit
        # what it was before contact scheduling existed.
        self.scheduler = scheduler
        self.clock = PhaseClock(seed.cycle_period)
        # Yaw arms. A rigid rotation of theta about the body centre moves a foot
        # at (px, py) by theta * (py, -px): the LATERAL displacement scales with
        # the foot's FORE/AFT arm and vice versa. Using one radius for both — as
        # this did with half the body WIDTH — makes the lateral term only
        # width/length of what a rigid turn needs (79% on this body), so every
        # stance foot is dragged sideways instead of tracing the turn circle.
        self._yaw_half_width = (config.body_mm.shoulder_lateral / 2.0) if config else 95.0
        self._yaw_half_length = (config.body_mm.fore_aft / 2.0) if config else 120.0
        # Fore/aft polarity. ``frame.forward_sign`` records which way the assembled
        # robot actually walks when the canonical +Y stride is commanded; it was
        # declared in the schema and shown during commissioning, but nothing ever
        # read it, so a robot assembled with its fore/aft axis reversed had no way
        # to say so and drove backwards on forward intent.
        #
        # This applies to the OPERATOR'S fore/aft intent only, not to the +Y axis
        # itself: stance geometry, the body_offset_y trim and both yaw terms keep
        # their canonical signs, so correcting forward travel cannot disturb a
        # turn that is already correct.
        self._fwd_sign = -1.0 if _is_reversed(config) else 1.0

    def step(self, dt: float, cmd: GaitCommand) -> dict[str, FootTarget]:
        self.clock.cycle_period = self.seed.cycle_period
        self.clock.advance(dt)
        return self.targets(cmd)

    def swing_progress(self, cmd: GaitCommand | None = None) -> dict[str, float | None]:
        """Per-leg progress through swing in [0, 1], or None while in stance.

        This is all :class:`~dogv3.gait.contact.ContactScheduler` needs to
        know about the gait, which keeps the two independent.
        """
        s = self.seed
        crawl = s.gait_type == "crawl"
        duty = max(s.duty_factor, CRAWL_MIN_DUTY) if crawl else s.duty_factor
        out: dict[str, float | None] = {}
        for leg in ("FL", "FR", "BL", "BR"):
            phi = self._leg_phase(leg, duty, crawl)
            out[leg] = None if phi < duty else (
                (phi - duty) / (1.0 - duty) if duty < 1 else 0.0)
        return out

    def targets(self, cmd: GaitCommand) -> dict[str, FootTarget]:
        s = self.seed
        crawl = s.gait_type == "crawl"
        # A crawl is only statically stable with >= 3/4 stance (one foot airborne
        # at a time), so the trot duty knob is floored in crawl mode.
        duty = max(s.duty_factor, CRAWL_MIN_DUTY) if crawl else s.duty_factor
        # Commanded displacement of the foot per half-stroke (mm). Body speed at
        # full stick is stride/(duty*period), so the seed's max_fwd_speed (mm/s)
        # caps the stride — the speed governor the operator can trust. max_yaw
        # (rad/s) likewise caps the per-stroke yaw arc.
        stride_cap = s.max_fwd_speed * duty * s.cycle_period
        stride_fwd = _clip(s.step_length * _clip(cmd.vy) * self._fwd_sign,
                           -stride_cap, stride_cap)
        stride_lat = _clip(s.step_length * _clip(cmd.vx), -stride_cap, stride_cap)
        yaw_cap = s.max_yaw * duty * s.cycle_period
        yaw = _clip(s.turn_gain * _clip(cmd.wz), -yaw_cap, yaw_cap)
        # Quiet idle: zero intent means STAND, not marching in place — no swing
        # lift, no wasted heat. The watchdog's hold-stand covers stale intent;
        # this covers fresh-but-zero intent.
        if abs(stride_fwd) < 0.5 and abs(stride_lat) < 0.5 and abs(yaw) < 1e-3:
            return self.stand_targets(cmd)
        bx, by = self._body_shift(crawl)

        out: dict[str, FootTarget] = {}
        for leg in ("FL", "FR", "BL", "BR"):
            phi = self._leg_phase(leg, duty, crawl)
            sx, sy = _HIP_SIGN[leg]
            # Yaw contributes a tangential stride: outer-front / inner-rear, etc.
            # Each component takes the arm that actually drives it, so the four
            # feet trace ONE common circle instead of fighting each other. The
            # lateral arm includes stance_width_offset because that is where the
            # foot really is, not where the hip axis is.
            yaw_fwd = -yaw * (self._yaw_half_width + s.stance_width_offset) * sx
            yaw_lat = yaw * self._yaw_half_length * sy
            total_fwd = stride_fwd + yaw_fwd
            total_lat = stride_lat + yaw_lat

            # Nominal stance foot position under the hip.
            nx = sx * s.stance_width_offset
            ny = 0.0
            z_down = -(s.body_height + cmd.body_z)

            if phi < duty:
                # Stance: foot drags from +stride/2 (leading) to -stride/2.
                u = phi / duty if duty > 0 else 0.0          # 0..1
                frac = 0.5 - u                                # +0.5 -> -0.5
                x = nx + total_lat * frac
                y = ny + total_fwd * frac
                # Suspension emulation: shorten the leg mid-stance so the body
                # dips as the pair loads — the spring-mass trot ride.
                z = z_down + s.stance_dip * math.sin(math.pi * u)
                in_stance = True
            else:
                # Swing: return foot from -stride/2 to +stride/2, lifting.
                w = (phi - duty) / (1.0 - duty) if duty < 1 else 0.0
                frac = _swing_frac(w, duty, s.swing_retract)  # -0.5 -> +0.5
                x = nx + total_lat * frac
                y = ny + total_fwd * frac
                z = z_down + _swing_height(w, s.swing_shape, s.step_height)
                # A leg that expected ground by now and has not found it reaches
                # further down rather than beginning its stance sweep on air.
                if self.scheduler is not None:
                    z -= self.scheduler.reach_mm(leg)
                in_stance = False

            # CoM management: leaning the body by (bx, by) moves every planted
            # foot by (-bx, -by) in the body frame, keeping feet world-fixed.
            x -= bx
            y -= by
            out[leg] = FootTarget(leg=leg, x=x, y=y, z=z, phase=phi, in_stance=in_stance)
        return out

    def _leg_phase(self, leg: str, duty: float, crawl: bool) -> float:
        """Per-leg phase in [0, 1): [0, duty) is stance, [duty, 1) is swing.

        Trot reuses the diagonal-pair offsets (identical to
        ``PhaseClock.leg_phase``); crawl shifts each leg so it begins swing
        exactly at its ``CRAWL_LIFT_ORDER`` global phase."""
        adj = self.scheduler.phase_adjust(leg) if self.scheduler is not None else 0.0
        if crawl:
            return (self.clock.phase - CRAWL_LIFT_ORDER[leg] + duty + adj) % 1.0
        return (self.clock.phase + TROT_OFFSETS[leg] + adj) % 1.0

    def _body_shift(self, crawl: bool) -> tuple[float, float]:
        """Body lean (bx, by) in mm: a static CoM trim (both gaits) plus, in
        crawl, a phase-locked LATERAL sway that leans away from the lifting foot
        — right while the (earlier-lifting) left legs swing, left while the right
        legs swing. ``body_sway_amp`` of 0 leaves only the static trim.

        There is deliberately NO longitudinal sway. A fore/aft term at twice the
        lateral frequency reads plausible — lean forward while a hind leg swings
        — but it crosses zero mid-swing and reaches the *wrong* sign by
        touchdown. A prior body-origin kinematic sweep found that term reduced
        the computed support margin, but that is not a measured CoM or physical
        stability result. Keep longitudinal shift absent until instrumented
        stand tests justify a model."""
        s = self.seed
        bx = s.body_offset_x
        by = s.body_offset_y
        if crawl and s.body_sway_amp:
            bx += s.body_sway_amp * _sway_shape(math.sin(2.0 * math.pi * self.clock.phase))
        return bx, by

    def stand_targets(self, cmd: GaitCommand | None = None) -> dict[str, FootTarget]:
        """A neutral, all-feet-planted pose for soft-start / safe stand."""
        s = self.seed
        body_z = cmd.body_z if cmd else 0.0
        # Manual trigger sway is intentionally stand-only. It never stacks with
        # the phase-locked crawl lean, which is handled by _body_shift().
        if cmd is None:
            manual_sway = 0.0
        elif cmd.sway_mm is not None:
            manual_sway = _clip(cmd.sway_mm, -MANUAL_SWAY_MAX_MM, MANUAL_SWAY_MAX_MM)
        else:
            sway_max = _clip(cmd.sway_max_mm, 0.0, MANUAL_SWAY_MAX_MM)
            manual_sway = sway_max * _clip(cmd.sway)
        if cmd is None:
            manual_pitch_deg = 0.0
        elif cmd.pitch_deg is not None:
            manual_pitch_deg = _clip(
                cmd.pitch_deg, -MANUAL_PITCH_MAX_DEG, MANUAL_PITCH_MAX_DEG)
        else:
            pitch_max = _clip(cmd.pitch_max_deg, 0.0, MANUAL_PITCH_MAX_DEG)
            manual_pitch_deg = pitch_max * _clip(cmd.pitch)
        half_length = self.config.body_mm.fore_aft / 2.0 if self.config else 120.0
        pitch_dz = math.sin(math.radians(manual_pitch_deg)) * half_length
        bx, by = s.body_offset_x + manual_sway, s.body_offset_y
        out: dict[str, FootTarget] = {}
        for leg in ("FL", "FR", "BL", "BR"):
            sx, sy = _HIP_SIGN[leg]
            out[leg] = FootTarget(
                leg=leg,
                x=sx * s.stance_width_offset - bx,
                y=-by,
                # Nose-up: front hips rise relative to the planted feet, so
                # front legs lengthen and rear legs shorten.
                z=-(s.body_height + body_z) - sy * pitch_dz,
                phase=self.clock.leg_phase(leg),
                in_stance=True,
            )
        return out


def _clip(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _is_reversed(config: RobotConfig | None) -> bool:
    """True when the frame declares that this robot's head faces -Y.

    Anything unrecognised means the canonical +Y: a typo must not silently
    reverse a robot that walks correctly.
    """
    if config is None:
        return False
    return str(config.frame.forward_sign).strip().upper().replace(" ", "") == "-Y"
