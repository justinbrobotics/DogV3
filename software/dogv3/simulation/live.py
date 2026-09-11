"""Live physics twin for the D2 operate GUI (``dogv3-operate --sim``).

One persistent MuJoCo world stepped in lockstep with the 60 Hz control loop:
the controller keeps producing foot targets exactly as it would for hardware,
and this twin plays them into physics — same IK, same commissioned count
clamps, same STS3215 SPEED/ACC command profile, same position-servo actuators
as :mod:`.rollout`. The GUI gets a world-frame skeleton back, so the operator
drives the physics robot with WASD and tunes sliders against real dynamics,
not the kinematic wireframe.

This makes D3 tuning produce D2 artifacts by construction: it is the same
server, the same gait seed, and the same profile save/load path.
"""
from __future__ import annotations

import math

from ..config.schema import RobotConfig
from ..kinematics.leg import LegGeometry, angle_to_count, count_to_angle, inverse_kinematics
from .dynamics_params import DynamicsParams
from .mujoco import LEGS, JOINTS, build_mujoco_model, stand_joint_angles
from .servo import ServoProfile

try:  # optional heavy dep
    import mujoco as mj
except ImportError:  # pragma: no cover
    mj = None

FALL_TILT_DEG = 40.0


class LiveTwin:
    """Owns one MuJoCo world. Not thread-safe: step/reset/snapshot must all be
    called from the control-loop thread (the same rule as the serial driver)."""

    def __init__(self, config: RobotConfig, params: DynamicsParams,
                 obstacle_mm: float | None = None, ramp_rad: float | None = None):
        if mj is None:
            raise RuntimeError("MuJoCo not installed: pip install mujoco (or `pip install .[sim]`)")
        self.config = config
        self.obstacle_mm = obstacle_mm
        xml = build_mujoco_model(config, params, allow_incomplete=True,
                                 obstacle_mm=obstacle_mm, ramp_rad=ramp_rad)
        self.model = mj.MjModel.from_xml_string(xml)
        self.data = mj.MjData(self.model)
        self.n_sub = max(1, round(params.simulation_rate_hz / params.control_rate_hz))

        self._act_id = {f"{leg}_{joint}": mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}")
                        for leg in LEGS for joint in JOINTS}
        self._body_id = {name: mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)
                         for name in ["body"] + [f"{leg}_{part}" for leg in LEGS
                                                 for part in ("hip", "femur", "tibia")]}
        self._site_id = {leg: mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, f"{leg}_foot_site")
                         for leg in LEGS}
        self._foot_r = float(self.model.geom(
            mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "FL_foot")).size[0])
        self._geoms = {leg: LegGeometry(L1=config.links_mm.L1_hip, L2=config.links_mm.L2_femur,
                                        L3=config.links_mm.L3_tibia, side=config.legs[leg].side,
                                        knee_sign=config.legs[leg].knee_sign) for leg in LEGS}
        self._specs = {f"{leg}_{joint}": config.servo_by_slot(f"{leg}_{joint}")
                       for leg in LEGS for joint in JOINTS}
        self._stand = dict(zip((f"{leg}_{joint}" for leg in LEGS for joint in JOINTS),
                               stand_joint_angles(config)))
        self._profiles: dict[str, ServoProfile] = {}
        self.fell = False
        self.reset()

    def reset(self) -> None:
        """Back to the stand keyframe (used after a fall, or to rehome)."""
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        mj.mj_forward(self.model, self.data)
        self._profiles = {name: ServoProfile(pos=a) for name, a in self._stand.items()}
        self.fell = False

    def joint_goals(self, targets) -> dict[str, float]:
        """Foot targets -> canonical goal angles through the same IK + count
        round trip (quantize, commissioned clamp, invert) the real bus applies."""
        out: dict[str, float] = {}
        for leg in LEGS:
            ft = targets[leg]
            angles = inverse_kinematics(self._geoms[leg], ft.x, ft.y, ft.z)
            for j, joint in enumerate(JOINTS):
                name = f"{leg}_{joint}"
                spec = self._specs[name]
                if spec is not None:
                    count = angle_to_count(angles[j], invert=spec.invert,
                                           min_raw=spec.min_raw, max_raw=spec.max_raw)
                    out[name] = count_to_angle(count, invert=spec.invert)
                else:
                    out[name] = angles[j]
        return out

    def step(self, targets) -> None:
        """Advance one control period (n_sub physics substeps) toward the
        controller's current foot targets. Keeps simulating after a fall so the
        GUI shows the ragdoll; ``fell`` stays latched until reset()."""
        goals = self.joint_goals(targets)
        ts = self.model.opt.timestep
        for _ in range(self.n_sub):
            for name, g in goals.items():
                self.data.ctrl[self._act_id[name]] = self._profiles[name].step(ts, g)
            mj.mj_step(self.model, self.data)
        if self._tilt_deg() > FALL_TILT_DEG or \
                self.data.qpos[2] < 0.35 * (self.config.gait_seed.body_height / 1000.0):
            self.fell = True

    def snapshot(self) -> dict:
        """World-frame skeleton for the browser, in mm (same {points, in_stance}
        leg shape as the kinematic pose snapshot, so the GUI reuses its drawer)."""
        legs = {}
        for leg in LEGS:
            pts = [self.data.xpos[self._body_id[f"{leg}_hip"]],
                   self.data.xpos[self._body_id[f"{leg}_femur"]],
                   self.data.xpos[self._body_id[f"{leg}_tibia"]],
                   self.data.site_xpos[self._site_id[leg]]]
            legs[leg] = {
                "points": [[round(1000.0 * float(v), 1) for v in p] for p in pts],
                "in_stance": bool(self.data.site_xpos[self._site_id[leg]][2] < self._foot_r * 1.6),
            }
        bp = self.data.xpos[self._body_id["body"]]
        return {
            "legs": legs,
            "body_mm": [round(1000.0 * float(v), 1) for v in bp],
            "tilt_deg": round(self._tilt_deg(), 1),
            "fell": self.fell,
            "obstacle_mm": self.obstacle_mm,
        }

    def _tilt_deg(self) -> float:
        up_z = float(self.data.xmat[self._body_id["body"]].reshape(3, 3)[2, 2])
        return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
