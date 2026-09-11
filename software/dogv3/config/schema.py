"""Pydantic schema for ``robot_config.json`` â€” the one versioned source of truth.

Written only by the Setup Program; read-only for Runtime. Both programs
fail-fast on schema mismatch or unverified servos (see :mod:`.loader`).
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator

SCHEMA_VERSION = 1

JointName = Literal["hip", "femur", "tibia"]
LegName = Literal["FL", "FR", "BL", "BR"]
Side = Literal["left", "right"]


class FrameSpec(BaseModel):
    lateral: str = "X"
    longitudinal: str = "Y"
    up: str = "Z"
    forward_sign: str = "+Y"


class BusSpec(BaseModel):
    role: Literal["front", "rear"]
    esp32_uart: int
    rx: int
    tx: int


class LinksMM(BaseModel):
    L1_hip: float = Field(gt=0)
    L2_femur: float = Field(gt=0)
    L3_tibia: float = Field(gt=0)


class BodyMM(BaseModel):
    shoulder_lateral: float = Field(gt=0)
    fore_aft: float = Field(gt=0)


class LegIds(BaseModel):
    hip: Optional[int] = None
    femur: Optional[int] = None
    tibia: Optional[int] = None

    def filled(self) -> bool:
        return None not in (self.hip, self.femur, self.tibia)


class LegSpec(BaseModel):
    bus: Literal["A", "B"]
    side: Side
    knee_sign: int = 1
    ids: LegIds = Field(default_factory=LegIds)

    @field_validator("knee_sign")
    @classmethod
    def _knee_pm1(cls, v: int) -> int:
        if v not in (-1, 1):
            raise ValueError("knee_sign must be +1 or -1")
        return v


class VerifiedBits(BaseModel):
    assigned: bool = False
    center: bool = False
    direction: bool = False
    range: bool = False

    def all_set(self) -> bool:
        return self.assigned and self.center and self.direction and self.range


class ServoSpec(BaseModel):
    slot: str  # e.g. "FL_hip"
    invert: bool = False
    ofs_calibrated: bool = False
    eeprom_min: int = 0
    eeprom_max: int = 4095
    home_raw: int = 2048
    min_raw: int = 0
    max_raw: int = 4095
    soft_min_deg: float = -90
    soft_max_deg: float = 90
    verified: VerifiedBits = Field(default_factory=VerifiedBits)

    @model_validator(mode="after")
    def _ranges_sane(self) -> "ServoSpec":
        if not (0 <= self.min_raw <= self.max_raw <= 4095):
            raise ValueError(f"raw range out of bounds for slot {self.slot}")
        if not (self.min_raw <= self.home_raw <= self.max_raw):
            raise ValueError(f"home_raw not within [min_raw, max_raw] for slot {self.slot}")
        if self.soft_min_deg > self.soft_max_deg:
            raise ValueError(f"soft_min_deg > soft_max_deg for slot {self.slot}")
        return self


class GaitSeed(BaseModel):
    gait_type: Literal["trot", "crawl"] = "trot"
    body_height: float = 160
    step_length: float = 40
    step_height: float = 30
    cycle_period: float = Field(default=0.6, gt=0)
    duty_factor: float = Field(default=0.6, gt=0, lt=1)
    stance_width_offset: float = 10
    body_sway_amp: float = 0
    body_offset_x: float = 0
    body_offset_y: float = 0
    turn_gain: float = 0.5
    max_fwd_speed: float = 80
    max_yaw: float = 0.6
    swing_shape: Literal["parabola", "sine", "cycloid", "flick"] = "parabola"
    # 0 = legacy constant-speed swing; 1 = full touchdown retraction (the foot
    # lands with ~zero ground-relative velocity, like an animal's paw flick).
    swing_retract: float = Field(default=0.0, ge=0, le=1)
    # Suspension emulation: mm of mid-stance leg shortening (body dips as the
    # diagonal pair loads, like a spring-mass trot). 0 = rigid legacy ride.
    stance_dip: float = Field(default=0.0, ge=0, le=15)
    # Intent shaping: max change of |vx|/|vy|/|wz| per second (full-scale
    # units). Commands slew instead of stepping â€” the "Spot glide" feel.
    accel_limit: float = Field(default=3.0, gt=0)


class TrickStep(BaseModel):
    """One step of a trick: blend into a named pose, then dwell there."""

    pose: str
    move_s: float = Field(default=1.0, gt=0.05, le=10)   # blend time into the pose
    hold_s: float = Field(default=0.5, ge=0, le=30)      # dwell at the pose


class NetworkSpec(BaseModel):
    """Robot-LAN addressing (GL.iNet Opal topology). Optional with full
    defaults so every existing config loads unchanged; never part of the
    commissioning gate. The Pi copy of robot_config.json is the single home
    of the config once the Pi is in play â€” these fields tell Home-PC tools
    where to find the robot's services."""

    pi_host: str = "dogv3.local"   # Set to your own Pi hostname
    operate_port: int = 8001          # dogv3-operate served from the Pi
    setup_port: int = 8000            # dogv3-setup (on-demand, on the Pi)
    camera_port: int = 8080           # MJPEG stream (dogv3-pi-camera)
    lidar_port: int = 8090            # point-cloud viewer (dogv3-pi-lidar)
    audio_port: int = 8091            # sfx/TTS service (dogv3-pi-audio)
    # Shared secret required for mutating endpoints whenever a GUI server
    # binds a non-loopback host. Empty = loopback-only operation.
    token: str = ""

    def camera_stream_url(self) -> str:
        return f"http://{self.pi_host}:{self.camera_port}/stream.mjpg"

    def operate_url(self) -> str:
        return f"http://{self.pi_host}:{self.operate_port}"

    def lidar_viewer_url(self) -> str:
        """Point-cloud viewer page, embedded as a panel inside the operate GUI
        so watching it never costs the operate tab its focus (pad polling is
        focus-gated, and a stolen focus stalls intent into hold-stand)."""
        return f"http://{self.pi_host}:{self.lidar_port}/"

    def audio_base_url(self) -> str:
        """Audio service root. The operate server proxies to it rather than
        letting the browser call it cross-origin."""
        return f"http://{self.pi_host}:{self.audio_port}"


class CameraSpec(BaseModel):
    """OV5647 CSI camera served by picamera2 hardware MJPEG. IR LEDs are
    photoresistor-switched on the camera board â€” no software involvement."""

    enabled: bool = True
    width: int = 1280
    height: int = 720
    fps: int = 30
    quality: int = 80  # JPEG quality hint for the hardware encoder


class LidarSpec(BaseModel):
    """RPLIDAR C1: classic A-series standard protocol, 460800 baud over its
    CP2102 USB adapter; VCC fed from the UBEC rail (800 mA spin-up)."""

    enabled: bool = True
    device: str = "/dev/rplidar"  # udev symlink; falls back to /dev/ttyUSB*
    baudrate: int = 460800


class AudioSpec(BaseModel):
    """USB sound card (driver-free UAC). Clips + offline TTS."""

    enabled: bool = True
    alsa_device: str = "default"
    tts_engine: Literal["espeak-ng", "piper"] = "espeak-ng"
    volume: int = Field(default=80, ge=0, le=100)


class ImuSpec(BaseModel):
    """BNO085 mounted at body center, wired to the Pi I2C header
    (SDA GPIO2 / SCL GPIO3). Read on the Pi and fed to the onboard
    controller in-process; the ESP32 I2C route is not used."""

    enabled: bool = False  # flip on once the sensor is wired to the Pi
    i2c_bus: int = 1
    # 0x4A or 0x4B depending on the board's ADR pin â€” check with an I2C scan
    # rather than assuming; a wrong address reads as "sensor absent".
    i2c_address: int = 0x4A
    # 20 Hz, NOT 60. The BNO085 needs a 50 kHz I2C clock (it stretches), and at
    # 60 Hz the reader thread starves the control thread's serial reads badly
    # enough that torque-enable writes stop being acknowledged â€” measured on the
    # robot as torque 0/12 with the loop overrunning, and 12/12 the moment the
    # IMU was disabled. 20 Hz measures ~16 Hz actual, costs nothing on the loop
    # (p99 3.4 ms), and is far more attitude bandwidth than the stabilizer needs.
    rate_hz: int = 20


class StabilizerSpec(BaseModel):
    """Attitude feedback: trim STANCE leg length to hold the body level.

    Optional with full defaults and OFF by default, deliberately. Which way the
    BNO085's roll and pitch axes point depends on how the board is physically
    oriented on the chassis, and a wrong sign turns this into positive feedback
    that amplifies a tilt instead of cancelling it. Verify the signs on the
    stand before enabling â€” see ``RobotController._apply_stabilizer``.

    ``level_*_deg`` is what the sensor reads with the robot standing level on
    flat ground. It is subtracted before any correction, so it absorbs both a
    skewed sensor mount and any residual asymmetry in the stand pose; the
    controller then drives toward "the attitude that was level", not toward a
    raw zero that may never have been level.
    """

    enabled: bool = False
    gain: float = Field(default=0.6, ge=0.0, le=1.5)  # 1.0 = full geometric correction
    deadband_deg: float = Field(default=0.8, ge=0.0)  # ignore noise and normal gait sway
    max_trim_mm: float = Field(default=15.0, ge=0.0, le=40.0)  # hard clamp per leg
    slew_mm_s: float = Field(default=80.0, gt=0.0)    # cannot outrun the gait
    # A sensor thread can stall without immediately changing its last good
    # quaternion. Never keep steering the stance from an old attitude.
    max_imu_age_s: float = Field(default=0.25, gt=0.0, le=2.0)
    invert_roll: bool = False
    invert_pitch: bool = False
    level_roll_deg: float = 0.0
    level_pitch_deg: float = 0.0


class PeripheralsSpec(BaseModel):
    camera: CameraSpec = Field(default_factory=CameraSpec)
    lidar: LidarSpec = Field(default_factory=LidarSpec)
    audio: AudioSpec = Field(default_factory=AudioSpec)
    imu: ImuSpec = Field(default_factory=ImuSpec)
    # ESP32 serial device on the Pi (udev symlink pinned by USB port path;
    # both the ESP32 clone and the lidar adapter can be CP2102s).
    esp32_device: str = "/dev/dogv3-esp32"


class RobotConfig(BaseModel):
    schema_version: int = SCHEMA_VERSION
    firmware_expected: str = "DOGV3-MUX v1.0"
    frame: FrameSpec = Field(default_factory=FrameSpec)
    buses: dict[str, BusSpec]
    links_mm: LinksMM
    body_mm: BodyMM
    joint_order: list[JointName] = Field(default_factory=lambda: ["hip", "femur", "tibia"])
    legs: dict[str, LegSpec]
    servos: dict[str, ServoSpec] = Field(default_factory=dict)
    # Pi/LAN-era sections: optional with full defaults, deliberately OUTSIDE
    # fully_commissioned()/unverified_servos() so commissioning semantics are
    # untouched (see CLAUDE.md).
    network: NetworkSpec = Field(default_factory=NetworkSpec)
    peripherals: PeripheralsSpec = Field(default_factory=PeripheralsSpec)
    stabilizer: StabilizerSpec = Field(default_factory=StabilizerSpec)
    gait_seed: GaitSeed = Field(default_factory=GaitSeed)
    # Named gait profiles: full lever sets saved from the D2 GUI for
    # trial-and-error tuning â€” save each candidate, then load/compare on the
    # robot (in air or on ground). ``gait_seed`` stays the boot/default gait;
    # profiles are the library. D3 twin tooling reads these as the set of
    # candidate gaits to evaluate in simulation.
    gait_profiles: dict[str, GaitSeed] = Field(default_factory=dict)
    # Named custom poses: name -> {leg: [x, y, z] foot target in the leg frame,
    # mm}. Authored/tuned in the D2 GUI, persisted here so the commissioned
    # config stays the single source of truth for everything the robot does.
    poses: dict[str, dict[LegName, list[float]]] = Field(default_factory=dict)
    # Named tricks: ordered pose sequences. The controller always enters a
    # trick from (a blend to) the stand "home" pose and returns to stand at
    # the end â€” the safe-entry rule â€” so steps only need to name poses.
    tricks: dict[str, list[TrickStep]] = Field(default_factory=dict)

    @field_validator("tricks")
    @classmethod
    def _tricks_nonempty(cls, v: dict) -> dict:
        for name, steps in v.items():
            if not steps:
                raise ValueError(f"trick {name!r} has no steps")
        return v

    @field_validator("poses")
    @classmethod
    def _poses_complete(cls, v: dict) -> dict:
        for name, feet in v.items():
            if set(feet) != {"FL", "FR", "BL", "BR"}:
                raise ValueError(f"pose {name!r} must define all four legs")
            for leg, xyz in feet.items():
                if len(xyz) != 3:
                    raise ValueError(f"pose {name!r} {leg} must be [x, y, z]")
        return v

    @field_validator("schema_version")
    @classmethod
    def _version_match(cls, v: int) -> int:
        if v != SCHEMA_VERSION:
            raise ValueError(
                f"schema_version {v} != supported {SCHEMA_VERSION}; both programs fail-fast on mismatch"
            )
        return v

    @model_validator(mode="after")
    def _legs_present(self) -> "RobotConfig":
        expected = {"FL", "FR", "BL", "BR"}
        if set(self.legs) != expected:
            raise ValueError(f"legs must be exactly {expected}, got {set(self.legs)}")
        return self

    # -- convenience helpers --------------------------------------------
    def slot_of(self, leg: str, joint: str) -> str:
        return f"{leg}_{joint}"

    def id_for(self, leg: str, joint: str) -> Optional[int]:
        return getattr(self.legs[leg].ids, joint)

    def servo_by_slot(self, slot: str) -> Optional[ServoSpec]:
        for s in self.servos.values():
            if s.slot == slot:
                return s
        return None

    def all_slots_assigned(self) -> bool:
        return all(leg.ids.filled() for leg in self.legs.values())

    def unverified_servos(self) -> list[str]:
        """IDs whose verified bits are not all set."""
        return [sid for sid, s in self.servos.items() if not s.verified.all_set()]

    def commissioning_integrity_errors(self) -> list[str]:
        """Return ID/slot mapping faults that make a config unsafe to drive.

        Partial configs remain valid during D1, but the runtime commissioning
        gate must reject duplicate assigned IDs, missing servo records, and
        records whose ``slot`` does not point back to the assigning leg/joint.
        """
        errors: list[str] = []
        slots_by_id: dict[int, list[str]] = {}
        assigned_by_slot: dict[str, int] = {}

        for leg in ("FL", "FR", "BL", "BR"):
            for joint in ("hip", "femur", "tibia"):
                slot = self.slot_of(leg, joint)
                sid = self.id_for(leg, joint)
                if sid is None:
                    continue
                slots_by_id.setdefault(sid, []).append(slot)
                assigned_by_slot[slot] = sid
                servo = self.servos.get(str(sid))
                if servo is None:
                    errors.append(f"{slot} references missing servo record {sid}")
                elif servo.slot != slot:
                    errors.append(
                        f"servo {sid} is assigned to {slot} but its record says {servo.slot}"
                    )

        for sid, slots in sorted(slots_by_id.items()):
            if len(slots) > 1:
                errors.append(f"duplicate servo ID {sid} assigned to {', '.join(slots)}")

        for sid_text, servo in self.servos.items():
            expected_id = assigned_by_slot.get(servo.slot)
            if expected_id is None:
                errors.append(f"servo record {sid_text} claims unassigned slot {servo.slot}")
            elif str(expected_id) != sid_text:
                errors.append(
                    f"servo record {sid_text} claims {servo.slot}, assigned to ID {expected_id}"
                )
        return errors

    def fully_commissioned(self) -> bool:
        return (
            self.all_slots_assigned()
            and len(self.servos) == 12
            and not self.unverified_servos()
            and not self.commissioning_integrity_errors()
        )
