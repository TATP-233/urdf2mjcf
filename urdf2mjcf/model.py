"""Defines the Pydantic model for the URDF to MJCF conversion."""

import enum
from typing import Literal

from pydantic import BaseModel

Angle = Literal["radian", "degree"]
SiteType = Literal["sphere", "capsule", "ellipsoid", "cylinder", "box"]


class CollisionParams(BaseModel):
    condim: int = 3
    contype: int = 0
    conaffinity: int = 1
    priority: int = 1
    solimp: list[float] = [0.99, 0.999, 0.00001]
    solref: list[float] = [0.005, 1.0]
    friction: list[float] = [1.0, 0.01, 0.01]


class dJoint(BaseModel):
    stiffness: float | None = None
    actuatorfrcrange: list[float] | None = None
    margin: float | None = None
    armature: float | None = None
    damping: float | None = None
    frictionloss: float | None = None

class dActuator(BaseModel):
    actuator_type: str | None = None
    kp: float | None = None
    kv: float | None = None
    gear: float | None = None
    ctrlrange: list[float] | None = None
    forcerange: list[float] | None = None

class DefaultJointMetadata(BaseModel):
    joint: dJoint
    actuator: dActuator

    @classmethod
    def from_dict(cls, data: dict) -> "DefaultJointMetadata":
        """Create DefaultJointMetadata from a plain dictionary."""
        joint = dJoint.model_validate(data["joint"])
        actuator = dActuator.model_validate(data["actuator"])
        return cls(joint=joint, actuator=actuator)

class ActuatorMetadata(BaseModel):
    joint_class: str | None = None
    actuator_type: str | None = None
    kp: float | None = None
    kv: float | None = None
    gear: float | None = None
    ctrlrange: list[float] | None = None
    forcerange: list[float] | None = None

    @classmethod
    def from_dict(cls, data: dict) -> "ActuatorMetadata":
        """Create JointParam from a plain dictionary."""
        return cls(**data)

class SiteMetadata(BaseModel):
    name: str
    body_name: str
    site_type: SiteType | None = None
    size: list[float] | None = None
    pos: list[float] | None = None

class ImuSensor(BaseModel):
    body_name: str
    pos: list[float] | None = None
    rpy: list[float] | None = None
    acc_noise: float | None = None
    gyro_noise: float | None = None
    mag_noise: float | None = None


class CameraSensor(BaseModel):
    name: str
    mode: str
    pos: list[float] | None = None
    rpy: list[float] | None = None
    fovy: float = 45.0


class ForceSensor(BaseModel):
    """Represents a force sensor attached to a site."""

    body_name: str
    site_name: str
    name: str | None = None
    noise: float | None = None


class TouchSensor(BaseModel):
    """Represents a touch sensor attached to a site."""

    body_name: str
    site_name: str
    name: str | None = None
    noise: float | None = None


class ExplicitFloorContacts(BaseModel):
    """Add explicit floor contacts."""

    contact_links: list[str]
    class_name: str = "collision"


class WeldConstraint(BaseModel):
    """Represents a weld constraint between two bodies.

    Useful for locking bodies in the air (suspending).
    By default, body2 is set to "world" to create a fixed constraint in global space.
    Default parameters approximately lock the body rigidly in place -- stiffer than Mujoco defaults.
    """

    body1: str
    body2: str = "world"
    solimp: list[float] = [0.95, 0.99, 0.005, 0.5, 2]  # Very tight width, aggressive response.
    solref: list[float] = [0.005, 1.0]  # critical damping 1.0, very quick time constant to return to static


class CollisionType(enum.Enum):
    BOX = enum.auto()
    PARALLEL_CAPSULES = enum.auto()
    CORNER_SPHERES = enum.auto()
    SINGLE_SPHERE = enum.auto()


class CollisionGeometry(BaseModel):
    name: str
    collision_type: CollisionType
    sphere_radius: float = 0.01
    axis_order: tuple[int, int, int] = (0, 1, 2)
    flip_axis: bool = False
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0


class ConversionMetadata(BaseModel):
    freejoint: bool = True
    collision_params: CollisionParams = CollisionParams()
    # joint_name_to_metadata: dict[str, ActuatorMetadata] | None = None
    # actuator_type_to_metadata: dict[str, JointMetadata] | None = None
    imus: list[ImuSensor] = []
    cameras: list[CameraSensor] = [
        CameraSensor(
            name="front_camera",
            mode="track",
            pos=[0, 2.0, 0.5],
            rpy=[90.0, 0.0, 180.0],
            fovy=90,
        ),
        CameraSensor(
            name="side_camera",
            mode="track",
            pos=[-2.0, 0.0, 0.5],
            rpy=[90.0, 0.0, 270.0],
            fovy=90,
        ),
    ]
    sites: list[SiteMetadata] = []
    force_sensors: list[ForceSensor] = []
    touch_sensors: list[TouchSensor] = []
    collision_geometries: list[CollisionGeometry] | None = None
    explicit_contacts: ExplicitFloorContacts | None = None
    weld_constraints: list[WeldConstraint] = []
    remove_redundancies: bool = True
    floating_base: bool = True
    maxhullvert: int | None = None
    angle: Angle = "radian"
    floor_name: str = "floor"
    add_floor: bool = True
    backlash: float | None = None
    backlash_damping: float = 0.01
    height_offset: float = 0.0
    visualize_collision_meshes: bool = True