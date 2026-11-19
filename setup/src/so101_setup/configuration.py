"""The standalone setup contract; control-loop and station settings stay outside."""

from typing import Annotated, Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

ControllerSerial = Annotated[str, Field(pattern=r"^[A-Za-z0-9_-]+$")]
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
Sign = Literal[-1, 1]
ByteRegisterValue = Annotated[int, Field(ge=0, le=255, strict=True)]
PositionGains = Annotated[list[ByteRegisterValue], Field(min_length=3, max_length=3)]
SO101_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
ModelVariant = Literal["stock", "soft-gripper-camera"]
DEFAULT_NEW_ARM_MODEL: ModelVariant = "soft-gripper-camera"
MOTOR_NAMES = (*SO101_JOINT_NAMES, "gripper")
MotorIds = dict[str, Annotated[int, Field(strict=True, ge=1, le=6)]]


def motor_id_mapping(ids: list[int]) -> dict[str, int]:
    return dict(zip(MOTOR_NAMES, ids, strict=True))


class SetupSection(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ModelAlignment(SetupSection):
    joint_signs: Annotated[tuple[Sign, ...], Field(min_length=5, max_length=5)] = (
        1,
        1,
        1,
        1,
        1,
    )
    joint_offsets_radians: Annotated[tuple[FiniteFloat, ...], Field(min_length=5, max_length=5)] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    gripper_sign: Sign = 1
    gripper_offset_radians: FiniteFloat = 0.0


class GripperEndpoints(SetupSection):
    closed_position_radians: FiniteFloat = 0.0
    open_position_radians: FiniteFloat = 1.5

    @model_validator(mode="after")
    def require_distinct_endpoints(self) -> Self:
        if self.closed_position_radians == self.open_position_radians:
            raise ValueError("Closed and open gripper positions must differ")
        return self


class ServoRegisterSettings(SetupSection):
    maximum_acceleration: Annotated[int, Field(ge=0, le=254)] = 254
    gripper_max_torque_limit: Annotated[int, Field(ge=0, le=1000)] = 500
    gripper_protection_current: Annotated[int, Field(ge=0, le=310)] = 250


class ServoSettings(SetupSection):
    # Starting configuration, not an empirically qualified tune for every arm.
    expected_position_gains: PositionGains = Field(default_factory=lambda: [16, 0, 32])
    expected_position_gains_by_motor: dict[int, PositionGains] = Field(default_factory=dict)
    expected_return_delay_time: ByteRegisterValue | None = 0


class ArmSetup(SetupSection):
    serial: ControllerSerial
    model_variant: ModelVariant = "stock"
    motor_ids: MotorIds = Field(default_factory=lambda: motor_id_mapping(MOTOR_IDS))
    alignment: ModelAlignment = Field(default_factory=ModelAlignment)
    gripper: GripperEndpoints = Field(default_factory=GripperEndpoints)
    servos: ServoSettings = Field(default_factory=ServoSettings)
    servo_registers: ServoRegisterSettings = Field(default_factory=ServoRegisterSettings)

    def motor_ids_in_joint_order(self) -> list[int]:
        return [self.motor_ids[name] for name in MOTOR_NAMES]

    @model_validator(mode="after")
    def require_known_gain_overrides(self) -> Self:
        if set(self.motor_ids) != set(MOTOR_NAMES) or set(self.motor_ids.values()) != set(
            MOTOR_IDS
        ):
            raise ValueError("Motor mapping must assign IDs 1-6 exactly once to the six joints")
        if set(self.servos.expected_position_gains_by_motor) - set(MOTOR_IDS):
            raise ValueError("Gain overrides must name an SO-101 motor (IDs 1-6)")
        return self
