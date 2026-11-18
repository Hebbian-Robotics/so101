"""The standalone setup contract; control-loop and station settings stay outside."""

from typing import Annotated, Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

ControllerSerial = Annotated[str, Field(pattern=r"^[A-Za-z0-9_-]+$")]
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
Sign = Literal[-1, 1]
PositionGains = Annotated[list[int], Field(min_length=3, max_length=3)]
SO101_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
ModelVariant = Literal["stock", "soft-gripper-camera"]
MotorIdOrder = Literal["ascending", "descending"]


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


class ServoSettings(SetupSection):
    # Starting configuration, not an empirically qualified tune for every arm.
    expected_position_gains: PositionGains = Field(default_factory=lambda: [16, 0, 32])
    expected_position_gains_by_motor: dict[int, PositionGains] = Field(default_factory=dict)
    expected_return_delay_time: Annotated[int, Field(ge=0, le=255)] | None = 0


class ArmSetup(SetupSection):
    serial: ControllerSerial
    model_variant: ModelVariant = "stock"
    motor_id_order: MotorIdOrder = "ascending"
    alignment: ModelAlignment = Field(default_factory=ModelAlignment)
    gripper: GripperEndpoints = Field(default_factory=GripperEndpoints)
    servos: ServoSettings = Field(default_factory=ServoSettings)

    def motor_ids_in_joint_order(self) -> list[int]:
        return list(MOTOR_IDS) if self.motor_id_order == "ascending" else MOTOR_IDS[::-1]

    @model_validator(mode="after")
    def require_known_gain_overrides(self) -> Self:
        if set(self.servos.expected_position_gains_by_motor) - set(MOTOR_IDS):
            raise ValueError("Gain overrides must name an SO-101 motor (IDs 1-6)")
        return self
