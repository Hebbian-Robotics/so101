"""Read-only encoder sweep math; estimates require operator verification."""

import math
from dataclasses import dataclass

ENCODER_RESOLUTION = 4096
ENCODER_MIDPOINT = 2047
RADIANS_PER_STEP = math.tau / ENCODER_RESOLUTION


@dataclass
class EncoderSweep:
    """Track a hand-moved single-turn encoder without losing seam crossings."""

    previous: int
    minimum: int
    maximum: int

    @classmethod
    def start(cls, position: int) -> "EncoderSweep":
        if not 0 <= position < ENCODER_RESOLUTION:
            raise ValueError("Encoder reading must be within one revolution")
        return cls(position, position, position)

    def record(self, position: int) -> int:
        if not 0 <= position < ENCODER_RESOLUTION:
            raise ValueError("Encoder reading must be within one revolution")
        delta = (position - self.previous + ENCODER_RESOLUTION // 2) % ENCODER_RESOLUTION
        if delta == 0:
            raise ValueError("Ambiguous half-turn between samples; move more slowly")
        unwrapped = self.previous + delta - ENCODER_RESOLUTION // 2
        minimum = min(self.minimum, unwrapped)
        maximum = max(self.maximum, unwrapped)
        if maximum - minimum > ENCODER_RESOLUTION:
            raise ValueError("Recorded travel exceeds one revolution")
        self.previous, self.minimum, self.maximum = unwrapped, minimum, maximum
        return unwrapped


@dataclass(frozen=True)
class SweepEstimate:
    """Encoder limits and a model-frame alignment derived from a sweep."""

    homing_offset: int
    range_min: int
    range_max: int
    model_offset_radians: float


def estimate_sweep(
    sweep: EncoderSweep,
    original_homing: int,
    sign: int,
    nominal_center_radians: float,
) -> SweepEstimate:
    """Center a recorded arc and align its midpoint to a nominal model angle.

    The model center is an estimate based on the selected assembly's expected
    stops; it is not evidence that the operator reached the real hard stops.
    """
    if sign not in (-1, 1) or not math.isfinite(nominal_center_radians):
        raise ValueError("Expected a valid joint direction and finite model center")
    low, high = sweep.minimum, sweep.maximum
    if low >= high:
        raise ValueError("Sweep the joint through a nonempty range")
    if high - low == ENCODER_RESOLUTION:
        high -= 1  # one full revolution has only 4096 distinct encoder positions
    midpoint = (low + high) / 2
    for center in (
        math.floor(midpoint),
        math.ceil(midpoint),
        math.floor(midpoint) - 1,
        math.ceil(midpoint) + 1,
    ):
        shift = ENCODER_MIDPOINT - center
        range_min, range_max = low + shift, high + shift
        homing = (
            original_homing - shift + ENCODER_RESOLUTION // 2
        ) % ENCODER_RESOLUTION - ENCODER_RESOLUTION // 2
        if 0 <= range_min < range_max < ENCODER_RESOLUTION and -2047 <= homing <= 2047:
            model_offset = nominal_center_radians - sign * (midpoint - center) * RADIANS_PER_STEP
            return SweepEstimate(homing, range_min, range_max, model_offset)
    raise ValueError("Sweep cannot be represented by the servo's limits")
