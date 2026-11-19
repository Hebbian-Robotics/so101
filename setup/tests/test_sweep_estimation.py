"""Estimates from encoder sweeps preserve travel and nominal alignment."""

import math

import pytest

from so101_setup.sweep_estimation import (
    ENCODER_MIDPOINT,
    ENCODER_RESOLUTION,
    RADIANS_PER_STEP,
    EncoderSweep,
    estimate_sweep,
)


@pytest.mark.parametrize("old_homing", [-1905, -550, 0, 1781])
@pytest.mark.parametrize("sign", [-1, 1])
def test_sweep_crossing_encoder_zero_preserves_angles_and_limits(
    old_homing: int, sign: int
) -> None:
    sweep = EncoderSweep.start(3800)
    for position in (4000, 100, 500, 1200, 1800):
        sweep.record(position)
    estimate = estimate_sweep(sweep, old_homing, sign, 0.25)
    assert estimate.range_max - estimate.range_min == 2096
    assert -2047 <= estimate.homing_offset <= 2047
    for observed, unwrapped in [(3800, 3800), (100, 4196), (1800, 5896)]:
        new_position = (observed + old_homing - estimate.homing_offset) % ENCODER_RESOLUTION
        assert estimate.range_min <= new_position <= estimate.range_max
        angle = (
            sign * (new_position - ENCODER_MIDPOINT) * RADIANS_PER_STEP
            + estimate.model_offset_radians
        )
        expected = 0.25 + sign * (unwrapped - 3800) * RADIANS_PER_STEP
        assert angle == pytest.approx(expected)


def test_wrist_sweep_can_cross_the_seam_and_cover_nearly_a_revolution() -> None:
    sweep = EncoderSweep.start(1200)
    for position in (500, 3900, 3200, 2500, 1800, 1490):
        sweep.record(position)
    estimate = estimate_sweep(sweep, -785, 1, math.radians(2.8))
    assert estimate.range_max - estimate.range_min == 3806
    assert 0 <= estimate.range_min < estimate.range_max < ENCODER_RESOLUTION


def test_unrepresentable_or_incomplete_sweeps_are_refused() -> None:
    sweep = EncoderSweep.start(2000)
    with pytest.raises(ValueError, match="nonempty"):
        estimate_sweep(sweep, 0, 1, 0.0)
    with pytest.raises(ValueError, match="half-turn"):
        sweep.record((2000 + 2048) % 4096)
    for position in (1000, 0, 3096, 2096):
        sweep.record(position)
    with pytest.raises(ValueError, match="exceeds"):
        sweep.record(1096)
