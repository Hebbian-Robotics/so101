use anyhow::{Result, bail, ensure};

use crate::calibration::{STS3215_RESOLUTION, encoder_steps_to_radians};
use crate::settings::Sign;

const HALF_REVOLUTION: i64 = STS3215_RESOLUTION / 2;
pub const ENCODER_MIDPOINT: i64 = HALF_REVOLUTION - 1;

#[derive(Clone, Copy, Debug)]
pub struct EncoderSweep {
    reference: i64,
    previous: i64,
    pub minimum: i64,
    pub maximum: i64,
}

#[derive(Clone, Copy, Debug)]
pub struct SweepEstimate {
    pub homing_offset: i64,
    pub range_min: i64,
    pub range_max: i64,
    pub reference_steps: i64,
    pub model_offset_radians: f64,
}

impl EncoderSweep {
    pub fn start(position: i64) -> Result<Self> {
        validate_position(position)?;
        Ok(Self {
            reference: position,
            previous: position,
            minimum: position,
            maximum: position,
        })
    }

    pub fn record(&mut self, position: i64) -> Result<()> {
        validate_position(position)?;
        let delta = (position - self.previous + HALF_REVOLUTION).rem_euclid(STS3215_RESOLUTION);
        ensure!(
            delta != 0,
            "Ambiguous half-turn between samples. Move more slowly."
        );
        let unwrapped = self.previous + delta - HALF_REVOLUTION;
        let minimum = self.minimum.min(unwrapped);
        let maximum = self.maximum.max(unwrapped);
        ensure!(
            maximum - minimum <= STS3215_RESOLUTION,
            "Recorded travel exceeds one revolution"
        );
        self.previous = unwrapped;
        self.minimum = minimum;
        self.maximum = maximum;
        Ok(())
    }

    pub fn estimate(
        self,
        original_homing: i64,
        sign: Sign,
        reference_radians: f64,
    ) -> Result<SweepEstimate> {
        ensure!(
            (-ENCODER_MIDPOINT..=ENCODER_MIDPOINT).contains(&original_homing)
                && reference_radians.is_finite(),
            "Invalid encoder offset or reference angle"
        );
        let low = self.minimum;
        let mut high = self.maximum;
        ensure!(low < high, "Sweep the joint through a nonempty range");
        if high - low == STS3215_RESOLUTION {
            high -= 1;
        }
        let sum = low + high;
        let floor = sum.div_euclid(2);
        let ceil = floor + sum.rem_euclid(2);
        for center in [floor, ceil, floor - 1, ceil + 1] {
            let shift = ENCODER_MIDPOINT - center;
            let range_min = low + shift;
            let range_max = high + shift;
            let homing_offset = (original_homing - shift + HALF_REVOLUTION)
                .rem_euclid(STS3215_RESOLUTION)
                - HALF_REVOLUTION;
            if range_min >= 0
                && range_max < STS3215_RESOLUTION
                && homing_offset >= -ENCODER_MIDPOINT
            {
                let reference_steps = (self.reference + shift).rem_euclid(STS3215_RESOLUTION);
                let model_offset_radians =
                    reference_radians - joint_radians(reference_steps, sign, 0.0);
                return Ok(SweepEstimate {
                    homing_offset,
                    range_min,
                    range_max,
                    reference_steps,
                    model_offset_radians,
                });
            }
        }
        bail!("Sweep cannot be represented by the servo's limits")
    }
}

pub(super) fn validate_position(position: i64) -> Result<()> {
    ensure!(
        (0..STS3215_RESOLUTION).contains(&position),
        "Encoder reading must be within one revolution, got {position}"
    );
    Ok(())
}

#[must_use]
pub fn joint_radians(position: i64, sign: Sign, offset: f64) -> f64 {
    encoder_steps_to_radians(position, sign.as_integer(), offset)
}

pub(super) fn rehomed_position(position: i64, previous: i64, new: i64) -> i64 {
    (position + previous - new).rem_euclid(STS3215_RESOLUTION)
}

pub(super) fn position_distance(first: i64, second: i64) -> i64 {
    let difference = (second - first).abs();
    difference.min(STS3215_RESOLUTION - difference)
}
