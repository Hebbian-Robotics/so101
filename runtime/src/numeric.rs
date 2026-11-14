//! The few numeric conversions the servo driver makes, each with the reason it is
//! sound, so the casts and their lint exceptions live in one place.

/// A servo register or encoder reading as a float: these are at most 32 bits
/// wide, far below the 2^52 an `f64` holds exactly.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub(crate) fn register_value(value: i64) -> f64 {
    value as f64
}

/// A value already rounded and bounded to a servo's range, as an integer.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub(crate) fn rounded(value: f64) -> i64 {
    value.round() as i64
}
