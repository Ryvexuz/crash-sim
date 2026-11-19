//! Crash pulse filtering and metrics: SAE J211 CFC filter, delta-V, peak, duration and OLC.
//!
//! Self-contained and std only: every function takes plain slices of uniformly sampled
//! acceleration (m/s^2, a deceleration is negative) and a sample step `dt` in seconds.
//! Not registered in `lib.rs` here; a later task does that. Tests include it with `#[path]`.

use std::fmt;

/// Why a pulse function refused its input. Nothing here panics on bad input.
#[derive(Debug, Clone, PartialEq)]
pub enum PulseError {
    /// `dt` is not a finite number above zero.
    BadDt(f64),
    /// `cfc` is not a finite number above zero.
    BadCfc(f64),
    /// The sample rate is under 10 x CFC (SAE J211 minimum).
    SampleRateTooLow { rate_hz: f64, min_hz: f64 },
    /// A sample (or the threshold / v0 argument) is NaN or infinite; the index is the sample's.
    NonFinite { index: usize },
    /// A scalar argument (threshold or v0) is not usable.
    BadArgument(&'static str),
    /// No samples were given.
    Empty,
}

impl fmt::Display for PulseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PulseError::BadDt(v) => write!(f, "dt must be finite and > 0, got {v}"),
            PulseError::BadCfc(v) => write!(f, "cfc must be finite and > 0, got {v}"),
            PulseError::SampleRateTooLow { rate_hz, min_hz } => write!(
                f,
                "sample rate {rate_hz} Hz is under the J211 minimum of 10 x CFC = {min_hz} Hz"
            ),
            PulseError::NonFinite { index } => write!(f, "sample {index} is not finite"),
            PulseError::BadArgument(what) => write!(f, "bad argument: {what}"),
            PulseError::Empty => write!(f, "no samples"),
        }
    }
}

impl std::error::Error for PulseError {}

fn check_dt(dt: f64) -> Result<(), PulseError> {
    if dt.is_finite() && dt > 0.0 {
        Ok(())
    } else {
        Err(PulseError::BadDt(dt))
    }
}

fn check_samples(samples: &[f64]) -> Result<(), PulseError> {
    match samples.iter().position(|v| !v.is_finite()) {
        Some(index) => Err(PulseError::NonFinite { index }),
        None => Ok(()),
    }
}

/// SAE J211-1 (March 1995) Appendix C digital filter, "phaseless" 4-pole Butterworth.
///
/// A 2-pole Butterworth is run forward over the data and then backward over its own
/// output, so the phase shift cancels and the result is 4-pole overall. With T = `dt`:
///
/// ```text
/// wd = 2 pi CFC 2.0775
/// wa = sin(wd T / 2) / cos(wd T / 2)
/// a0 = wa^2 / (1 + sqrt(2) wa + wa^2)
/// a1 = 2 a0
/// a2 = a0
/// b1 = -2 (wa^2 - 1) / (1 + sqrt(2) wa + wa^2)
/// b2 = (-1 + sqrt(2) wa - wa^2) / (1 + sqrt(2) wa + wa^2)
/// Y[t] = a0 X[t] + a1 X[t-1] + a2 X[t-2] + b1 Y[t-1] + b2 Y[t-2]
/// ```
///
/// Ends: the data is extended at both ends by an odd (point) reflection about the end
/// sample, `x[-k] = 2 x[0] - x[k]`, for `pad = min(n - 1, ceil(3 / (CFC T)))` samples (three
/// periods of CFC), and the filter state is started at its steady state for the first
/// padded value. The reflection keeps value and slope continuous at the ends, so a constant
/// signal comes out exactly constant and a ramp or a pulse starting off zero does not ring
/// from a step at the boundary. The padding is cut off again; the output has `samples.len()`
/// values. (J211 itself only says to pad; the length and the odd reflection are this
/// module's choice.)
///
/// Refuses (never panics): `dt` or `cfc` not finite and > 0, a sample rate under
/// 10 x CFC (J211's minimum), any non-finite sample.
pub fn cfc_filter(samples: &[f64], dt: f64, cfc: f64) -> Result<Vec<f64>, PulseError> {
    check_dt(dt)?;
    if !(cfc.is_finite() && cfc > 0.0) {
        return Err(PulseError::BadCfc(cfc));
    }
    let rate_hz = 1.0 / dt;
    let min_hz = 10.0 * cfc;
    if rate_hz < min_hz {
        return Err(PulseError::SampleRateTooLow { rate_hz, min_hz });
    }
    check_samples(samples)?;
    let n = samples.len();
    if n < 2 {
        return Ok(samples.to_vec());
    }

    let wd = 2.0 * std::f64::consts::PI * cfc * 2.0775;
    let wa = (wd * dt / 2.0).sin() / (wd * dt / 2.0).cos();
    let s2 = std::f64::consts::SQRT_2;
    let den = 1.0 + s2 * wa + wa * wa;
    let a0 = wa * wa / den;
    let a1 = 2.0 * a0;
    let a2 = a0;
    let b1 = -2.0 * (wa * wa - 1.0) / den;
    let b2 = (-1.0 + s2 * wa - wa * wa) / den;

    let pad = (((3.0 / (cfc * dt)).ceil()) as usize).min(n - 1);
    let mut ext = Vec::with_capacity(n + 2 * pad);
    for k in (1..=pad).rev() {
        ext.push(2.0 * samples[0] - samples[k]);
    }
    ext.extend_from_slice(samples);
    for k in 1..=pad {
        ext.push(2.0 * samples[n - 1] - samples[n - 1 - k]);
    }

    let pass = |x: &[f64]| -> Vec<f64> {
        let mut y = Vec::with_capacity(x.len());
        // Steady state for a constant input equal to the first value.
        let (mut x1, mut x2, mut y1, mut y2) = (x[0], x[0], x[0], x[0]);
        for &xt in x {
            let yt = a0 * xt + a1 * x1 + a2 * x2 + b1 * y1 + b2 * y2;
            y.push(yt);
            x2 = x1;
            x1 = xt;
            y2 = y1;
            y1 = yt;
        }
        y
    };
    let mut fwd = pass(&ext);
    fwd.reverse();
    let mut both = pass(&fwd);
    both.reverse();
    Ok(both[pad..pad + n].to_vec())
}

/// Change in speed: the trapezoid integral of the samples, in m/s for m/s^2 input.
/// An empty or one-sample slice integrates to 0.
pub fn delta_v(samples: &[f64], dt: f64) -> Result<f64, PulseError> {
    check_dt(dt)?;
    check_samples(samples)?;
    Ok(samples.windows(2).map(|w| 0.5 * (w[0] + w[1]) * dt).sum())
}

/// The most negative sample and its time `index * dt`. Ties keep the first. Refuses an
/// empty slice. (A crash pulse is a deceleration, so the peak is the minimum.)
pub fn peak(samples: &[f64], dt: f64) -> Result<(f64, f64), PulseError> {
    check_dt(dt)?;
    check_samples(samples)?;
    if samples.is_empty() {
        return Err(PulseError::Empty);
    }
    let mut best = 0;
    for (i, &v) in samples.iter().enumerate() {
        if v < samples[best] {
            best = i;
        }
    }
    Ok((samples[best], best as f64 * dt))
}

/// Pulse duration: the time from the first to the last sample whose magnitude is at least
/// `threshold` (`threshold` must be finite and >= 0), `(last - first) * dt`. `None` if no
/// sample reaches it. A pulse with a gap in the middle counts the gap.
pub fn duration(samples: &[f64], dt: f64, threshold: f64) -> Result<Option<f64>, PulseError> {
    check_dt(dt)?;
    check_samples(samples)?;
    if !(threshold.is_finite() && threshold >= 0.0) {
        return Err(PulseError::BadArgument("threshold must be finite and >= 0"));
    }
    let first = samples.iter().position(|v| v.abs() >= threshold);
    let last = samples.iter().rposition(|v| v.abs() >= threshold);
    Ok(match (first, last) {
        (Some(a), Some(b)) => Some((b - a) as f64 * dt),
        _ => None,
    })
}

/// Free-flight distance of the virtual occupant in the OLC definition, metres.
pub const OLC_FREE_FLIGHT_M: f64 = 0.065;
/// Distance over which the virtual occupant is then decelerated, metres.
pub const OLC_RESTRAINT_M: f64 = 0.235;

/// Occupant Load Criterion, in m/s^2 (divide by 9.80665 for g).
///
/// Definition (Kubler, Gargallo-Doerr et al., "Occupant Load Criterion", also used by
/// Euro NCAP's frontal protocols): `samples` is the vehicle acceleration (deceleration
/// negative), `v0` the initial vehicle speed in m/s (> 0, finite). A virtual occupant
/// starts at the vehicle speed and keeps it (free flight) while the vehicle slows, until
/// it has moved `OLC_FREE_FLIGHT_M` (65 mm) relative to the vehicle, time t1. From t1 it
/// is decelerated at a constant rate `a_o` such that its speed equals the vehicle's at
/// some t2 and the relative displacement from t1 to t2 is `OLC_RESTRAINT_M` (235 mm).
/// OLC is that `a_o`, returned as a positive number. Here t2 is the first time the
/// relative displacement reaches 235 mm, where `a_o = (v0 - v_vehicle(t2)) / (t2 - t1)`.
///
/// Vehicle speed and position come from the trapezoid integral of the samples; t1 and t2
/// are linearly interpolated between samples. Returns `None` if the occupant never travels
/// 65 mm or never completes the 235 mm within the data (pulse too short or too mild).
pub fn olc(samples: &[f64], dt: f64, v0: f64) -> Result<Option<f64>, PulseError> {
    check_dt(dt)?;
    check_samples(samples)?;
    if !(v0.is_finite() && v0 > 0.0) {
        return Err(PulseError::BadArgument("v0 must be finite and > 0"));
    }
    let n = samples.len();
    if n < 2 {
        return Ok(None);
    }
    // Vehicle speed vv[i] and position xv[i] at t = i dt (trapezoid), and the occupant's
    // relative displacement during free flight s[i] = integral of (v0 - vv).
    let mut vv = vec![v0; n];
    let mut xv = vec![0.0; n];
    let mut s = vec![0.0; n];
    for i in 1..n {
        vv[i] = vv[i - 1] + 0.5 * (samples[i - 1] + samples[i]) * dt;
        xv[i] = xv[i - 1] + 0.5 * (vv[i - 1] + vv[i]) * dt;
        s[i] = s[i - 1] + 0.5 * ((v0 - vv[i - 1]) + (v0 - vv[i])) * dt;
    }
    let lerp = |a: f64, b: f64, f: f64| a + (b - a) * f;

    // t1: first time the free-flying occupant is 65 mm ahead (as index + fraction).
    let i1 = match (1..n).find(|&i| s[i] >= OLC_FREE_FLIGHT_M) {
        Some(i) => i,
        None => return Ok(None),
    };
    let f1 = (OLC_FREE_FLIGHT_M - s[i1 - 1]) / (s[i1] - s[i1 - 1]);
    let t1 = (i1 - 1) as f64 * dt + f1 * dt;
    let vv1 = lerp(vv[i1 - 1], vv[i1], f1);
    let xv1 = lerp(xv[i1 - 1], xv[i1], f1);

    // D(t2) = relative displacement from t1 to t2 if the occupant is decelerated at the
    // constant rate that makes its speed meet the vehicle's at t2:
    //   tau (v0 + vv(t2)) / 2 - (xv(t2) - xv1),  tau = t2 - t1.
    let d_at = |t2: f64, vv2: f64, xv2: f64| (t2 - t1) * (v0 + vv2) / 2.0 - (xv2 - xv1);
    let (mut prev_t, mut prev_vv, mut prev_d) = (t1, vv1, 0.0);
    for j in i1..n {
        let t2 = j as f64 * dt;
        if t2 <= t1 {
            continue;
        }
        let d = d_at(t2, vv[j], xv[j]);
        if d >= OLC_RESTRAINT_M {
            let f = (OLC_RESTRAINT_M - prev_d) / (d - prev_d);
            let t2i = lerp(prev_t, t2, f);
            let vv2 = lerp(prev_vv, vv[j], f);
            return Ok(Some((v0 - vv2) / (t2i - t1)));
        }
        prev_t = t2;
        prev_vv = vv[j];
        prev_d = d;
    }
    Ok(None)
}
