//! Strict reader for validation reference pulses (`<name>.pulse.csv` and `<name>.meta`), linear
//! resampling, and a per-metric error table for comparing a sim pulse with a reference.
//!
//! Self-contained and std only. Not registered in `lib.rs` here; a later task does that. Tests
//! include it with `#[path]`. The metric functions are passed in by the caller (n20-09 passes
//! n20-03's `pulse.rs` functions), so this file does not depend on that one.
//!
//! File formats (fixed by n20-02):
//! - pulse: header line `t_s,accel_mps2`, then one `time,acceleration` row per sample (s, m/s^2).
//! - meta: `key = value` lines; blank lines and lines starting with `#` are skipped.

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::Path;

/// Exact header of a pulse file.
pub const PULSE_HEADER: &str = "t_s,accel_mps2";
/// A pulse needs at least this many samples.
pub const MIN_SAMPLES: usize = 10;
/// A time column is uniform when every time is within this fraction of the sample step from the
/// straight line through the first and last time. (A 6-decimal CSV at 10 kHz rounds each time by
/// up to 0.5% of the step; 2% leaves room for that and still refuses a dropped or doubled row.)
pub const UNIFORM_TOL_FRAC: f64 = 0.02;
/// Slack, as a fraction of the step, when checking that a resample time base lies inside the data.
pub const RANGE_TOL_FRAC: f64 = 1e-6;

/// Keys every meta file must have.
pub const REQUIRED_KEYS: [&str; 5] = ["test", "mass_kg", "speed_kmh", "channel", "sample_rate_hz"];
/// Optional keys n20-02 writes; any other key is accepted but listed in `Meta::unknown_keys`.
pub const KNOWN_OPTIONAL_KEYS: [&str; 5] = [
    "vehicle",
    "channel_location",
    "prefilter",
    "url",
    "static_crush_m",
];

/// Why a reference file or a comparison was refused. Line numbers are 1-based file lines.
#[derive(Debug, Clone, PartialEq)]
pub enum RefError {
    /// The file could not be read.
    Io(String),
    /// Line 1 is not `t_s,accel_mps2`.
    BadHeader { found: String },
    /// A pulse row is not `time,acceleration` with two numbers.
    BadRow { line: usize, reason: String },
    /// A time or acceleration is NaN or infinite.
    NonFinite { line: usize },
    /// A time is not above the one before it.
    NonIncreasing { line: usize },
    /// A time is off the uniform grid by more than `UNIFORM_TOL_FRAC` of the step.
    NonUniform {
        line: usize,
        deviation_s: f64,
        tolerance_s: f64,
    },
    /// Fewer than `MIN_SAMPLES` samples.
    TooFewSamples { found: usize, last_line: usize },
    /// A meta line is not `key = value`.
    BadMetaLine { line: usize },
    /// A meta key is given twice.
    DuplicateKey { line: usize, key: String },
    /// A required meta key is absent.
    MissingKey(String),
    /// A meta value is empty or not a finite number above zero where one is needed.
    BadValue {
        line: usize,
        key: String,
        reason: String,
    },
    /// `resample` or `compare` got a bad step, a time base outside the data, or unequal lengths.
    BadArgument(String),
    /// A metric function passed to `compare` failed.
    Metric { which: &'static str, reason: String },
}

impl fmt::Display for RefError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RefError::Io(s) => write!(f, "cannot read file: {s}"),
            RefError::BadHeader { found } => {
                write!(f, "line 1: header must be `{PULSE_HEADER}`, found `{found}`")
            }
            RefError::BadRow { line, reason } => write!(f, "line {line}: {reason}"),
            RefError::NonFinite { line } => write!(f, "line {line}: value is not finite"),
            RefError::NonIncreasing { line } => {
                write!(f, "line {line}: time is not greater than the previous time")
            }
            RefError::NonUniform { line, deviation_s, tolerance_s } => write!(
                f,
                "line {line}: time is {deviation_s:e} s off a uniform grid, tolerance {tolerance_s:e} s ({}% of the step)",
                UNIFORM_TOL_FRAC * 100.0
            ),
            RefError::TooFewSamples { found, last_line } => write!(
                f,
                "line {last_line}: only {found} samples, at least {MIN_SAMPLES} are needed"
            ),
            RefError::BadMetaLine { line } => write!(f, "line {line}: expected `key = value`"),
            RefError::DuplicateKey { line, key } => write!(f, "line {line}: key `{key}` given twice"),
            RefError::MissingKey(k) => write!(f, "meta file lacks required key `{k}`"),
            RefError::BadValue { line, key, reason } => write!(f, "line {line}: key `{key}`: {reason}"),
            RefError::BadArgument(s) => write!(f, "{s}"),
            RefError::Metric { which, reason } => write!(f, "metric {which} failed: {reason}"),
        }
    }
}

impl std::error::Error for RefError {}

/// A uniformly sampled acceleration record: sample `i` is at `t0 + i * dt`.
#[derive(Debug, Clone, PartialEq)]
pub struct Pulse {
    pub t0: f64,
    pub dt: f64,
    pub accel: Vec<f64>,
}

impl Pulse {
    /// Time of sample `i`.
    pub fn time(&self, i: usize) -> f64 {
        self.t0 + i as f64 * self.dt
    }
    /// Time of the last sample.
    pub fn t_end(&self) -> f64 {
        self.time(self.accel.len() - 1)
    }
}

/// Read a pulse file. See `parse_pulse`.
pub fn load_pulse(path: impl AsRef<Path>) -> Result<Pulse, RefError> {
    let text = fs::read_to_string(path.as_ref())
        .map_err(|e| RefError::Io(format!("{}: {e}", path.as_ref().display())))?;
    parse_pulse(&text)
}

/// Parse pulse text. Refuses, naming the 1-based line: a wrong header, a row that is not two
/// numbers, a non-finite value, a time that does not increase, fewer than `MIN_SAMPLES` samples,
/// and a time column off the uniform grid by more than `UNIFORM_TOL_FRAC` of the step. The step
/// is (last time - first time) / (n - 1).
pub fn parse_pulse(text: &str) -> Result<Pulse, RefError> {
    let mut lines = text.lines();
    let header = lines.next().unwrap_or("");
    if header.trim() != PULSE_HEADER {
        return Err(RefError::BadHeader {
            found: header.trim().to_string(),
        });
    }
    let mut times: Vec<f64> = Vec::new();
    let mut accel: Vec<f64> = Vec::new();
    let mut last_line = 1;
    for (k, raw) in lines.enumerate() {
        let line = k + 2;
        last_line = line;
        let fields: Vec<&str> = raw.trim().split(',').collect();
        if fields.len() != 2 {
            return Err(RefError::BadRow {
                line,
                reason: format!("expected 2 comma-separated fields, found {}", fields.len()),
            });
        }
        let parse = |s: &str| -> Result<f64, RefError> {
            s.trim().parse::<f64>().map_err(|_| RefError::BadRow {
                line,
                reason: format!("`{}` is not a number", s.trim()),
            })
        };
        let (t, a) = (parse(fields[0])?, parse(fields[1])?);
        if !t.is_finite() || !a.is_finite() {
            return Err(RefError::NonFinite { line });
        }
        if let Some(&prev) = times.last() {
            if t <= prev {
                return Err(RefError::NonIncreasing { line });
            }
        }
        times.push(t);
        accel.push(a);
    }
    let n = times.len();
    if n < MIN_SAMPLES {
        return Err(RefError::TooFewSamples {
            found: n,
            last_line,
        });
    }
    let t0 = times[0];
    let dt = (times[n - 1] - t0) / (n - 1) as f64;
    let tol = UNIFORM_TOL_FRAC * dt;
    for (i, &t) in times.iter().enumerate() {
        let dev = (t - (t0 + i as f64 * dt)).abs();
        if dev > tol {
            return Err(RefError::NonUniform {
                line: i + 2,
                deviation_s: dev,
                tolerance_s: tol,
            });
        }
    }
    Ok(Pulse { t0, dt, accel })
}

/// A checked meta file.
#[derive(Debug, Clone, PartialEq)]
pub struct Meta {
    pub test: String,
    pub mass_kg: f64,
    pub speed_kmh: f64,
    pub channel: String,
    pub sample_rate_hz: f64,
    /// `static_crush_m` if the file has it (finite, above zero).
    pub static_crush_m: Option<f64>,
    /// Every key and value as read, in key order.
    pub values: BTreeMap<String, String>,
    /// Keys that are neither required nor in `KNOWN_OPTIONAL_KEYS`, in key order.
    pub unknown_keys: Vec<String>,
}

/// Read a meta file. See `parse_meta`.
pub fn load_meta(path: impl AsRef<Path>) -> Result<Meta, RefError> {
    let text = fs::read_to_string(path.as_ref())
        .map_err(|e| RefError::Io(format!("{}: {e}", path.as_ref().display())))?;
    parse_meta(&text)
}

/// Parse meta text: `key = value` per line (split at the first `=`, both sides trimmed; blank
/// lines and `#` lines skipped). Refuses a line without `=` or with an empty key or value, a
/// duplicate key, a missing required key, and `mass_kg`, `speed_kmh`, `sample_rate_hz` or
/// `static_crush_m` that is not a finite number above zero. Unknown keys are kept and listed.
pub fn parse_meta(text: &str) -> Result<Meta, RefError> {
    let mut values: BTreeMap<String, String> = BTreeMap::new();
    let mut line_of: BTreeMap<String, usize> = BTreeMap::new();
    for (k, raw) in text.lines().enumerate() {
        let line = k + 1;
        let s = raw.trim();
        if s.is_empty() || s.starts_with('#') {
            continue;
        }
        let Some((key, value)) = s.split_once('=') else {
            return Err(RefError::BadMetaLine { line });
        };
        let (key, value) = (key.trim(), value.trim());
        if key.is_empty() {
            return Err(RefError::BadMetaLine { line });
        }
        if value.is_empty() {
            return Err(RefError::BadValue {
                line,
                key: key.to_string(),
                reason: "value is empty".into(),
            });
        }
        if values.insert(key.to_string(), value.to_string()).is_some() {
            return Err(RefError::DuplicateKey {
                line,
                key: key.to_string(),
            });
        }
        line_of.insert(key.to_string(), line);
    }
    for key in REQUIRED_KEYS {
        if !values.contains_key(key) {
            return Err(RefError::MissingKey(key.to_string()));
        }
    }
    let number = |key: &str| -> Result<f64, RefError> {
        let line = line_of[key];
        let v: f64 = values[key].parse().map_err(|_| RefError::BadValue {
            line,
            key: key.to_string(),
            reason: format!("`{}` is not a number", values[key]),
        })?;
        if !v.is_finite() || v <= 0.0 {
            return Err(RefError::BadValue {
                line,
                key: key.to_string(),
                reason: format!("must be finite and above zero, got {}", values[key]),
            });
        }
        Ok(v)
    };
    let mass_kg = number("mass_kg")?;
    let speed_kmh = number("speed_kmh")?;
    let sample_rate_hz = number("sample_rate_hz")?;
    let static_crush_m = if values.contains_key("static_crush_m") {
        Some(number("static_crush_m")?)
    } else {
        None
    };
    let unknown_keys = values
        .keys()
        .filter(|k| {
            !REQUIRED_KEYS.contains(&k.as_str()) && !KNOWN_OPTIONAL_KEYS.contains(&k.as_str())
        })
        .cloned()
        .collect();
    Ok(Meta {
        test: values["test"].clone(),
        mass_kg,
        speed_kmh,
        channel: values["channel"].clone(),
        sample_rate_hz,
        static_crush_m,
        unknown_keys,
        values,
    })
}

/// Linear interpolation at `n` times `t_start + k * dt`. Refuses `dt` not finite and above zero,
/// `n == 0`, a non-finite `t_start`, and any time outside `[t0, t_end]` of the data (no
/// extrapolation; `RANGE_TOL_FRAC` of the data's own step is allowed for rounding). The result has
/// `t0 = t_start` and step `dt`. At a data time the result is the data value exactly.
pub fn resample_at(p: &Pulse, t_start: f64, dt: f64, n: usize) -> Result<Pulse, RefError> {
    if !(dt.is_finite() && dt > 0.0) {
        return Err(RefError::BadArgument(format!(
            "resample step must be finite and above zero, got {dt}"
        )));
    }
    if n == 0 || !t_start.is_finite() {
        return Err(RefError::BadArgument(
            "resample needs a finite start and at least one sample".into(),
        ));
    }
    if p.accel.len() < 2 {
        return Err(RefError::BadArgument(
            "pulse has fewer than 2 samples".into(),
        ));
    }
    let slack = RANGE_TOL_FRAC * p.dt;
    let t_last = t_start + (n - 1) as f64 * dt;
    if t_start < p.t0 - slack || t_last > p.t_end() + slack {
        return Err(RefError::BadArgument(format!(
            "time base [{t_start}, {t_last}] s is outside the data [{}, {}] s",
            p.t0,
            p.t_end()
        )));
    }
    let last = p.accel.len() - 1;
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let t = t_start + k as f64 * dt;
        let x = ((t - p.t0) / p.dt).clamp(0.0, last as f64);
        let i = (x.floor() as usize).min(last);
        let frac = x - i as f64;
        out.push(if frac == 0.0 || i == last {
            p.accel[i]
        } else {
            p.accel[i] + (p.accel[i + 1] - p.accel[i]) * frac
        });
    }
    Ok(Pulse {
        t0: t_start,
        dt,
        accel: out,
    })
}

/// Resample from the pulse's own first time at step `dt`, as many samples as fit inside the data
/// (`n = floor((t_end - t0) / dt) + 1`). Refuses a bad `dt`, or one longer than the record.
pub fn resample(p: &Pulse, dt: f64) -> Result<Pulse, RefError> {
    if !(dt.is_finite() && dt > 0.0) {
        return Err(RefError::BadArgument(format!(
            "resample step must be finite and above zero, got {dt}"
        )));
    }
    if p.accel.len() < 2 {
        return Err(RefError::BadArgument(
            "pulse has fewer than 2 samples".into(),
        ));
    }
    let span = p.t_end() - p.t0;
    let n = ((span / dt) + RANGE_TOL_FRAC).floor() as usize + 1;
    resample_at(p, p.t0, dt, n)
}

/// A metric function: samples and step in, a value or an error text out.
pub type MetricFn<T> = dyn Fn(&[f64], f64) -> Result<T, String>;

/// The metric functions `compare` applies to both pulses. Errors are reported as text.
pub struct MetricFns<'a> {
    /// delta-V of the samples (m/s).
    pub delta_v: &'a MetricFn<f64>,
    /// `(peak magnitude, time of peak in s)`.
    pub peak: &'a MetricFn<(f64, f64)>,
    /// Duration in s, or `None` when the pulse never reaches the threshold.
    pub duration: &'a MetricFn<Option<f64>>,
}

/// One metric of both pulses and how they differ.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MetricError {
    pub sim: f64,
    pub reference: f64,
    /// `sim - reference`.
    pub abs_err: f64,
    /// `(sim - reference) / reference`; `Some(0.0)` when both are 0, `None` when only the
    /// reference is 0.
    pub rel_err: Option<f64>,
}

impl MetricError {
    pub fn new(sim: f64, reference: f64) -> Self {
        let abs_err = sim - reference;
        let rel_err = if reference != 0.0 {
            Some(abs_err / reference)
        } else if sim == 0.0 {
            Some(0.0)
        } else {
            None
        };
        MetricError {
            sim,
            reference,
            abs_err,
            rel_err,
        }
    }
}

/// The error table n20-09 gates on. `duration` is `None` when either pulse has no duration (never
/// reaches the threshold); `duration_sim` and `duration_ref` then say which.
#[derive(Debug, Clone, PartialEq)]
pub struct Comparison {
    pub delta_v: MetricError,
    pub peak: MetricError,
    pub peak_time: MetricError,
    pub duration: Option<MetricError>,
    pub duration_sim: Option<f64>,
    pub duration_ref: Option<f64>,
}

/// Compare a sim pulse with a reference pulse, both already on the same time base `dt` and of
/// equal length (resample first). The metric functions come from the caller.
pub fn compare(
    sim: &[f64],
    reference: &[f64],
    dt: f64,
    fns: &MetricFns,
) -> Result<Comparison, RefError> {
    if !(dt.is_finite() && dt > 0.0) {
        return Err(RefError::BadArgument(format!(
            "dt must be finite and above zero, got {dt}"
        )));
    }
    if sim.len() != reference.len() || sim.is_empty() {
        return Err(RefError::BadArgument(format!(
            "pulses must be non-empty and of equal length, got {} and {}",
            sim.len(),
            reference.len()
        )));
    }
    fn m<T>(which: &'static str, r: Result<T, String>) -> Result<T, RefError> {
        r.map_err(|reason| RefError::Metric { which, reason })
    }
    let (dv_s, dv_r) = (
        m("delta_v", (fns.delta_v)(sim, dt))?,
        m("delta_v", (fns.delta_v)(reference, dt))?,
    );
    let (pk_s, pk_r) = (
        m("peak", (fns.peak)(sim, dt))?,
        m("peak", (fns.peak)(reference, dt))?,
    );
    let (du_s, du_r) = (
        m("duration", (fns.duration)(sim, dt))?,
        m("duration", (fns.duration)(reference, dt))?,
    );
    Ok(Comparison {
        delta_v: MetricError::new(dv_s, dv_r),
        peak: MetricError::new(pk_s.0, pk_r.0),
        peak_time: MetricError::new(pk_s.1, pk_r.1),
        duration: match (du_s, du_r) {
            (Some(s), Some(r)) => Some(MetricError::new(s, r)),
            _ => None,
        },
        duration_sim: du_s,
        duration_ref: du_r,
    })
}
