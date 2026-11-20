//! Tracks and lap timing (Stage 7 m1, nb-43; docs/STAGE7.md "Shared definitions" and "The track
//! file"; the grammar is in docs/FORMAT.md, "The `.track` format").
//!
//! The track lies on the ground, y = 0: a point is (x, z). `Track::parse` is strict like the
//! `.bsn` parser: every refusal is a `ParseError` with a named `ErrorKind` and the line (and the
//! column when one token is at fault); nothing is defaulted. `LapTimer` is the only place the lap
//! rule lives: the race program and the viewer only report what it decides.

use crate::vec3::Vec3;
use std::fmt;

/// The only version this parser reads.
pub const VERSION: u32 = 1;

/// Which rule a refused file broke. Each is a named error (docs/FORMAT.md, "Refusals").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    /// Empty file, or the first line is not `track 1`.
    Header,
    /// A version other than 1.
    UnknownVersion,
    /// A `\r` anywhere (CR or CRLF line endings).
    CarriageReturn,
    /// A tab character.
    Tab,
    /// The file does not end with `\n`.
    MissingFinalNewline,
    /// Leading, trailing or doubled spaces, or a blank line made of spaces.
    Spacing,
    /// A keyword the grammar does not have.
    UnknownKeyword,
    /// A keyword before an earlier one, or a one-off keyword given twice.
    OutOfOrder,
    /// A missing or extra token, or a keyword that is not the expected one inside a line.
    Syntax,
    /// A token that is not a number, or not a plain decimal index.
    BadNumber,
    /// `nan`, `inf` or a number that overflows to infinity.
    NonFinite,
    /// A width that is zero or negative.
    NonPositiveWidth,
    /// `grid` below zero, or longer than the centreline loop.
    BadGrid,
    /// A required line is missing (`width`, `start`, `grid`, `checkpoint`).
    MissingLine,
    /// Fewer than 3 points.
    TooFewPoints,
    /// Two neighbouring points in the loop are the same point.
    ZeroLengthSegment,
    /// The centreline crosses, touches or doubles back over itself.
    SelfIntersecting,
    /// The start index is not a point.
    StartOutOfRange,
    /// A checkpoint index is not a point.
    CheckpointOutOfRange,
    /// A checkpoint equals the start index.
    CheckpointIsStart,
    /// Checkpoints are not strictly increasing in driving order from the start index.
    CheckpointsOutOfOrder,
}

/// Why a file was refused. `line` is 1-based; `column` is the 1-based character column of the
/// offending token when there is one.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub kind: ErrorKind,
    pub line: usize,
    pub column: Option<usize>,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.column {
            Some(c) => write!(f, "line {}, column {}: {}", self.line, c, self.message),
            None => write!(f, "line {}: {}", self.line, self.message),
        }
    }
}

impl std::error::Error for ParseError {}

fn err(kind: ErrorKind, line: usize, column: Option<usize>, message: String) -> ParseError {
    ParseError {
        kind,
        line,
        column,
        message,
    }
}

/// A track: a closed centreline with a width at every point, a start/finish index, checkpoint
/// indices and the grid distance.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    /// Centreline (x, z), in driving order; the loop closes from the last point to point 0.
    pub points: Vec<[f64; 2]>,
    /// Width (m) at each point: the file's one width, or that point's own override.
    pub widths: Vec<f64>,
    /// Index of the start/finish point.
    pub start: usize,
    /// Checkpoint indices, strictly increasing in driving order counting cyclically from `start`.
    pub checkpoints: Vec<usize>,
    /// The car starts this far before the start line, measured backwards along the centreline.
    pub grid: f64,
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn len(a: [f64; 2]) -> f64 {
    dot(a, a).sqrt()
}

/// Sign of the turn a -> b -> c: 1 left, -1 right, 0 straight (exact, on the given numbers).
fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> i32 {
    let v = cross(sub(b, a), sub(c, a));
    if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    }
}

/// c lies in the box of a-b (used only when a, b, c are collinear).
fn on_box(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    c[0] >= a[0].min(b[0])
        && c[0] <= a[0].max(b[0])
        && c[1] >= a[1].min(b[1])
        && c[1] <= a[1].max(b[1])
}

/// Closed segments a-b and c-d share at least one point (touching counts).
fn segments_meet(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let (o1, o2, o3, o4) = (
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    );
    if o1 != o2 && o3 != o4 {
        return true;
    }
    (o1 == 0 && on_box(a, b, c))
        || (o2 == 0 && on_box(a, b, d))
        || (o3 == 0 && on_box(c, d, a))
        || (o4 == 0 && on_box(c, d, b))
}

/// Why a token is not a number.
fn parse_f64(tok: &str, line: usize, col: usize, what: &str) -> Result<f64, ParseError> {
    match tok.parse::<f64>() {
        Err(_) => Err(err(
            ErrorKind::BadNumber,
            line,
            Some(col),
            format!("{what} `{tok}` is not a number"),
        )),
        Ok(v) if !v.is_finite() => Err(err(
            ErrorKind::NonFinite,
            line,
            Some(col),
            format!("{what} `{tok}` is not finite (nan, inf and overflow are refused)"),
        )),
        Ok(v) => Ok(v),
    }
}

/// A decimal index: digits only, no sign, no leading zero, at most 4294967295.
fn parse_index(tok: &str, line: usize, col: usize, what: &str) -> Result<usize, ParseError> {
    let ok = !tok.is_empty()
        && tok.bytes().all(|b| b.is_ascii_digit())
        && (tok == "0" || !tok.starts_with('0'))
        && tok.parse::<u32>().is_ok();
    if !ok {
        return Err(err(
            ErrorKind::BadNumber,
            line,
            Some(col),
            format!("{what} `{tok}` is not an index (decimal digits, no sign, no leading zero)"),
        ));
    }
    Ok(tok.parse::<u32>().unwrap_or(0) as usize)
}

/// Keyword rank: the grammar's order.
fn rank(kw: &str) -> Option<u8> {
    match kw {
        "width" => Some(0),
        "start" => Some(1),
        "grid" => Some(2),
        "checkpoint" => Some(3),
        "point" => Some(4),
        _ => None,
    }
}

impl Track {
    /// Parse a `.track` file. See docs/FORMAT.md for the grammar and every refusal.
    pub fn parse(text: &str) -> Result<Track, ParseError> {
        if let Some(pos) = text.find('\r') {
            let before = &text[..pos];
            let line = before.matches('\n').count() + 1;
            let col = before.rsplit('\n').next().unwrap_or("").chars().count() + 1;
            return Err(err(
                ErrorKind::CarriageReturn,
                line,
                Some(col),
                "carriage return: the file must use `\\n` line endings only".to_string(),
            ));
        }
        if text.is_empty() {
            return Err(err(
                ErrorKind::Header,
                1,
                None,
                "empty file: line 1 must be `track 1`".to_string(),
            ));
        }
        if !text.ends_with('\n') {
            let line = text.matches('\n').count() + 1;
            return Err(err(
                ErrorKind::MissingFinalNewline,
                line,
                None,
                "the file must end with a newline".to_string(),
            ));
        }
        let body = &text[..text.len() - 1];
        let lines: Vec<&str> = body.split('\n').collect();
        let last_line = lines.len();

        let mut width: Option<f64> = None;
        let mut start: Option<(usize, usize)> = None; // (index, line)
        let mut grid: Option<(f64, usize)> = None;
        let mut checkpoints: Vec<(usize, usize)> = Vec::new(); // (index, line)
        let mut points: Vec<[f64; 2]> = Vec::new();
        let mut overrides: Vec<Option<f64>> = Vec::new();
        let mut point_lines: Vec<usize> = Vec::new();
        let mut max_rank = 0u8;

        for (i, raw) in lines.iter().enumerate() {
            let ln = i + 1;
            if raw.contains('\t') {
                let col = raw.chars().take_while(|c| *c != '\t').count() + 1;
                return Err(err(
                    ErrorKind::Tab,
                    ln,
                    Some(col),
                    "tab character: tokens are separated by single spaces".to_string(),
                ));
            }
            if ln == 1 {
                match *raw {
                    "track 1" => {}
                    other => {
                        let mut it = other.split(' ');
                        if it.next() == Some("track") {
                            if let (Some(v), None) = (it.next(), it.next()) {
                                if v.parse::<u64>().is_ok() {
                                    return Err(err(
                                        ErrorKind::UnknownVersion,
                                        1,
                                        Some(7),
                                        format!(
                                            "unknown version {v}: this parser reads version 1 only"
                                        ),
                                    ));
                                }
                            }
                        }
                        return Err(err(
                            ErrorKind::Header,
                            1,
                            None,
                            "line 1 must be `track 1`".to_string(),
                        ));
                    }
                }
                continue;
            }
            if raw.is_empty() || raw.starts_with('#') {
                continue;
            }
            if raw.starts_with(' ') || raw.ends_with(' ') || raw.contains("  ") {
                return Err(err(
                    ErrorKind::Spacing,
                    ln,
                    None,
                    "tokens are separated by exactly one space (no leading, trailing or doubled spaces)".to_string(),
                ));
            }
            let toks: Vec<&str> = raw.split(' ').collect();
            // 1-based column of the start of token k.
            let col = |k: usize| -> usize {
                toks[..k]
                    .iter()
                    .map(|t| t.chars().count() + 1)
                    .sum::<usize>()
                    + 1
            };
            let kw = toks[0];
            let Some(r) = rank(kw) else {
                return Err(err(
                    ErrorKind::UnknownKeyword,
                    ln,
                    Some(1),
                    format!("unknown keyword `{kw}`"),
                ));
            };
            if r < max_rank {
                return Err(err(
                    ErrorKind::OutOfOrder,
                    ln,
                    Some(1),
                    format!("`{kw}` may not follow a later keyword (order: width, start, grid, checkpoint, point)"),
                ));
            }
            max_rank = r;
            match kw {
                "width" => {
                    if width.is_some() {
                        return Err(err(
                            ErrorKind::OutOfOrder,
                            ln,
                            Some(1),
                            "a second `width` line".to_string(),
                        ));
                    }
                    if toks.len() != 2 {
                        return Err(err(
                            ErrorKind::Syntax,
                            ln,
                            None,
                            "`width` takes exactly one value: `width <w>`".to_string(),
                        ));
                    }
                    let w = parse_f64(toks[1], ln, col(1), "width")?;
                    if w <= 0.0 {
                        return Err(err(
                            ErrorKind::NonPositiveWidth,
                            ln,
                            Some(col(1)),
                            format!("width `{}` must be greater than 0", toks[1]),
                        ));
                    }
                    width = Some(w);
                }
                "start" => {
                    if start.is_some() {
                        return Err(err(
                            ErrorKind::OutOfOrder,
                            ln,
                            Some(1),
                            "a second `start` line".to_string(),
                        ));
                    }
                    if toks.len() != 2 {
                        return Err(err(
                            ErrorKind::Syntax,
                            ln,
                            None,
                            "`start` takes exactly one index: `start <i>`".to_string(),
                        ));
                    }
                    start = Some((parse_index(toks[1], ln, col(1), "start index")?, ln));
                }
                "grid" => {
                    if grid.is_some() {
                        return Err(err(
                            ErrorKind::OutOfOrder,
                            ln,
                            Some(1),
                            "a second `grid` line".to_string(),
                        ));
                    }
                    if toks.len() != 2 {
                        return Err(err(
                            ErrorKind::Syntax,
                            ln,
                            None,
                            "`grid` takes exactly one distance: `grid <m>`".to_string(),
                        ));
                    }
                    let g = parse_f64(toks[1], ln, col(1), "grid distance")?;
                    if g < 0.0 {
                        return Err(err(
                            ErrorKind::BadGrid,
                            ln,
                            Some(col(1)),
                            format!("grid distance `{}` must be 0 or more", toks[1]),
                        ));
                    }
                    grid = Some((g, ln));
                }
                "checkpoint" => {
                    if toks.len() != 2 {
                        return Err(err(
                            ErrorKind::Syntax,
                            ln,
                            None,
                            "`checkpoint` takes exactly one index: `checkpoint <i>`".to_string(),
                        ));
                    }
                    checkpoints.push((parse_index(toks[1], ln, col(1), "checkpoint index")?, ln));
                }
                _ => {
                    // point <x> <z> [width <w>]
                    if toks.len() != 3 && toks.len() != 5 {
                        return Err(err(
                            ErrorKind::Syntax,
                            ln,
                            None,
                            "`point` is `point <x> <z>` or `point <x> <z> width <w>`".to_string(),
                        ));
                    }
                    let x = parse_f64(toks[1], ln, col(1), "x")?;
                    let z = parse_f64(toks[2], ln, col(2), "z")?;
                    let mut ov = None;
                    if toks.len() == 5 {
                        if toks[3] != "width" {
                            return Err(err(
                                ErrorKind::Syntax,
                                ln,
                                Some(col(3)),
                                format!("expected `width`, found `{}`", toks[3]),
                            ));
                        }
                        let w = parse_f64(toks[4], ln, col(4), "point width")?;
                        if w <= 0.0 {
                            return Err(err(
                                ErrorKind::NonPositiveWidth,
                                ln,
                                Some(col(4)),
                                format!("point width `{}` must be greater than 0", toks[4]),
                            ));
                        }
                        ov = Some(w);
                    }
                    points.push([x, z]);
                    overrides.push(ov);
                    point_lines.push(ln);
                }
            }
        }

        let Some(width) = width else {
            return Err(err(
                ErrorKind::MissingLine,
                last_line,
                None,
                "missing `width` line".to_string(),
            ));
        };
        let Some((start, start_line)) = start else {
            return Err(err(
                ErrorKind::MissingLine,
                last_line,
                None,
                "missing `start` line".to_string(),
            ));
        };
        let Some((grid, grid_line)) = grid else {
            return Err(err(
                ErrorKind::MissingLine,
                last_line,
                None,
                "missing `grid` line".to_string(),
            ));
        };
        if checkpoints.is_empty() {
            return Err(err(
                ErrorKind::MissingLine,
                last_line,
                None,
                "missing `checkpoint` line: at least one checkpoint is required".to_string(),
            ));
        }
        let n = points.len();
        if n < 3 {
            let line = point_lines.last().copied().unwrap_or(last_line);
            return Err(err(
                ErrorKind::TooFewPoints,
                line,
                None,
                format!("a track needs at least 3 points, found {n}"),
            ));
        }
        // Neighbouring points must differ (a zero-length segment has no direction).
        for i in 0..n {
            let j = (i + 1) % n;
            if points[i] == points[j] {
                let line = point_lines[j];
                return Err(err(
                    ErrorKind::ZeroLengthSegment,
                    line,
                    None,
                    format!("point {j} is the same point as point {i}: a zero-length segment"),
                ));
            }
        }
        // The loop must not cross, touch or double back over itself.
        for i in 0..n {
            let (a, b) = (points[i], points[(i + 1) % n]);
            for j in (i + 1)..n {
                let (c, d) = (points[j], points[(j + 1) % n]);
                let adjacent = j == i + 1 || (i == 0 && j == n - 1);
                let bad = if adjacent {
                    // The shared point is the only allowed contact: the two segments must not
                    // fold back over each other (same direction from the shared point).
                    let (shared, p, q) = if j == i + 1 { (b, a, d) } else { (a, b, c) };
                    orient(p, shared, q) == 0 && dot(sub(p, shared), sub(q, shared)) > 0.0
                } else {
                    segments_meet(a, b, c, d)
                };
                if bad {
                    return Err(err(
                        ErrorKind::SelfIntersecting,
                        point_lines[j],
                        None,
                        format!(
                            "the centreline meets itself: the segment from point {j} meets the segment from point {i} (line {})",
                            point_lines[i]
                        ),
                    ));
                }
            }
        }
        if start >= n {
            return Err(err(
                ErrorKind::StartOutOfRange,
                start_line,
                None,
                format!(
                    "start index {start} is out of range: the track has {n} points (0 to {})",
                    n - 1
                ),
            ));
        }
        let mut prev_offset = 0usize;
        for &(c, line) in &checkpoints {
            if c >= n {
                return Err(err(
                    ErrorKind::CheckpointOutOfRange,
                    line,
                    None,
                    format!(
                        "checkpoint index {c} is out of range: the track has {n} points (0 to {})",
                        n - 1
                    ),
                ));
            }
            if c == start {
                return Err(err(
                    ErrorKind::CheckpointIsStart,
                    line,
                    None,
                    format!("checkpoint {c} is the start index"),
                ));
            }
            let offset = (c + n - start) % n;
            if offset <= prev_offset {
                return Err(err(
                    ErrorKind::CheckpointsOutOfOrder,
                    line,
                    None,
                    format!("checkpoint {c} is out of order: checkpoints must increase in driving order counting from start index {start}"),
                ));
            }
            prev_offset = offset;
        }
        let widths: Vec<f64> = overrides.iter().map(|o| o.unwrap_or(width)).collect();
        let track = Track {
            points,
            widths,
            start,
            checkpoints: checkpoints.iter().map(|c| c.0).collect(),
            grid,
        };
        if grid > track.centre_length() {
            return Err(err(
                ErrorKind::BadGrid,
                grid_line,
                None,
                format!(
                    "grid distance {grid} is longer than the centreline loop ({})",
                    track.centre_length()
                ),
            ));
        }
        Ok(track)
    }

    /// Length of the closed centreline polygon (m).
    pub fn centre_length(&self) -> f64 {
        let n = self.points.len();
        (0..n)
            .map(|i| len(sub(self.points[(i + 1) % n], self.points[i])))
            .sum()
    }

    /// Smallest and largest width over the points.
    pub fn width_range(&self) -> (f64, f64) {
        let min = self.widths.iter().copied().fold(f64::INFINITY, f64::min);
        let max = self
            .widths
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    /// Unit tangent at point i: along p[i+1] - p[i-1] (indices wrap).
    pub fn tangent(&self, i: usize) -> [f64; 2] {
        let n = self.points.len();
        let d = sub(self.points[(i + 1) % n], self.points[(i + n - 1) % n]);
        let l = len(d);
        [d[0] / l, d[1] / l]
    }

    /// The gate at point i: the segment through p[i], perpendicular to the tangent there, as long
    /// as the width at i.
    pub fn gate(&self, i: usize) -> Gate {
        let t = self.tangent(i);
        let h = self.widths[i] / 2.0;
        let p = self.points[i];
        let nrm = [-t[1], t[0]];
        Gate {
            a: [p[0] - nrm[0] * h, p[1] - nrm[1] * h],
            b: [p[0] + nrm[0] * h, p[1] + nrm[1] * h],
            tangent: t,
        }
    }

    /// The grid position: `grid` metres before the start point, measured backwards along the
    /// centreline.
    pub fn grid_point(&self) -> [f64; 2] {
        let n = self.points.len();
        let mut remaining = self.grid;
        let mut i = self.start;
        loop {
            let prev = (i + n - 1) % n;
            let seg = len(sub(self.points[i], self.points[prev]));
            if remaining <= seg {
                let f = remaining / seg;
                let (p, q) = (self.points[i], self.points[prev]);
                return [p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f];
            }
            remaining -= seg;
            i = prev;
        }
    }

    /// On track: the distance from `pos` (x and z; y is ignored) to the centreline polyline is
    /// at most half the track width at the nearest point, the width being interpolated linearly
    /// along a segment whose two ends differ. A non-finite position is off track.
    pub fn on_track(&self, pos: Vec3) -> bool {
        let p = [pos.x, pos.z];
        let n = self.points.len();
        let mut best = f64::INFINITY;
        let mut best_half = 0.0;
        for i in 0..n {
            let j = (i + 1) % n;
            let (a, b) = (self.points[i], self.points[j]);
            let ab = sub(b, a);
            let f = (dot(sub(p, a), ab) / dot(ab, ab)).clamp(0.0, 1.0);
            let q = [a[0] + ab[0] * f, a[1] + ab[1] * f];
            let d = len(sub(p, q));
            if d < best {
                best = d;
                best_half = (self.widths[i] + (self.widths[j] - self.widths[i]) * f) / 2.0;
            }
        }
        best <= best_half
    }
}

/// A gate: a segment across the track and the direction of travel through it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gate {
    pub a: [f64; 2],
    pub b: [f64; 2],
    /// Unit tangent of the centreline at the gate (the forward direction).
    pub tangent: [f64; 2],
}

/// How one substep's straight move p0 -> p1 meets a gate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Crossing {
    /// Fraction along p0 -> p1 of the meeting point, 0 < s <= 1.
    pub s: f64,
    /// True when (p1 - p0) . tangent > 0.
    pub forward: bool,
}

impl Gate {
    /// The crossing of the move from `p0` to `p1` (x and z), if any. Half-open in s: a move that
    /// starts exactly on the gate (s = 0) does not cross it, one that ends exactly on it (s = 1)
    /// does, so a car that stops on the line crosses it once, not twice. A move parallel to the
    /// gate, or of zero length, never crosses.
    pub fn crossing(&self, p0: Vec3, p1: Vec3) -> Option<Crossing> {
        let a = [p0.x, p0.z];
        let r = [p1.x - p0.x, p1.z - p0.z];
        let e = sub(self.b, self.a);
        let d = cross(r, e);
        if d == 0.0 || !d.is_finite() {
            return None;
        }
        let q = sub(self.a, a);
        let s = cross(q, e) / d;
        let u = cross(q, r) / d;
        if s > 0.0 && s <= 1.0 && (0.0..=1.0).contains(&u) {
            Some(Crossing {
                s,
                forward: dot(r, self.tangent) > 0.0,
            })
        } else {
            None
        }
    }
}

/// Why a lap is not valid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidReason {
    MissedCheckpoint,
    OffTrack,
}

impl InvalidReason {
    /// The word printed on the `lap` line.
    pub fn as_str(self) -> &'static str {
        match self {
            InvalidReason::MissedCheckpoint => "missed_checkpoint",
            InvalidReason::OffTrack => "off_track",
        }
    }
}

/// A finished lap (valid or not).
#[derive(Clone, Debug, PartialEq)]
pub struct LapRecord {
    /// Lap number: every completed lap counts, valid or not, from 1.
    pub n: u32,
    /// The lap's two start-line crossing times.
    pub start_t: f64,
    pub end_t: f64,
    /// `round((end_t - start_t) * 1000)`: the one integer both the printed and the on-screen
    /// time are made from.
    pub time_ms: u64,
    /// None when valid; `MissedCheckpoint` is checked first.
    pub invalid: Option<InvalidReason>,
    /// Total time off track during the lap, summed over substeps.
    pub off_track_s: f64,
}

impl LapRecord {
    pub fn valid(&self) -> bool {
        self.invalid.is_none()
    }
}

/// An event one `update` call reports, in time order.
#[derive(Clone, Debug, PartialEq)]
pub enum LapEvent {
    /// The first forward crossing of the start line: lap 1 starts at `t`.
    Start { t: f64 },
    /// A forward crossing of the next expected checkpoint (`k` counts from 1).
    Checkpoint { lap: u32, k: usize, t: f64 },
    /// A lap ended at a forward crossing of the start line; the next lap starts at the same time.
    Lap(LapRecord),
}

/// Why `update` refused a step. The timer's state is unchanged when it does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerError {
    /// A position or time that is not finite.
    NonFinite,
    /// `t` is earlier than `prev_t`.
    TimeWentBackwards,
}

impl fmt::Display for TimerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TimerError::NonFinite => write!(f, "lap timer: a position or time is not finite"),
            TimerError::TimeWentBackwards => write!(f, "lap timer: t is earlier than prev_t"),
        }
    }
}

impl std::error::Error for TimerError {}

/// The lap in progress.
#[derive(Clone, Debug)]
struct Running {
    n: u32,
    start_t: f64,
    /// Index into `checkpoints` of the next checkpoint to cross (== len when all are crossed).
    next: usize,
    off_track_s: f64,
    was_off: bool,
}

/// Lap timing. The lap rule (docs/STAGE7.md "Shared definitions", rules 1 to 5) lives here and
/// nowhere else.
///
/// Off track is judged at the end position of every substep. A substep's time counts toward the
/// lap that its end position lies in, so when a substep contains a start-line crossing, the lap
/// that starts there gets it.
#[derive(Clone, Debug)]
pub struct LapTimer {
    track: Track,
    /// gates[0] is the start/finish gate; gates[1 + k] is checkpoint k.
    gates: Vec<Gate>,
    running: Option<Running>,
    laps: Vec<LapRecord>,
}

impl LapTimer {
    pub fn new(track: &Track) -> LapTimer {
        let mut gates = vec![track.gate(track.start)];
        gates.extend(track.checkpoints.iter().map(|&c| track.gate(c)));
        LapTimer {
            track: track.clone(),
            gates,
            running: None,
            laps: Vec::new(),
        }
    }

    /// Feed one substep: the car moved in a straight line from `prev_pos` (time `prev_t`) to `pos`
    /// (time `t`). Returns the events it caused, in time order. Crossing times are interpolated
    /// inside the step, `prev_t + s * (t - prev_t)`, never rounded to it.
    pub fn update(
        &mut self,
        prev_pos: Vec3,
        pos: Vec3,
        prev_t: f64,
        t: f64,
    ) -> Result<Vec<LapEvent>, TimerError> {
        let finite = [prev_pos.x, prev_pos.z, pos.x, pos.z, prev_t, t]
            .iter()
            .all(|v| v.is_finite());
        if !finite {
            return Err(TimerError::NonFinite);
        }
        if t < prev_t {
            return Err(TimerError::TimeWentBackwards);
        }
        let mut hits: Vec<(f64, usize, bool)> = Vec::new();
        for (gi, g) in self.gates.iter().enumerate() {
            if let Some(c) = g.crossing(prev_pos, pos) {
                hits.push((c.s, gi, c.forward));
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        let mut events = Vec::new();
        for (s, gi, forward) in hits {
            let tc = prev_t + s * (t - prev_t);
            if gi == 0 {
                if !forward {
                    continue; // rule 5: a backward crossing of the line never counts
                }
                if let Some(run) = self.running.take() {
                    let missed = run.next < self.track.checkpoints.len();
                    let invalid = if missed {
                        Some(InvalidReason::MissedCheckpoint)
                    } else if run.was_off {
                        Some(InvalidReason::OffTrack)
                    } else {
                        None
                    };
                    let rec = LapRecord {
                        n: run.n,
                        start_t: run.start_t,
                        end_t: tc,
                        time_ms: ((tc - run.start_t) * 1000.0).round() as u64,
                        invalid,
                        off_track_s: run.off_track_s,
                    };
                    self.laps.push(rec.clone());
                    events.push(LapEvent::Lap(rec));
                    self.running = Some(Running {
                        n: run.n + 1,
                        start_t: tc,
                        next: 0,
                        off_track_s: 0.0,
                        was_off: false,
                    });
                } else {
                    events.push(LapEvent::Start { t: tc });
                    self.running = Some(Running {
                        n: 1,
                        start_t: tc,
                        next: 0,
                        off_track_s: 0.0,
                        was_off: false,
                    });
                }
            } else if forward {
                if let Some(run) = self.running.as_mut() {
                    if run.next == gi - 1 {
                        run.next += 1;
                        events.push(LapEvent::Checkpoint {
                            lap: run.n,
                            k: run.next,
                            t: tc,
                        });
                    }
                }
            }
        }
        if let Some(run) = self.running.as_mut() {
            if !self.track.on_track(pos) {
                run.was_off = true;
                run.off_track_s += t - prev_t;
            }
        }
        Ok(events)
    }

    /// Every lap completed so far, in order.
    pub fn laps(&self) -> &[LapRecord] {
        &self.laps
    }

    /// The running lap's number and start time, or None before the first start-line crossing.
    pub fn current_lap(&self) -> Option<(u32, f64)> {
        self.running.as_ref().map(|r| (r.n, r.start_t))
    }

    /// The shortest valid lap, in milliseconds.
    pub fn best_valid_ms(&self) -> Option<u64> {
        self.laps
            .iter()
            .filter(|l| l.valid())
            .map(|l| l.time_ms)
            .min()
    }
}

/// `ms` as seconds with exactly 3 decimals, made from the integer (never through a float
/// format): 83456 -> `83.456`.
pub fn format_seconds(ms: u64) -> String {
    format!("{}.{:03}", ms / 1000, ms % 1000)
}

/// `ms` as `M:SS.mmm` (the HUD form): 83456 -> `1:23.456`.
pub fn format_clock(ms: u64) -> String {
    format!("{}:{:02}.{:03}", ms / 60000, (ms / 1000) % 60, ms % 1000)
}
