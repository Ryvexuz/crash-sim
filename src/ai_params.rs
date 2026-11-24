//! Strict reader for `scenarios/ai_driver.params` (provided by a later slice).
//!
//! Grammar and refusal order:
//! - Refuse CR anywhere, empty input, then a missing final LF.
//! - On each line refuse tabs first; line 1 must be exactly `ai 1`.
//! - Skip empty lines and lines beginning with `#`; otherwise require single spaces,
//!   exactly `key value`, a known unrepeated key, and a finite Rust `f64`.
//! - After the last line require every key, in canonical order.
//!
//! A comment is only a whole line beginning with `#`. Put each value's reason on
//! its own comment line above the key, never after the value.
//! Nothing is defaulted, rounded, scaled, or range-checked. Range rules remain in
//! `race::run` and `speed_profile::profile`. Callers use `AiParams::line_of` to map
//! `RaceError::Driver(field)` or `ProfileError::BadParameter(name)` to a line.

use crate::ai::DriverParams;
use crate::race::ProfileParams;

/// Every key, in canonical order (driver first, then profile).
pub const KEYS: [&str; 11] = [
    "lookahead_k",
    "lookahead_min",
    "lookahead_max",
    "kp",
    "ki",
    "integral_max",
    "mu",
    "g",
    "a_accel",
    "a_brake",
    "v_max",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Header,
    UnknownVersion,
    CarriageReturn,
    Tab,
    MissingFinalNewline,
    Spacing,
    Syntax,
    UnknownKey,
    RepeatedKey,
    MissingKey,
    BadNumber,
    NonFinite,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub kind: ErrorKind,
    /// 1-based line.
    pub line: usize,
    /// 1-based character column of the offending token, when available.
    pub column: Option<usize>,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.column {
            Some(c) => write!(f, "line {}, column {}: {}", self.line, c, self.message),
            None => write!(f, "line {}: {}", self.line, self.message),
        }
    }
}

impl std::error::Error for ParseError {}

#[derive(Clone, Copy, Debug)]
pub struct AiParams {
    pub driver: DriverParams,
    pub profile: ProfileParams,
    /// `lines[i]` is the 1-based line of `KEYS[i]`.
    pub lines: [usize; 11],
}

impl AiParams {
    /// The line of `key`, or `None` if it is not in `KEYS`.
    pub fn line_of(&self, key: &str) -> Option<usize> {
        KEYS.iter().position(|&k| k == key).map(|i| self.lines[i])
    }
}

fn error(kind: ErrorKind, line: usize, column: Option<usize>, message: &str) -> ParseError {
    ParseError {
        kind,
        line,
        column,
        message: message.to_owned(),
    }
}

/// Parse the complete file, preserving each key's source line.
pub fn parse(text: &str) -> Result<AiParams, ParseError> {
    let mut line = 1;
    let mut column = 1;
    for c in text.chars() {
        if c == '\r' {
            return Err(error(
                ErrorKind::CarriageReturn,
                line,
                Some(column),
                "carriage return is forbidden",
            ));
        }
        if c == '\n' {
            line += 1;
            column = 1;
        } else {
            column += 1;
        }
    }
    if text.is_empty() {
        return Err(error(ErrorKind::Header, 1, None, "expected ai 1"));
    }
    if !text.ends_with('\n') {
        return Err(error(
            ErrorKind::MissingFinalNewline,
            line,
            None,
            "expected final newline",
        ));
    }
    let last_line = line - 1;
    let mut lines = [None; 11];
    let mut values = [0.0; 11];
    for (index, content) in text.split_terminator('\n').enumerate() {
        let line = index + 1;
        if let Some(c) = content.chars().position(|c| c == '\t') {
            return Err(error(ErrorKind::Tab, line, Some(c + 1), "tab is forbidden"));
        }
        if line == 1 {
            if content != "ai 1" {
                let version = content
                    .strip_prefix("ai ")
                    .filter(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()));
                if let Some(v) = version.and_then(|s| s.parse::<u64>().ok()) {
                    if v != 1 {
                        return Err(error(
                            ErrorKind::UnknownVersion,
                            line,
                            Some(4),
                            "unsupported ai version",
                        ));
                    }
                }
                return Err(error(ErrorKind::Header, line, None, "expected ai 1"));
            }
            continue;
        }
        if content.is_empty() || content.starts_with('#') {
            continue;
        }
        if content.starts_with(' ') || content.ends_with(' ') || content.contains("  ") {
            return Err(error(
                ErrorKind::Spacing,
                line,
                None,
                "expected single spaces",
            ));
        }
        let mut tokens = content.split(' ');
        let key = tokens.next().unwrap_or("");
        let value = match (tokens.next(), tokens.next()) {
            (Some(value), None) => value,
            _ => return Err(error(ErrorKind::Syntax, line, None, "expected key value")),
        };
        let Some(i) = KEYS.iter().position(|&k| k == key) else {
            return Err(error(ErrorKind::UnknownKey, line, Some(1), "unknown key"));
        };
        if let Some(first) = lines[i] {
            return Err(error(
                ErrorKind::RepeatedKey,
                line,
                Some(1),
                &format!("{key} was first given on line {first}"),
            ));
        }
        let column = Some(key.chars().count() + 2);
        let value = value
            .parse::<f64>()
            .map_err(|_| error(ErrorKind::BadNumber, line, column, "invalid number"))?;
        if !value.is_finite() {
            return Err(error(
                ErrorKind::NonFinite,
                line,
                column,
                "non-finite number",
            ));
        }
        values[i] = value;
        lines[i] = Some(line);
    }
    for (i, source) in lines.iter().enumerate() {
        if source.is_none() {
            return Err(error(
                ErrorKind::MissingKey,
                last_line,
                None,
                &format!("missing key {}", KEYS[i]),
            ));
        }
    }
    Ok(AiParams {
        driver: DriverParams {
            lookahead_k: values[0],
            lookahead_min: values[1],
            lookahead_max: values[2],
            kp: values[3],
            ki: values[4],
            integral_max: values[5],
        },
        profile: ProfileParams {
            mu: values[6],
            g: values[7],
            a_accel: values[8],
            a_brake: values[9],
            v_max: values[10],
        },
        // Every entry was checked above; no caller input can reach this with a missing key.
        lines: lines.map(|source| source.unwrap_or(0)),
    })
}
