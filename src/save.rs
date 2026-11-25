//! The racing save file: the best lap per track, as text. Version 2 is current; version 1 files
//! (no car, no date) still load and are filled with `unknown`. Saving always writes version 2.
//!
//! ```text
//! save 2
//! best <track id> <seconds> <car id> <date YYYY-MM-DD>
//! ```
//!
//! Version 1 has `best <track id> <seconds>` lines under `save 1`. The parser is strict: every
//! malformed input is a `SaveError::Parse` naming the line, never a silent default. Lines are
//! LF-terminated, one `best` line per track, sorted by track id, so `write(&parse(x)?)` is
//! byte-identical to `x` for a canonical version 2 file.
//!
//! `Save::validate` is the one rule set: `parse` (so `load`), `record_lap` and `save` all use it.
//! Overlapping writers of the same save file are not supported (see `write_atomic`).
//!
//! Self-contained (std only); registered in `lib.rs` as `pub mod save`.

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

/// The version `write` produces.
pub const VERSION: u32 = 2;
/// The placeholder a v1 file's missing car and date become.
pub const UNKNOWN: &str = "unknown";

/// One track's best lap.
#[derive(Clone, Debug, PartialEq)]
pub struct Best {
    pub seconds: f64,
    pub car: String,
    pub date: String,
}

/// The whole save: best lap per track id.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Save {
    pub best: BTreeMap<String, Best>,
}

/// Why a save could not be read, written or updated.
#[derive(Debug)]
pub enum SaveError {
    /// A malformed file. `line` is 1-based.
    Parse { line: usize, message: String },
    /// A bad value passed to `record_lap`.
    Invalid(String),
    /// A file system error, with the path involved.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
}

impl fmt::Display for SaveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SaveError::Parse { line, message } => write!(f, "line {line}: {message}"),
            SaveError::Invalid(m) => write!(f, "{m}"),
            SaveError::Io { path, source } => write!(f, "{}: {source}", path.display()),
        }
    }
}

impl std::error::Error for SaveError {}

fn perr(line: usize, message: impl Into<String>) -> SaveError {
    SaveError::Parse {
        line,
        message: message.into(),
    }
}

/// An id (track or car): non-empty, printable ASCII, no spaces.
fn check_id(what: &str, s: &str) -> Result<(), String> {
    if s.is_empty() {
        return Err(format!("{what} is empty"));
    }
    if !s.bytes().all(|b| b.is_ascii_graphic()) {
        return Err(format!(
            "{what} {s:?} must be printable ASCII without spaces"
        ));
    }
    Ok(())
}

/// `YYYY-MM-DD` as a real calendar date (year 0001..=9999), or `unknown`.
fn check_date(s: &str) -> Result<(), String> {
    if s == UNKNOWN {
        return Ok(());
    }
    let bad = || format!("date {s:?} must be YYYY-MM-DD or {UNKNOWN}");
    let b = s.as_bytes();
    if !s.is_ascii() || b.len() != 10 || b[4] != b'-' || b[7] != b'-' {
        return Err(bad());
    }
    let digits = |r: std::ops::Range<usize>| -> Option<u32> {
        let part = &s[r];
        if part.bytes().all(|c| c.is_ascii_digit()) {
            part.parse().ok()
        } else {
            None
        }
    };
    let (y, m, d) = match (digits(0..4), digits(5..7), digits(8..10)) {
        (Some(y), Some(m), Some(d)) => (y, m, d),
        _ => return Err(bad()),
    };
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let days = match m {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 => {
            if leap {
                29
            } else {
                28
            }
        }
        _ => return Err(bad()),
    };
    if y == 0 || d == 0 || d > days {
        return Err(bad());
    }
    Ok(())
}

/// A plain decimal (digits, optionally `.digits`), positive and finite: the only form `write`
/// produces, so a canonical file round-trips byte for byte.
fn parse_seconds(s: &str) -> Result<f64, String> {
    let (int, frac) = match s.split_once('.') {
        Some((i, f)) => (i, Some(f)),
        None => (s, None),
    };
    let all_digits = |p: &str| !p.is_empty() && p.bytes().all(|c| c.is_ascii_digit());
    if !all_digits(int) || frac.is_some_and(|f| !all_digits(f)) {
        return Err(format!("seconds {s:?} must be a plain decimal number"));
    }
    let v: f64 = s
        .parse()
        .map_err(|_| format!("seconds {s:?} is not a number"))?;
    check_seconds(v)?;
    Ok(v)
}

/// The one rule set for an entry, used by `Save::validate` (so by `save`), by `parse` for every
/// line (so by `load`) and by `record_lap`: ids, date, and positive finite seconds that survive
/// write -> parse.
fn check_entry(track: &str, b: &Best) -> Result<(), String> {
    check_id("track id", track)?;
    check_seconds(b.seconds)?;
    // The canonical form is the contract: a value that would not survive write -> parse is refused.
    if parse_seconds(&b.seconds.to_string())
        .map(|v| v != b.seconds)
        .unwrap_or(true)
    {
        return Err(format!("seconds {} cannot be written exactly", b.seconds));
    }
    check_id("car id", &b.car)?;
    check_date(&b.date)
}

fn check_seconds(v: f64) -> Result<(), String> {
    if !v.is_finite() {
        return Err(format!("seconds {v} must be finite"));
    }
    if v <= 0.0 {
        return Err(format!("seconds {v} must be positive"));
    }
    Ok(())
}

impl Save {
    pub fn new() -> Save {
        Save::default()
    }

    /// Parse a version 1 or version 2 file.
    pub fn parse(text: &str) -> Result<Save, SaveError> {
        if text.is_empty() {
            return Err(perr(1, "empty file: expected `save 1` or `save 2`"));
        }
        if !text.ends_with('\n') {
            let n = text.split('\n').count();
            return Err(perr(n, "file must end with a newline"));
        }
        let body = &text[..text.len() - 1];
        let mut version = 0u32;
        let mut save = Save::new();
        for (i, line) in body.split('\n').enumerate() {
            let n = i + 1;
            if line.contains('\r') {
                return Err(perr(n, "carriage return: files use LF line endings"));
            }
            if i == 0 {
                version = match line {
                    "save 1" => 1,
                    "save 2" => 2,
                    _ => {
                        let msg = if let Some(rest) = line.strip_prefix("save ") {
                            format!(
                                "unsupported version {:?}: this reads save 1 and save 2",
                                rest
                            )
                        } else {
                            "first line must be `save 1` or `save 2`".to_string()
                        };
                        return Err(perr(n, msg));
                    }
                };
                continue;
            }
            // Tokens split on single spaces only, so doubled or trailing spaces are refused.
            let toks: Vec<&str> = line.split(' ').collect();
            if toks[0] != "best" {
                return Err(perr(
                    n,
                    format!("unknown line {:?}: expected `best ...`", toks[0]),
                ));
            }
            let want = if version == 1 { 3 } else { 5 };
            if toks.len() != want {
                let shape = if version == 1 {
                    "best <track id> <seconds>"
                } else {
                    "best <track id> <seconds> <car id> <date>"
                };
                return Err(perr(
                    n,
                    format!(
                        "save {version} line needs `{shape}`, found {} fields",
                        toks.len()
                    ),
                ));
            }
            check_id("track id", toks[1]).map_err(|m| perr(n, m))?;
            let seconds = parse_seconds(toks[2]).map_err(|m| perr(n, m))?;
            let (car, date) = if version == 1 {
                (UNKNOWN.to_string(), UNKNOWN.to_string())
            } else {
                (toks[3].to_string(), toks[4].to_string())
            };
            let entry = Best { seconds, car, date };
            check_entry(toks[1], &entry).map_err(|m| perr(n, m))?;
            if save.best.insert(toks[1].to_string(), entry).is_some() {
                return Err(perr(
                    n,
                    format!("duplicate track {:?}: one best line per track", toks[1]),
                ));
            }
        }
        if version == 0 {
            return Err(perr(1, "missing header"));
        }
        Ok(save)
    }

    /// Check every entry against the one rule set `parse` applies to each line, so a save that
    /// passes is a save `load` accepts. `best` is a public field, so `save` calls this too.
    pub fn validate(&self) -> Result<(), SaveError> {
        for (track, b) in &self.best {
            check_entry(track, b)
                .map_err(|m| SaveError::Invalid(format!("track {track:?}: {m}")))?;
        }
        Ok(())
    }

    /// The version 2 text, canonical: tracks sorted, LF endings, a final newline.
    pub fn to_text(&self) -> String {
        let mut out = format!("save {VERSION}\n");
        for (track, b) in &self.best {
            out.push_str(&format!(
                "best {track} {} {} {}\n",
                b.seconds, b.car, b.date
            ));
        }
        out
    }

    /// Record a lap. Keeps the best (strictly faster replaces; a tie keeps the old entry).
    /// Returns whether the entry changed. Bad values are refused before anything changes.
    pub fn record_lap(
        &mut self,
        track: &str,
        seconds: f64,
        car: &str,
        date: &str,
    ) -> Result<bool, SaveError> {
        let entry = Best {
            seconds,
            car: car.to_string(),
            date: date.to_string(),
        };
        check_entry(track, &entry).map_err(SaveError::Invalid)?;
        if let Some(old) = self.best.get(track) {
            if seconds >= old.seconds {
                return Ok(false);
            }
        }
        self.best.insert(track.to_string(), entry);
        Ok(true)
    }

    /// Read and parse a save file.
    pub fn load(path: &Path) -> Result<Save, SaveError> {
        let text = fs::read_to_string(path).map_err(|source| SaveError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Save::parse(&text)
    }

    /// Validate, then write the save atomically (always version 2): see `write_atomic`. Invalid
    /// data is refused with `SaveError::Invalid` before any file is touched.
    pub fn save(&self, path: &Path) -> Result<(), SaveError> {
        self.validate()?;
        write_atomic(path, self.to_text().as_bytes())
    }
}

/// The temp file `write_atomic` uses for `path`: the same folder, the file name plus `.tmp`.
pub fn temp_path(path: &Path) -> PathBuf {
    let mut name = path
        .file_name()
        .map(|n| n.to_os_string())
        .unwrap_or_default();
    name.push(".tmp");
    path.with_file_name(name)
}

/// Write `bytes` to a temp file next to `path`, flush it to disk, then rename it over `path`.
/// On any failure the temp file is removed and an existing `path` is left as it was.
pub fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), SaveError> {
    let tmp = temp_path(path);
    let io = |p: &Path| {
        let p = p.to_path_buf();
        move |source| SaveError::Io { path: p, source }
    };
    let result = (|| {
        // A leftover temp from an earlier crash is removed first (removing a symlink or hard link
        // drops the name, never the file it points at); a directory at that name fails. The new
        // temp is then created exclusively, so it can never be an existing file or link.
        match fs::remove_file(&tmp) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(io(&tmp)(e)),
        }
        let mut f = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&tmp)
            .map_err(io(&tmp))?;
        let r = f
            .write_all(bytes)
            .and_then(|_| f.flush())
            .and_then(|_| f.sync_all())
            .map_err(io(&tmp));
        drop(f);
        if let Err(e) = r {
            let _ = fs::remove_file(&tmp);
            return Err(e);
        }
        fs::rename(&tmp, path).map_err(|e| {
            let _ = fs::remove_file(&tmp);
            io(path)(e)
        })
    })();
    result
}
